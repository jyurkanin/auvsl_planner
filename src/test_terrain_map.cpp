#include <iostream>
#include <thread>
#include <vector>
#include <math.h>

#include "auvsl_planner_node.h"
#include "GlobalParams.h"
#include "PlannerVisualizer.h"
#include "GlobalPlanner.h"
#include "JackalDynamicSolver.h"
#include "OctoTerrainMap.h"


#include <std_srvs/Empty.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <ompl/util/RandomNumbers.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

#include <rbdl/rbdl.h>

#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/OccupancyGrid.h>


unsigned got_init_pose = 0;
geometry_msgs::Pose initial_pose;

unsigned got_grid_map = 0;
geometry_msgs::Pose origin;
float map_res;
unsigned height;
unsigned width;

void get_pos_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  initial_pose = msg->pose.pose;
  got_init_pose = 1;
}


void get_grid_map(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  height = msg->info.height;
  width = msg->info.width;
  map_res = msg->info.resolution;
  origin = msg->info.origin;

  ROS_INFO("grid map header frame %s", msg->header.frame_id.c_str());
  ROS_INFO("Res %f  Cols %u Rows %u", map_res, width, height);
  ROS_INFO("Occ_grid origin %f,%f      end  %f,%f", origin.position.x, origin.position.y,     origin.position.x + (width*map_res), origin.position.y + (height*map_res));
  
  std::ofstream log_file;  
  log_file.open("/home/justin/state_valid_grid.csv", std::ofstream::out);
  log_file << "x,y,occupancy\n";


  float *blurred_occ_grid = new float[height*width];
  const int kernel_size = 10;

  for(int i = 0; i < height; i++){
    for(int j = 0; j < width; j++){
      blurred_occ_grid[(i*width) + j] = msg->data[(i*width) + j];
    }
  }
  
  int occupancy;
  float sigma_sq = 16;
  
  for(int i = kernel_size; i < height-kernel_size; i++){
    for(int j = kernel_size; j < width-kernel_size; j++){
      occupancy = msg->data[(i*width) + j];
      
      float dist_sq;
      float weight;
      float total_weight = 0;
      float sum = 0;
      for(int k = -kernel_size; k <= kernel_size; k++){
        for(int m = -kernel_size; m <= kernel_size; m++){
          dist_sq = (k*k) + (m*m);
          weight = expf(-.5*dist_sq/sigma_sq);
          sum += weight*msg->data[((i+k)*width) + j+m];
          total_weight += weight;
          
          //ROS_INFO("Dist %f   weight %f", dist_sq, weight);          
        }
      }
      //ROS_INFO("Sum %f    total_weight %f", sum, total_weight);
      
      blurred_occ_grid[(i*width) + j] = sum / total_weight;
    }
  }

  float blur_occ;
  for(int i = 0; i < height; i++){
      for(int j = 0; j < width; j++){
        blur_occ = blurred_occ_grid[(i*width) + j];
        if(blur_occ > 20){
          log_file << (j*map_res) + origin.position.x << "," << (i*map_res) + origin.position.y << "," << blur_occ << "\n";
        }
      }
  }
  
  ROS_INFO("The fun is done.");
  
  log_file.close();
  delete blurred_occ_grid;
  got_grid_map = 1;  
}


int main(int argc, char **argv){
  ROS_INFO("Starting up test_terrain_node\n");
  
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  
  GlobalParams::load_params(&nh);
  ompl::RNG::setSeed(GlobalParams::get_seed());
  
  ros::Rate loop_rate(10);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf(tf_buffer);
  costmap_2d::Costmap2DROS costmap("my_costmap", tf_buffer); //This does nothing apparently. I think it needs a lot more work to set it to subscribe to everything and recieve an actual costmap. But I don't know how and I can't find out.

  
  ros::Subscriber grid_map_sub = nh.subscribe("/map", 100, get_grid_map);
  while(!got_grid_map){
    ros::spinOnce();
    loop_rate.sleep();
  }
  grid_map_sub.shutdown();
  ROS_INFO("Got costmap");
  
  
  ros::ServiceClient localization_srv = nh.serviceClient<std_srvs::Empty>("/rtabmap/set_mode_localization");
  std_srvs::Empty empty_srv;
  localization_srv.waitForExistence();
  if(localization_srv.call(empty_srv)){
      ROS_INFO("Localization mode set");
  }
  else{
      ROS_INFO("Failed to set Localization mode set");
  }
  
  ROS_INFO("Getting initial pose");
  ros::Subscriber initial_pose_sub = nh.subscribe("/rtabmap/localization_pose", 100, get_pos_callback);
  while(!got_init_pose){
    ros::spinOnce();
    loop_rate.sleep();
  }
  initial_pose_sub.shutdown();
  ROS_INFO("Got initial pose");
  
  geometry_msgs::PoseStamped startp;
  geometry_msgs::PoseStamped goalp;

  startp.pose = initial_pose;
  goalp.pose.position.x = 8;
  goalp.pose.position.y = 0;
  goalp.pose.position.z = .16;

  goalp.pose.orientation.x = 0;
  goalp.pose.orientation.y = 0;
  goalp.pose.orientation.z = 0;
  goalp.pose.orientation.w = 0;

  std::vector<geometry_msgs::PoseStamped> plan;

  /*
  ROS_INFO("Getting OctoTerrainMap");
  OctoTerrainMap terrain_map(costmap.getCostmap());
  ROS_INFO("Constructed terrain map");
  auvsl::GlobalPlanner planner;
  planner.initialize("planner", &costmap);
  planner.makePlan(startp, goalp, plan);
  */
  /*
  std::ofstream log_file;
  log_file.open("/home/justin/circle_point.csv", std::ofstream::out);
  log_file << "x,y,alt\n";

  float x;
  float y;
  float alt = 0;
  for(int j = 0; j < height; j++){
      for(int i = 0; i < width; i++){
        x = (i*map_res) + origin.position.x;
        y = (j*map_res) + origin.position.y;
        
        alt = terrain_map.averageNeighbors(x, y, alt);
        log_file << x << "," << y << "," << alt << "\n";
      }
  }
  
  log_file.close();
  */
  

  
  
  //JackalDynamicSolver::set_terrain_map((TerrainMap*) &terrain_map);
  //JackalDynamicSolver::init_model(2);

  /*
  JackalDynamicSolver solver;
  
  float start_state[21];
  float end_state[21];
  
  start_state[0] = initial_pose.position.x;
  start_state[1] = initial_pose.position.y;
  start_state[2] = initial_pose.position.z;
  
  start_state[3] = initial_pose.orientation.x;
  start_state[4] = initial_pose.orientation.y;
  start_state[5] = initial_pose.orientation.z;
  start_state[10] = initial_pose.orientation.w;
  
  start_state[6] = 0;
  start_state[7] = 0;
  start_state[8] = 0;
  start_state[9] = 0;
  
  for(unsigned i = 11; i < 21; i++){
    start_state[i] = 0;
  }
  
  ROS_INFO("Starting Sim");
  solver.solve(start_state, end_state, 5, 5, 10);
  ROS_INFO("Sim is done");
  
  ROS_INFO("Final position: %f %f %f", end_state[0], end_state[1], end_state[2]);
  */
  
  JackalDynamicSolver::del_model();
}
