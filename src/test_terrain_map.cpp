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

OctoTerrainMap *terrain_map;

void get_pos_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  initial_pose = msg->pose.pose;
  got_init_pose = 1;
}

float evaluateSimulatedTrajectory(){
  //Step 1. Read Odometry into an array for searching.
  ROS_INFO("Evaluating");
  std::ifstream odom_file("/home/justin/code/AUVSL_ROS/bags/rantoul/bags/rantoul_2_odom_10hz.csv");
  std::string line;
  std::stringstream ss;
  char comma;
  unsigned num_rows = 0;
  float *X_odom;

  ROS_INFO("Reading File");
  while(odom_file.peek() != EOF){
    std::getline(odom_file, line);
    num_rows++;
  }
  
  odom_file.clear();
  odom_file.seekg(0);
  
  X_odom = new float[3*num_rows];

  unsigned timestamp;
  ROS_INFO("X_odom");
  for(unsigned i = 0; i < num_rows; i++){
    std::getline(odom_file, line);
    ss.str(line);
    ss.clear();

    unsigned row = 3*i;

    ss >> timestamp >> comma;
    ss >> X_odom[row+0] >> comma;
    ss >> X_odom[row+1] >> comma;
    ss >> X_odom[row+2];// >> comma;
  }
  
  
  ROS_INFO("sim xout file");
  //Step 2. Open and prepare xout.csv to be read.
  std::ifstream sim_xout_file("/home/justin/xout_file.csv");
  std::getline(sim_xout_file, line); //remove first line
  
  float sim_x, sim_y, sim_z, ignoreme;

  ROS_INFO("parse sim xou file");
  while(sim_xout_file.peek() != EOF){
    std::getline(sim_xout_file, line);
    ss.str(line);
    ss.clear();
    
    ss >> ignoreme >> comma; //skip quaternion components
    ss >> ignoreme >> comma;
    ss >> ignoreme >> comma;
    ss >> ignoreme >> comma;
    
    ss >> sim_x >> comma;
    ss >> sim_y >> comma;
    ss >> sim_z;  // >> comma;
    
  }
  
  ROS_INFO("dxdy");
  float dx = sim_x - X_odom[(3*(num_rows-1))+0];
  float dy = sim_y - X_odom[(3*(num_rows-1))+1];
  
  delete[] X_odom;
  return sqrtf((dx*dx) + (dy*dy));
}

void record_altitude_under_path(TerrainMap *terrain_map){
  std::string line;
  std::stringstream ss;
  char comma;
  std::ifstream sim_xout_file("/home/justin/xout_file.csv");
  std::getline(sim_xout_file, line);
  
  float sim_x, sim_y, sim_z, ignoreme, alt;
  std::ofstream alt_log("/home/justin/alt_file.csv");
  alt_log << "r,alt\n";

  alt = 0;
  
  while(sim_xout_file.peek() != EOF){
    std::getline(sim_xout_file, line);
    ss.str(line);
    ss.clear();
    
    ss >> ignoreme >> comma; //skip quaternion components
    ss >> ignoreme >> comma;
    ss >> ignoreme >> comma;
    ss >> ignoreme >> comma;
    
    ss >> sim_x >> comma;
    ss >> sim_y >> comma;
    ss >> sim_z;   // >> comma;
    
    alt = terrain_map->getAltitude(sim_x, sim_y, alt);
    alt_log << sqrtf(sim_x*sim_x + sim_y*sim_y) << ',' << alt << "\n";
    
  }
  
  sim_xout_file.close();
  alt_log.close();
  
}

void searchSoilParams(){
  std::ifstream odom_file("/home/justin/code/AUVSL_ROS/bags/rantoul/bags/rantoul_2_odom_10hz.csv");
  std::string line;
  std::stringstream ss;
  char comma;
  unsigned num_rows = 0;
  unsigned timestamp;
  float x_real, y_real;
  
  ROS_INFO("Reading File");
  while(odom_file.peek() != EOF){
    std::getline(odom_file, line);
    ss.str(line);
    ss.clear();
    
    ss >> timestamp >> comma;
    ss >> x_real >> comma;
    ss >> y_real >> comma;
  }
  
  
  JackalDynamicSolver solver;
  BekkerData best_bekker_params;
  
  terrain_map->test_bekker_data_ = lookup_soil_table(0);
  terrain_map->test_bekker_data_.name = "Search";
  
  float X_final[21];
  float dx, dy;
  float err;
  float best_err = 1000000;
  for(float kc = 27; kc < 30; kc+=.1){
    for(float kphi = 500; kphi < 1000; kphi+=100){
      terrain_map->test_bekker_data_.kc = kc;
      terrain_map->test_bekker_data_.kphi = kphi;
      terrain_map->test_bekker_data_.n0 = .6;
      terrain_map->test_bekker_data_.n1 = 0;
      terrain_map->test_bekker_data_.phi = 22.5*M_PI/180.0;
      
      solver.simulateRealTrajectory("/home/justin/code/AUVSL_ROS/bags/rantoul/bags/rantoul_odom_2.csv", "/home/justin/code/AUVSL_ROS/bags/rantoul/bags/rantoul_2_joint_states_50hz.csv", X_final);
      dx = x_real - X_final[0];
      dy = y_real - X_final[1];
      err = sqrtf((dx*dx)+(dy*dy));
      //err = evaluateSimulatedTrajectory();
      ROS_INFO("Error %f", err);
      if(err < best_err){
        best_bekker_params = terrain_map->test_bekker_data_;
        ROS_INFO("New Best");
        ROS_INFO("New Best");
        ROS_INFO("New Best");
        best_err = err;
      }
      ROS_INFO("kc %f, kphi %f, n0 %f, n1 %f, phi %f", best_bekker_params.kc, best_bekker_params.kphi, best_bekker_params.n0, best_bekker_params.n1, best_bekker_params.phi);
    }
  }
  
  ROS_INFO("kc %f, kphi %f, n0 %f, n1 %f, phi %f", terrain_map->test_bekker_data_.kc, terrain_map->test_bekker_data_.kphi, terrain_map->test_bekker_data_.n0, terrain_map->test_bekker_data_.n1, terrain_map->test_bekker_data_.phi);
  
}

int main(int argc, char **argv){
  
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;

  ROS_INFO("Starting up test_terrain_node\n");
  
  int plot_res;
  nh.getParam("/TerrainMap/plot_res", plot_res); 
  
  GlobalParams::load_params(&nh);
  ompl::RNG::setSeed(GlobalParams::get_seed());
  
  ros::Rate loop_rate(10);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf(tf_buffer);
  costmap_2d::Costmap2DROS costmap("my_costmap", tf_buffer); //This does nothing apparently. I think it needs a lot more work to set it to subscribe to everything and recieve an actual costmap. But I don't know how and I can't find out.
  
  ros::ServiceClient localization_srv = nh.serviceClient<std_srvs::Empty>("/rtabmap/set_mode_localization");
  std_srvs::Empty empty_srv;
  localization_srv.waitForExistence();
  if(localization_srv.call(empty_srv)){
      ROS_INFO("Localization mode set");
  }
  else{
      ROS_INFO("Failed to set Localization mode set");
  }
  
  
  ROS_INFO("Getting OctoTerrainMap");
  terrain_map = new OctoTerrainMap(costmap.getCostmap());
  //SimpleTerrainMap terrain_map;
  ROS_INFO("Constructed terrain map");

  
  std::ofstream log_file;
  log_file.open("/home/justin/occ.csv", std::ofstream::out);
  log_file << "x,y,occ\n";
  
  //map_res = .1;
  //height = 100;
  //width = 100;

  /*
  float x;
  float y;
  float alt = 0;
  float occ;
  unsigned idx;
  for(int j = 0; j < terrain_map->rows_; j++){
      idx = j*terrain_map->cols_;
      for(int i = 0; i < terrain_map->cols_; i++){
          x = (i*terrain_map->map_res_) + terrain_map->x_origin_;
          y = (j*terrain_map->map_res_) + terrain_map->y_origin_;
        
          occ = terrain_map->occ_grid_blur_[idx+i];
          log_file << x << "," << y << "," << occ << "\n";
      }
  }
  
  log_file.close();
  */
  
  //25.000000, kphi 500.000000, n0 0.600000, n1 0.000000, phi 0.392699
  //kc 29.000000, kphi 500.000000, n0 0.600000, n1 0.000000, phi 0.392699
  terrain_map->test_bekker_data_.kc = 29;
  terrain_map->test_bekker_data_.kphi = 500;
  terrain_map->test_bekker_data_.n0 = .6;
  terrain_map->test_bekker_data_.n1 = 0;
  terrain_map->test_bekker_data_.phi = .392699;

  float X_final[21];
  JackalDynamicSolver::set_terrain_map((TerrainMap*) terrain_map);
  JackalDynamicSolver::init_model(2);
  JackalDynamicSolver solver;
  solver.simulateRealTrajectory("/home/justin/code/AUVSL_ROS/bags/rantoul/bags/rantoul_odom_2.csv", "/home/justin/code/AUVSL_ROS/bags/rantoul/bags/rantoul_2_joint_states_50hz.csv", X_final);
  
  ROS_INFO("Final Position %f %f", X_final[0], X_final[1]);
  
  //searchSoilParams();
  //record_altitude_under_path((TerrainMap*) &terrain_map);
  
  
  JackalDynamicSolver::del_model();
  
  
  ROS_INFO("test_terrain is exiting");
}
