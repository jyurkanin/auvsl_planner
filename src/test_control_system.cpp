#include <iostream>
#include <thread>
#include <vector>
#include <unistd.h>

#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "GlobalParams.h"
#include "ControlSystem.h"
#include "JackalDynamicSolver.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


#include <ompl/util/RandomNumbers.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

#include <rbdl/rbdl.h>



using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

//using namespace auvsl_planner;


//launch-prefix="gdb -ex run --args

//This function is for later on when other nodes may want to request a global path plan
//bool globalPlannerCallback(GlobalPathPlan::Request &req, GlobalPathPlan::Response &resp){
//This functionality is provided by the move_base/navigation packages.


geometry_msgs::Pose vehicle_pose;

void get_pos_callback(const nav_msgs::Odometry::ConstPtr& msg){
  vehicle_pose = msg->pose.pose;
}


int main(int argc, char **argv){
  ROS_INFO("Starting up test_cs_node\n");
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  GlobalParams::load_params(&nh);
  
  ros::Rate loop_rate(100);
  
  //unsigned num_pts = 59;
  //float waypoints[num_pts*2] = {2.041539,-0.412668,2.141262,-0.512668,2.240985,-0.612668,2.340709,-0.712668,2.440432,-0.812668,2.540155,-0.912668,2.639878,-0.912668,2.739602,-1.012668,2.839324,-1.112668,2.939048,-1.212668,3.038771,-1.312668,3.138494,-1.412668,3.238217,-1.412668,3.337941,-1.412668,3.437664,-1.412668,3.537387,-1.412668,3.637110,-1.412668,3.736834,-1.412668,3.836557,-1.412668,3.936280,-1.412668,4.036003,-1.412668,4.135726,-1.312668,4.235449,-1.312668,4.335173,-1.212668,4.434896,-1.212668,4.534619,-1.212668,4.634342,-1.212668,4.734066,-1.212668,4.833789,-1.112668,4.933512,-1.012668,5.033235,-0.912668,5.132958,-0.812668,5.232682,-0.712668,5.332405,-0.612668,5.432128,-0.512668,5.531851,-0.412668,5.631574,-0.312668,5.731297,-0.212668,5.831021,-0.112668,5.930744,-0.012668,6.030467,-0.012668,6.130190,-0.012668,6.229914,-0.012668,6.329637,-0.012668,6.429360,-0.012668,6.529083,-0.012668,6.628807,-0.012668,6.728529,-0.012668,6.828253,-0.012668,6.927976,-0.012668,7.027699,-0.012668,7.127423,-0.012668,7.227146,-0.012668,7.326869,-0.012668,7.426592,-0.012668,7.526315,-0.012668,7.626038,-0.012668,7.725762,-0.012668,7.825485,-0.012668};
  unsigned num_pts = 2;
  float waypoints[num_pts*2] = {-0.387035, 9.735388,  -0.387035, 9.735388};
  
  float dist, best_dist;
  float dx, dy;
  float v_forward, v_angular, vl, vr;
  unsigned idx = 0;
  unsigned best_idx;
  unsigned prev_idx = 0;
  
  std::vector<Eigen::Vector2f> lookahead;
  
  auvsl::ControlSystem *control_system = new auvsl::AnfisControlSystem();
  control_system->initialize();

  TerrainMap *terrain_map = new SimpleTerrainMap();
  JackalDynamicSolver::set_terrain_map((TerrainMap*) terrain_map);
  JackalDynamicSolver::init_model(2);
  JackalDynamicSolver solver;
  //               x,y,z,  qx,qy,qz,  q1,q2,q3,q4,  qw,  vx,vy,vz,  ax,ay,az,  qd1,qd2,qd3,qd4
  float X_n[21] = {0,0,.16,  0,0,0,      0,0,0,0,     1,   0,0,0,     0,0,0,     0,0,0,0};
  float X_n1[21];
  
  lookahead.push_back(Eigen::Vector2f(0,0));
  lookahead.push_back(Eigen::Vector2f(waypoints[0], waypoints[1]));
  lookahead.push_back(Eigen::Vector2f(waypoints[2], waypoints[3]));
  
  geometry_msgs::Pose pose;
  
  do{
    pose.position.x = X_n[0];
    pose.position.y = X_n[1];
    pose.position.z = X_n[2];

    pose.orientation.x = X_n[3];
    pose.orientation.y = X_n[4];
    pose.orientation.z = X_n[5];
    pose.orientation.w = X_n[10];

    lookahead[0][0] = X_n[0];
    lookahead[0][1] = X_n[1];
    
    control_system->computeVelocityCommand(lookahead, pose, v_forward, v_angular);
    ROS_INFO("Position %f %f  Current %f %f   Target %f %f   future %f %f   Control  %f %f", X_n[0], X_n[1], lookahead[0][0], lookahead[0][1],   lookahead[1][0], lookahead[1][1],     lookahead[2][0], lookahead[2][1],    v_forward, v_angular);
    
    float base_width = solver.base_size[0];
    vl = (v_forward - v_angular*(base_width/2.0))/solver.tire_radius;
    vr = (v_forward + v_angular*(base_width/2.0))/solver.tire_radius;
    
    //ROS_INFO("Vf W: %f %f\tVl Vr: %f %f", v_forward, v_angular,  vl, vr);
    //X_n[17] = X_n[18] = vr;
    //X_n[19] = X_n[20] = vl;
    solver.solve(X_n,X_n1,  vl,vr,  .01f);
    
    //ROS_INFO("Position: %f %f", X_n[0], X_n[1]);
    
    for(unsigned i = 0; i < 21; i++){
      X_n[i] = X_n1[i];
      //ROS_INFO("Xn  %u   %f", i, X_n[i]);
    }
    
    //Reasses waypoints. Get closest next waypoint
    best_idx = idx;
    dx = waypoints[(idx*2)+0] - X_n1[0];
    dy = waypoints[(idx*2)+1] - X_n1[1];
    
    best_dist = sqrtf(dx*dx + dy*dy);// + atan2f(dy,dx);
    ROS_INFO("Dist to wp %f", best_dist);

    if(best_dist < 1){
      break;
    }
    
    for(unsigned i = idx+1; i < num_pts; i++){
      dx = waypoints[(i*2)+0] - X_n1[0];
      dy = waypoints[(i*2)+1] - X_n1[1];
      dist = sqrtf(dx*dx + dy*dy);
      if(dist < best_dist){
        best_dist = dist;
        best_idx = i;
      }
    }
    
    if(best_idx != idx){
      prev_idx = idx;
      idx = best_idx;
      
      lookahead[0][0] = waypoints[(prev_idx*2)];
      lookahead[0][1] = waypoints[(prev_idx*2) + 1];
      
      lookahead[1][0] = waypoints[(idx*2) + 2];
      lookahead[1][1] = waypoints[(idx*2) + 3];

      if(idx == num_pts-2){
        lookahead[2][0] = waypoints[(idx*2) + 2];
        lookahead[2][1] = waypoints[(idx*2) + 3];
      }
      else{
        lookahead[2][0] = waypoints[(idx*2) + 4];
        lookahead[2][1] = waypoints[(idx*2) + 5];
      }
      idx++;
    }
    
    //loop_rate.sleep();
  } while(idx < (num_pts-1));
  
  JackalDynamicSolver::del_model();
  
  return 0;
}



//test control system in gazebo.
int main_gazebo(int argc, char **argv){
  ROS_INFO("Starting up auvsl_planner_node\n");
  
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  GlobalParams::load_params(&nh);
  
  ros::Rate loop_rate(100);
  
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  geometry_msgs::Twist msg;
  
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0;
  
  cmd_vel_pub.publish(msg);
  
  ros::ServiceClient localization_srv = nh.serviceClient<std_srvs::Empty>("/rtabmap/set_mode_localization");
  std_srvs::Empty empty_srv;
  localization_srv.waitForExistence();
  if(localization_srv.call(empty_srv)){
      ROS_INFO("Localization mode set");
  }
  else{
      ROS_INFO("Failed to set Localization mode set");
  }
  
  ros::Subscriber pose_sub = nh.subscribe("/odometry/filtered", 100, get_pos_callback);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  unsigned num_pts = 59;
  float waypoints[num_pts*2] = {2.041539,-0.412668,2.141262,-0.512668,2.240985,-0.612668,2.340709,-0.712668,2.440432,-0.812668,2.540155,-0.912668,2.639878,-0.912668,2.739602,-1.012668,2.839324,-1.112668,2.939048,-1.212668,3.038771,-1.312668,3.138494,-1.412668,3.238217,-1.412668,3.337941,-1.412668,3.437664,-1.412668,3.537387,-1.412668,3.637110,-1.412668,3.736834,-1.412668,3.836557,-1.412668,3.936280,-1.412668,4.036003,-1.412668,4.135726,-1.312668,4.235449,-1.312668,4.335173,-1.212668,4.434896,-1.212668,4.534619,-1.212668,4.634342,-1.212668,4.734066,-1.212668,4.833789,-1.112668,4.933512,-1.012668,5.033235,-0.912668,5.132958,-0.812668,5.232682,-0.712668,5.332405,-0.612668,5.432128,-0.512668,5.531851,-0.412668,5.631574,-0.312668,5.731297,-0.212668,5.831021,-0.112668,5.930744,-0.012668,6.030467,-0.012668,6.130190,-0.012668,6.229914,-0.012668,6.329637,-0.012668,6.429360,-0.012668,6.529083,-0.012668,6.628807,-0.012668,6.728529,-0.012668,6.828253,-0.012668,6.927976,-0.012668,7.027699,-0.012668,7.127423,-0.012668,7.227146,-0.012668,7.326869,-0.012668,7.426592,-0.012668,7.526315,-0.012668,7.626038,-0.012668,7.725762,-0.012668,7.825485,-0.012668};
  float dist, best_dist;
  float dx, dy;
  float v_forward, v_angular;
  unsigned idx = 0;
  unsigned best_idx;
  unsigned prev_idx = 0;
  
  std::vector<Eigen::Vector2f> lookahead;
  
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  
  auvsl::ControlSystem *control_system = new auvsl::AnfisControlSystem();
  control_system->initialize();
  
  lookahead.push_back(Eigen::Vector2f(vehicle_pose.position.x, vehicle_pose.position.y));
  lookahead.push_back(Eigen::Vector2f(waypoints[0], waypoints[1]));
  lookahead.push_back(Eigen::Vector2f(waypoints[2], waypoints[3]));
  
  do{
    control_system->computeVelocityCommand(lookahead, vehicle_pose, v_forward, v_angular);  
    ROS_INFO("Position %f %f  Current %f %f   Target %f %f   future %f %f   Control  %f %f", vehicle_pose.position.x, vehicle_pose.position.y, lookahead[0][0], lookahead[0][1],   lookahead[1][0], lookahead[1][1],     lookahead[2][0], lookahead[2][1],    v_forward, v_angular);
    
    msg.linear.x = v_forward;
    msg.angular.z = v_angular;
    
    cmd_vel_pub.publish(msg);

    //Reasses waypoints. Get closest next waypoint
    best_idx = idx;
    dx = waypoints[(idx*2)+0] - vehicle_pose.position.x;
    dy = waypoints[(idx*2)+1] - vehicle_pose.position.y;
    best_dist = sqrtf(dx*dx + dy*dy);// + atan2f(dy,dx);
    for(unsigned i = idx+1; i < num_pts; i++){
      dx = waypoints[(i*2)+0] - vehicle_pose.position.x;
      dy = waypoints[(i*2)+1] - vehicle_pose.position.y;
      dist = sqrtf(dx*dx + dy*dy);
      if(dist < best_dist){
        best_dist = dist;
        best_idx = i;
      }
    }
    
    if(best_idx != idx){
      prev_idx = idx;
      idx = best_idx;
      
      lookahead[0][0] = waypoints[(prev_idx*2)];
      lookahead[0][1] = waypoints[(prev_idx*2) + 1];
      
      lookahead[1][0] = waypoints[(idx*2) + 2];
      lookahead[1][1] = waypoints[(idx*2) + 3];

      if(idx == num_pts-2){
        lookahead[2][0] = waypoints[(idx*2) + 2];
        lookahead[2][1] = waypoints[(idx*2) + 3];
      }
      else{
        lookahead[2][0] = waypoints[(idx*2) + 4];
        lookahead[2][1] = waypoints[(idx*2) + 5];
      }
      idx++;
    }
    
    loop_rate.sleep();
  } while(idx < (num_pts-1));
  
  return 0;
}

