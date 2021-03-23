#include "GlobalParams.h"



float GlobalParams::fuzzy_constant_speed;
float GlobalParams::max_angular_vel;
float GlobalParams::timestep;
int GlobalParams::debug_level;
float GlobalParams::planner_resolution;
float GlobalParams::state_checker_resolution;
float GlobalParams::propagation_step_size;
float GlobalParams::visualize_planner;
float GlobalParams::goal_bias;
std::vector<float> GlobalParams::distance_weights;


void GlobalParams::load_params(ros::NodeHandle *nh){
  nh->getParam("/fuzzy_constant_speed", fuzzy_constant_speed);
  nh->getParam("/max_angular_vel", max_angular_vel);
  nh->getParam("/timestep", timestep);
  nh->getParam("/debug_level", debug_level);
  nh->getParam("/planner_resolution", planner_resolution);
  nh->getParam("/state_checker_resolution", state_checker_resolution);

  nh->getParam("/propagation_step_size", propagation_step_size);
  nh->getParam("/visualize_planner", visualize_planner);
  nh->getParam("/goal_bias", goal_bias);
  nh->getParam("/distance_weights", distance_weights);
  
}




float GlobalParams::get_fuzzy_constant_speed(){return GlobalParams::fuzzy_constant_speed;}
float GlobalParams::get_max_angular_vel(){return GlobalParams::max_angular_vel;}
float GlobalParams::get_timestep(){return GlobalParams::timestep;}
int   GlobalParams::get_debug_level(){return GlobalParams::debug_level;}
float GlobalParams::get_planner_resolution(){return GlobalParams::planner_resolution;}
float GlobalParams::get_state_checker_resolution(){return GlobalParams::state_checker_resolution;}
float GlobalParams::get_propagation_step_size(){return GlobalParams::propagation_step_size;}
float GlobalParams::get_visualize_planner(){return GlobalParams::visualize_planner;}
float GlobalParams::get_goal_bias(){return GlobalParams::goal_bias;}
std::vector<float> GlobalParams::get_distance_weights(){return GlobalParams::distance_weights;}
