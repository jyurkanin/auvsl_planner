#include <ros/ros.h>
#include <std_msgs/String.h>

#include "VehicleStateSpace.h"
#include "GlobalParams.h"
#include "JackalStatePropagator.h"
#include "PlannerVisualizer.h"
#include "TerrainMap.h"
#include "GlobalPlanner.h"
#include "DStarPlanner.h"

#include <ompl/base/goals/GoalSpace.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

#include <rbdl/rbdl.h>

#include <stdlib.h>
#include <climits>
#include <math.h>

ompl::base::StateSamplerPtr sampler;
ompl::control::ControlSamplerPtr control_sampler;

ompl::control::SpaceInformationPtr si;
ompl::base::VehicleStateSpace *space;
ompl::control::RealVectorControlSpace *cspace;


/*
 * Test if vehicle state space is a metric space
 * Test for positive definiteness
 * Test for symmetry
 */
void test_vehicle_space(){
  
  space->sanityChecks();
  
  ompl::base::State *state1 = si->allocState();
  ompl::base::State *state2 = si->allocState();
  ompl::base::State *state3 = si->allocState();

  ROS_INFO("Starting Metric Space Test");
  for(int i = 0; i < 1000; i++){
    sampler->sampleUniform(state1);
    sampler->sampleUniform(state2);
    sampler->sampleUniform(state3);

    const double *s1 = static_cast<const ompl::base::VehicleStateSpace::StateType *>(state1)->values;
    const double *s2 = static_cast<const ompl::base::VehicleStateSpace::StateType *>(state2)->values;
    const double *s3 = static_cast<const ompl::base::VehicleStateSpace::StateType *>(state3)->values;

    float temp1 = space->distance(state1, state2);
    float temp2 = space->distance(state2, state3);
    float temp3 = space->distance(state1, state3);
    if((temp1 + temp2) < temp3){
      ROS_INFO("Fails Triangle inequality\n %f %f %f     %f %f %f %f     %f %f %f %f      %f %f %f %f", temp1, temp2, temp3, s1[0], s1[1], s1[3], s1[4],    s2[0], s2[1], s2[3], s2[4],     s3[0], s3[1], s3[3], s3[4]);
      
    }
    if(space->distance(state1, state2) != space->distance(state1, state2)){
      ROS_INFO("Fails Symmetry");
    }
    if(space->distance(state1, state1) != 0){
       ROS_INFO("Fails Zero Property");
    }
    if(space->distance(state1, state2) <= 0){
      ROS_INFO("Not positive definite");
    }
    
    
    
  }
  
  si->freeState(state1);
  si->freeState(state2);
  
  ROS_INFO("Metric space test completed");
}



void test_quaternion_math(){
  float test_val, qz, qw, siny_cosp, cosy_cosp, result1, result2;

  float eps = 1e-5;
  
  ROS_INFO("Doing Quaternion test");
  srand(123);
  
  for(int i = 0; i < 1000; i++){
    test_val = M_PI + (M_PI*((float) rand() / INT_MAX));
    
    qz = sinf(test_val/2.0f); //qz
    qw = cosf(test_val/2.0f); //qw
    
    siny_cosp = 2 * (qw*qz);    //Ripped from wikipedia
    cosy_cosp = 1 - 2 * (qz*qz);
    
    result1 = fmod((2*M_PI) + atan2f(siny_cosp, cosy_cosp), 2*M_PI);
    if(fabs(result1 - test_val) > eps){
      ROS_INFO("Failed Quaternion Conversion Test1 %f %f", test_val, result1);
    }

    result2 = fmod((2*M_PI) + (2*atan2f(qz, qw)), 2*M_PI);
    if(fabs(result2 - test_val) > eps){
      ROS_INFO("Failed Quaternion Conversion Test2 %f %f", test_val, result2);
    }
  }

  ROS_INFO("Quaternion test done %f", test_val);
}


//Compare the dynamic model to state propagator (which uses the dynamic model)
//Just to make sure everything is doing what I expect
void test_dynamic_model(){
  JackalDynamicSolver::init_model(0);
  JackalDynamicSolver solver;
  
  SimpleTerrainMap *terrain_map = new SimpleTerrainMap();
  terrain_map->generateObstacles();
  terrain_map->generateUnknownObstacles();
  
  JackalDynamicSolver::set_terrain_map(terrain_map);

  
  int vehicle_state_len = 21;
  
  float x_start[vehicle_state_len];
  float x_end[vehicle_state_len];
  
  for(unsigned i = 0; i < vehicle_state_len; i++){
    x_start[i] = 0;
    x_end[i] = 0;
  }
  
  x_start[0] =   6.275635; //x
  x_start[1] =  -7.405864; //y
  x_start[5] =  -0.766786; //qz
  x_start[10] = -0.641903; //qw
  x_start[11] = -1.498179; //vx
  x_start[12] =  0.450911; //vy
  x_start[16] = -5.527424; //wz
  
  float vl = 1.417517; //  
  float vr = 0.060885;
  float duration = .5;


  //solver.get_tire_sinkages_and_cpts(x_start, sinkages, cpt_X);
  solver.solve(x_start, x_end, vl, vr, duration);
  
  for(unsigned i = 0; i < vehicle_state_len; i++){
    x_start[i] = x_end[i];
  }


  delete terrain_map;
  JackalDynamicSolver::del_model();
}




// Will two trajectories with slightly different
// initial conditions lead to very different trajectories?
// Lets find out
void test_state_propagator_divergence(){
  ROS_INFO("Trajectory Divergence Test");

  ompl::control::Control *contr = si->allocControl();
  
  JackalStatePropagator model(si);
  ompl::base::State *start_state1 = si->allocState();
  ompl::base::State *end_state1 = si->allocState();

  ompl::base::State *start_state2 = si->allocState();
  ompl::base::State *end_state2 = si->allocState();
  
  ompl::control::Control *control = si->allocControl();
  
  sampler->sampleUniform(start_state1);
  si->copyState(start_state2, start_state1);

  double *control_vector;
  double *start_vector1 = static_cast<ompl::base::VehicleStateSpace::StateType *>(start_state1)->values;
  double *start_vector2 = static_cast<ompl::base::VehicleStateSpace::StateType *>(start_state2)->values;

  start_vector1[2] = 0;
  start_vector1[3] = 0;
  start_vector1[4] = 0;
  start_vector1[5] = 0;
  start_vector1[6] = 0;
  start_vector1[7] = 0;
  start_vector1[8] = 0;
  start_vector1[9] = 0;
  
  
  start_vector2[2] = 0;
  start_vector2[3] = 0;
  start_vector2[4] = 0;
  start_vector2[5] = 1e-3;
  start_vector2[6] = 0;
  start_vector2[7] = 0;
  start_vector2[8] = 0;
  start_vector2[9] = 0;
  
  for(int i = 0; i < 10; i++){
    control_sampler->sample(control);
    
    //control_vector[0] = 0;
    //control_vector[1] = 1;
    
    model.propagate(start_state1, control, 1.0, end_state1);
    model.propagate(start_state2, control, 1.0, end_state2);
    
    start_vector1 = static_cast<ompl::base::VehicleStateSpace::StateType *>(start_state1)->values;
    start_vector2 = static_cast<ompl::base::VehicleStateSpace::StateType *>(start_state2)->values;
    control_vector = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    
    ROS_INFO("Control %f %f", control_vector[0], control_vector[1]);
    ROS_INFO("States  %f %f      %f %f", start_vector1[0], start_vector1[1],      start_vector2[0], start_vector2[1]);
    
    float dist = 0;
    for(int j = 0; j < 6; j++){
      float temp = (start_vector1[j] - start_vector2[j]);
      dist += temp*temp;
    }
    ROS_INFO("Distance %f", sqrtf(dist));
    
    si->copyState(start_state1, end_state1);
    si->copyState(start_state2, end_state2);
  }
  ROS_INFO("DONE");
}



//Make sure angle difference is the between [-pi,pi]
void test_angle_math(){
  ROS_INFO("Math Test");
  
  float source, dest, ang_disp;
  
  source = 0;
  dest = .5;
  ang_disp = source - dest;
  if(ang_disp < -M_PI){
    ang_disp += (2*M_PI);
  }
  else if(ang_disp > M_PI){
    ang_disp -= (2*M_PI);
  }
  
  ROS_INFO("%f - %f = %f", dest, source, ang_disp);



  source = 0;
  dest = 5.6;
  ang_disp = source - dest;
  if(ang_disp < -M_PI){
    ang_disp += (2*M_PI);
  }
  else if(ang_disp > M_PI){
    ang_disp -= (2*M_PI);
  }
  
  ROS_INFO("%f - %f = %f", dest, source, ang_disp);



  source = 5.5;
  dest = .5;
  ang_disp = source - dest;
  if(ang_disp < -M_PI){
    ang_disp += (2*M_PI);
  }
  else if(ang_disp > M_PI){
    ang_disp -= (2*M_PI);
  }
  
  ROS_INFO("%f - %f = %f", dest, source, ang_disp);
  
}


void test_vehicle_control_sampler(){
  ROS_INFO("test vehicl control sampler");
  
  float angle = M_PI/4.0f;
  Eigen::Matrix3d rotz = RigidBodyDynamics::Math::rotz(angle);
  Eigen::Vector3d dist = Eigen::Vector3d(1, 1, 2);
  Eigen::Vector3d dist_body_frame = rotz*dist;
  ROS_INFO("<%f %f %f>   |   <%f %f %f>", dist[0], dist[1], dist[2],    dist_body_frame[0], dist_body_frame[1], dist_body_frame[2]);


  angle = M_PI/2.0f;
  rotz = RigidBodyDynamics::Math::rotz(angle);
  dist = Eigen::Vector3d(1, 1, 2);
  dist_body_frame = rotz*dist;
  ROS_INFO("<%f %f %f>   |   <%f %f %f>", dist[0], dist[1], dist[2],    dist_body_frame[0], dist_body_frame[1], dist_body_frame[2]);



  angle = M_PI/2.0f;
  rotz = RigidBodyDynamics::Math::rotz(angle);
  dist = Eigen::Vector3d(1, 2, 2);
  dist_body_frame = rotz*dist;
  ROS_INFO("<%f %f %f>   |   <%f %f %f>", dist[0], dist[1], dist[2],    dist_body_frame[0], dist_body_frame[1], dist_body_frame[2]);

  
}




bool isStateValid(const ompl::base::State *state){
  const ompl::base::VehicleStateSpace::StateType& state_vector = *state->as<ompl::base::VehicleStateSpace::StateType>();
  
  //ROS_INFO("States %f %f %f %f %f %f\n", state_vector[0], state_vector[1], state_vector[2], state_vector[3], state_vector[4], state_vector[5]);
  // Square Obstacle at the origin
  if((fabs(state_vector[0]) < 3 && fabs(state_vector[1]) < 3) ||
     fabs(state_vector[0]) > 10 || fabs(state_vector[1]) > 10){
    return false;
  }
  else{
    return true;
  }
}

ompl::control::DirectedControlSamplerPtr allocCustomControlSampler(const ompl::control::SpaceInformation *si){
  return std::make_shared<ompl::control::SimpleDirectedControlSampler>(si, GlobalParams::get_num_control_samples());
}







void init_ompl(){
  space = new ompl::base::VehicleStateSpace(10);
  ompl::base::RealVectorBounds bounds(10);
  
  bounds.setLow(0, -10);
  bounds.setHigh(0, 10);
  
  bounds.setLow(1, -10);
  bounds.setHigh(1, 10);
  
  bounds.setLow(2, 0);
  bounds.setHigh(2, 2*M_PI);
  
  bounds.setLow(3, -100);
  bounds.setHigh(3, 100);
  
  bounds.setLow(4, -100);
  bounds.setHigh(4, 100);

  bounds.setLow(5, -100);
  bounds.setHigh(5, 100);

  bounds.setLow(6, -100);
  bounds.setHigh(6, 100);

  bounds.setLow(7, -100);
  bounds.setHigh(7, 100);

  bounds.setLow(8, -100);
  bounds.setHigh(8, 100);

  bounds.setLow(9, -100);
  bounds.setHigh(9, 100);
  space->setBounds(bounds);
  
  ompl::base::StateSpacePtr space_ptr = ompl::base::StateSpacePtr(space);
  cspace = new ompl::control::RealVectorControlSpace(space_ptr, 2);
  ompl::base::RealVectorBounds cbounds(2);
  
  cbounds.setLow(0, -GlobalParams::get_max_angular_vel());
  cbounds.setHigh(0, GlobalParams::get_max_angular_vel());

  cbounds.setLow(1, -.1);
  cbounds.setHigh(1, GlobalParams::get_fuzzy_constant_speed());
  
  cspace->setBounds(cbounds);
  
  si = ompl::control::SpaceInformationPtr(new ompl::control::SpaceInformation(space_ptr, ompl::control::ControlSpacePtr(cspace)));
  
  ompl::control::StatePropagatorPtr dynamic_model_ptr(new JackalStatePropagator(si));
  si->setStatePropagator(dynamic_model_ptr);
  si->setPropagationStepSize(GlobalParams::get_propagation_step_size());
  si->setMinMaxControlDuration(1, 10);

  si->setDirectedControlSamplerAllocator(allocCustomControlSampler);
  
  si->setStateValidityChecker(isStateValid);
  si->setStateValidityCheckingResolution(GlobalParams::get_state_checker_resolution());    //this is for checking motions
  si->setup();
  
  sampler = si->allocStateSampler();
  control_sampler = si->allocControlSampler();
  ompl::control::Control *control = si->allocControl();
}

void del_ompl(){
  
}


void test_g_planner(){
  SimpleTerrainMap terrain_map;
  terrain_map.generateObstacles();
  terrain_map.generateUnknownObstacles();
  
  JackalDynamicSolver::set_terrain_map(&terrain_map);
  
  GlobalPlanner g_planner(&terrain_map);
  

  std::vector<RigidBodyDynamics::Math::Vector2d> waypoints;
  float start_state[17] = {50,30,0, 0,0,0,1,  0,0,0,0,0,0,  0,0,0,0};
  RigidBodyDynamics::Math::Vector2d goal_pos(51,30);

  g_planner.plan(waypoints, start_state, goal_pos, .5);
  
  
}


void test_l_planner(){
  SimpleTerrainMap terrain_map;
  terrain_map.generateObstacles();
  terrain_map.generateUnknownObstacles();
  
  DStarPlanner l_planner(&terrain_map);
  
  std::vector<RigidBodyDynamics::Math::Vector2d> waypoints;
  waypoints.push_back(RigidBodyDynamics::Math::Vector2d(-95,-95));
  waypoints.push_back(RigidBodyDynamics::Math::Vector2d(95,95));
  
  l_planner.initWindow();
  l_planner.setGlobalPath(waypoints);
  l_planner.runPlanner();
}

void test_obstacle_detection(){
  SimpleTerrainMap terrain_map;
  Rectangle *rect = new Rectangle();
  
  rect->x = 5;
  rect->y = 0;
  rect->width = 5;
  rect->height = 5;
    
  terrain_map.unknown_obstacles.push_back(rect);

  ROS_INFO("4.9 0 is valid:%d", terrain_map.isStateValid(4.9,0));
  ROS_INFO("5 0 is valid:%d", terrain_map.isStateValid(5,0));
  ROS_INFO("5.1 0 is valid:%d", terrain_map.isStateValid(5.1,0));
  ROS_INFO("5.1 .1 is valid:%d", terrain_map.isStateValid(5.1,.1));
  
  ROS_INFO("0 0 is detected:%d", terrain_map.detectObstacles(0,0));
  terrain_map.unknown_obstacles.push_back(rect);
  ROS_INFO("0 5 is detected:%d", terrain_map.detectObstacles(0,5));

  terrain_map.unknown_obstacles.push_back(rect);
  ROS_INFO("10 0 is detected:%d", terrain_map.detectObstacles(10,0));
  terrain_map.unknown_obstacles.push_back(rect);
  ROS_INFO("0 10 is detected:%d", terrain_map.detectObstacles(0,10));
  terrain_map.unknown_obstacles.push_back(rect);
  ROS_INFO("5.1 .1 is detected:%d", terrain_map.detectObstacles(5.1,.1));
}

int main(int argc, char **argv){
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::String>("Global_Planner", 1); //buffer size is one. Only one global plan needed
  
  GlobalParams::load_params(&nh);
  
  ros::Rate loop_rate(10);
  

  //init_ompl();
  //test_vehicle_space();
  //test_quaternion_math();
  //test_state_propagator_divergence();
  //test_dynamic_model();
  //test_l_planner();
  test_obstacle_detection();
  //test_get_base_velocity();
  //test_angle_math();
  //test_vehicle_control_sampler();
  
  //del_ompl();
  
  ros::spinOnce();
  loop_rate.sleep();

  ros::shutdown();
  return 0;
}
