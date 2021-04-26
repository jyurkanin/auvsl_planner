#include <iostream>
#include <thread>
#include <vector>

#include "auvsl_planner_node.h"
#include "JackalStatePropagator.h"
#include "GlobalParams.h"
#include "PlannerVisualizer.h"
#include "VehicleControlSampler.h"
#include "DirectedVehicleControlSampler.h"

#include <ompl/util/RandomNumbers.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

//launch-prefix="gdb -ex run --args

  
/*
 * State of dynamic vehicle is actually:
 * [x,y,z,qx,qy,qz,q1,q2,q3,q4,qw,vx,vy,vz,wx,wy,wz,qd1,qd2,qd3,qd4]
 *
 * Reduced state for planner is going to be:
 * [x, y, theta, vx, vy, wz]
 *
 * Vx,Vy,Wz are important, but x and y are going to be most important
 * for determining distance between states 
 * 
 */

float Xmax = 100;
float Xmin = -100;

float Ymax = 100;
float Ymin = -100;

std::vector<Rectangle*> obstacles;

int isPosInBox(float x, float y, Rectangle *rect){
  return (x > rect->x) && (x < (rect->x + rect->width)) &&
         (y > rect->y) && (y < (rect->y + rect->height));
}

bool isStateValid(const ompl::base::State *state){
  const ompl::base::VehicleStateSpace::StateType& state_vector = *state->as<ompl::base::VehicleStateSpace::StateType>();
  
  for(unsigned i = 0; i < obstacles.size(); i++){
    if(isPosInBox(state_vector[0], state_vector[1], obstacles[i])){
      return false;
    }
  }

  return (state_vector[0] > Xmin) && (state_vector[0] < Xmax) &&
         (state_vector[1] > Ymin) && (state_vector[1] < Ymax);
}




ompl::control::DirectedControlSamplerPtr allocCustomDirectedControlSampler(const ompl::control::SpaceInformation *si){
  return std::make_shared<DirectedVehicleControlSampler>(si, GlobalParams::get_num_control_samples());
}


ompl::control::ControlSamplerPtr allocCustomControlSampler(const ompl::control::ControlSpace *cspace){
  return std::make_shared<VehicleControlSampler>(cspace);
}


/*
*/



void generateObstacles(){
  //Just going to try a couple seed until I get a good obstacle field.
  //Not going to error check, i.e. if the starting point is inside an
  //obstacle or whatever. Ill just check and try a different seed.
  
  ompl::RNG rng;
  const int max_obstacles = 10;

  for(int i = 0; i < max_obstacles; i++){
    Rectangle *rect = new Rectangle();

    rect->width = rng.uniformReal(5, 10);
    rect->height = rng.uniformReal(100, 140);
    
    rect->x = -80 + (160*i/(max_obstacles-1)); //rng.uniformReal(-100, 100);
    rect->y = rng.uniformReal(-50, 50) - rect->height/2;
    
    
    obstacles.push_back(rect);
  }
  
}



void plan(){  
  // construct the state space we are planning in
  ompl::base::VehicleStateSpace space(10);  
  ompl::base::RealVectorBounds bounds(10);
  bounds.setLow(0, -100);
  bounds.setHigh(0, 100);
  
  bounds.setLow(1, -100);
  bounds.setHigh(1, 100);
  
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
  space.setBounds(bounds);
  

  ompl::base::StateSpacePtr space_ptr = ompl::base::StateSpacePtr(&space);
  ompl::control::RealVectorControlSpace cspace(space_ptr, 2);
  
  //cspace.setControlSamplerAllocator(allocCustomControlSampler);
  
  ompl::base::RealVectorBounds cbounds(2);
  
  cbounds.setLow(0, -GlobalParams::get_max_angular_vel());
  cbounds.setHigh(0, GlobalParams::get_max_angular_vel());

  cbounds.setLow(1, 0);
  cbounds.setHigh(1, GlobalParams::get_fuzzy_constant_speed());
  
  cspace.setBounds(cbounds);

  ompl::control::SpaceInformationPtr si(new ompl::control::SpaceInformation(space_ptr, ompl::control::ControlSpacePtr(&cspace)));
  
  ompl::control::StatePropagatorPtr dynamic_model_ptr(new JackalStatePropagator(si));
  si->setStatePropagator(dynamic_model_ptr);
  si->setPropagationStepSize(GlobalParams::get_propagation_step_size());
  si->setMinMaxControlDuration(5, 10);
  
  si->setDirectedControlSamplerAllocator(allocCustomDirectedControlSampler);
  
  si->setStateValidityChecker(isStateValid);
  si->setStateValidityCheckingResolution(GlobalParams::get_state_checker_resolution());    //this is for checking motions
  si->setup();
  
  ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
  ompl::base::ScopedState<> start(space_ptr);
  start[0] = 90;
  start[1] = 30;
  start[2] = 0;
  start[3] = 0;
  start[4] = 0;
  start[5] = 0;
  
  start[6] = 0;
  start[7] = 0;
  start[8] = 0;
  start[9] = 0;
  
  ompl::base::GoalSpace goal(si);
  ompl::base::VehicleStateSpace gspace(10);
  ompl::base::RealVectorBounds gbounds(10);
  gbounds.setLow(0, -90.5);
  gbounds.setHigh(0, -89.5);
  
  gbounds.setLow(1, -60.5);
  gbounds.setHigh(1, -59.5);
  
  gbounds.setLow(2, 0);
  gbounds.setHigh(2, 2*M_PI);
  
  gbounds.setLow(3, -10);
  gbounds.setHigh(3, 10);
  
  gbounds.setLow(4, -10);
  gbounds.setHigh(4, 10);

  gbounds.setLow(5, -10);
  gbounds.setHigh(5, 10);

  gbounds.setLow(6, -100);
  gbounds.setHigh(6, 100);

  gbounds.setLow(7, -100);
  gbounds.setHigh(7, 100);
  
  gbounds.setLow(8, -100);
  gbounds.setHigh(8, 100);

  gbounds.setLow(9, -100);
  gbounds.setHigh(9, 100);

  
  gspace.setBounds(gbounds);
  goal.setSpace(ompl::base::StateSpacePtr(&gspace));
  
  pdef->addStartState(start);
  pdef->setGoal((ompl::base::GoalPtr) &goal);
  
  
  ompl::control::VehicleRRT planner(si);
  planner.setProblemDefinition(pdef);
  planner.setGoalBias(GlobalParams::get_goal_bias()); //.05 was recommended.
  planner.setIntermediateStates(GlobalParams::get_add_intermediate_states());
  
  
  PlannerVisualizer planner_visualizer(si, (ompl::base::PlannerPtr) &planner, .5);
  planner_visualizer.setObstacles(obstacles);
  if(GlobalParams::get_visualize_planner()){
    planner_visualizer.startMonitor();
  }
  
  float max_runtime = 600; //seconds
  ompl::base::PlannerTerminationCondition ptc = ompl::base::plannerOrTerminationCondition(ompl::base::timedPlannerTerminationCondition(max_runtime), ompl::base::exactSolnPlannerTerminationCondition(pdef));
  ompl::base::PlannerStatus solved = planner.solve(ptc);
  
  if(solved){
    ROS_INFO("Solution Found");
    ompl::base::PlannerSolution soln = pdef->getSolutions()[0];
    planner_visualizer.setSolution(&soln);
    
    //pdef->getSolutionPath()->print(std::cout);
  }
  else{
    ROS_INFO("No Solution Found");
  }

  std::cin.get();
}






int main(int argc, char **argv){
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::String>("Global_Planner", 1); //buffer size is one. Only one global plan needed
  
  GlobalParams::load_params(&nh);
  
  ros::Rate loop_rate(10);
  
  //while(ros::ok()){
  ompl::RNG::setSeed(GlobalParams::get_seed());
  generateObstacles();
  plan();
    //pub.publish(msg);
    
  ros::spinOnce();
  loop_rate.sleep();
    //}
  
  return 0;
}





