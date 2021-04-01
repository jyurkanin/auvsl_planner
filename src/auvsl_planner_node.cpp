#include <iostream>
#include <thread>

#include "auvsl_planner_node.h"
#include "JackalStatePropagator.h"
#include "GlobalParams.h"
#include "PlannerVisualizer.h"
//#include <ompl/base/spaces/RealVectorStateSpace.h>
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

/*
*/



void plan(){
  // construct the state space we are planning in
  ompl::base::VehicleStateSpace space(6);
  
  // set the bounds
  ompl::base::RealVectorBounds bounds(6);
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
  space.setBounds(bounds);
  

  ompl::base::StateSpacePtr space_ptr = ompl::base::StateSpacePtr(&space);
  ompl::control::RealVectorControlSpace cspace(space_ptr, 2);
  ompl::base::RealVectorBounds cbounds(2);
  
  cbounds.setLow(0, -GlobalParams::get_max_angular_vel());
  cbounds.setHigh(0, GlobalParams::get_max_angular_vel());

  cbounds.setLow(1, -.1);
  cbounds.setHigh(1, GlobalParams::get_fuzzy_constant_speed());
  
  cspace.setBounds(cbounds);

  ompl::control::SpaceInformationPtr si(new ompl::control::SpaceInformation(space_ptr, ompl::control::ControlSpacePtr(&cspace)));
  
  ompl::control::StatePropagatorPtr dynamic_model_ptr(new JackalStatePropagator(si));
  si->setStatePropagator(dynamic_model_ptr);
  si->setPropagationStepSize(GlobalParams::get_propagation_step_size());
  si->setMinMaxControlDuration(1, 10);

  si->setDirectedControlSamplerAllocator(allocCustomControlSampler);
  
  si->setStateValidityChecker(isStateValid);
  si->setStateValidityCheckingResolution(GlobalParams::get_state_checker_resolution());    //this is for checking motions
  si->setup();
  
  ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
  ompl::base::ScopedState<> start(space_ptr);
  start[0] = 8;
  start[1] = 0;
  start[2] = 0;
  start[3] = 0;
  start[4] = 0;
  start[5] = 0;
  
  ompl::base::GoalSpace goal(si);
  ompl::base::VehicleStateSpace gspace(6);
  ompl::base::RealVectorBounds gbounds(6);
  gbounds.setLow(0, -5.1);
  gbounds.setHigh(0, -4.9);
  
  gbounds.setLow(1, -8.1);
  gbounds.setHigh(1, -7.9);
  
  gbounds.setLow(2, 0);
  gbounds.setHigh(2, 2*M_PI);
  
  gbounds.setLow(3, -10);
  gbounds.setHigh(3, 10);
  
  gbounds.setLow(4, -10);
  gbounds.setHigh(4, 10);

  gbounds.setLow(5, -10);
  gbounds.setHigh(5, 10);
  
  gspace.setBounds(gbounds);
  goal.setSpace(ompl::base::StateSpacePtr(&gspace));
  
  pdef->addStartState(start);
  pdef->setGoal((ompl::base::GoalPtr) &goal);
  
  
  ompl::control::RRT planner(si);
  planner.setProblemDefinition(pdef);
  planner.setGoalBias(GlobalParams::get_goal_bias()); //.05 was recommended.
  planner.setIntermediateStates(GlobalParams::get_add_intermediate_states());

  PlannerVisualizer planner_visualizer(si, (ompl::base::PlannerPtr) &planner, .5);
  if(GlobalParams::get_visualize_planner()){
    planner_visualizer.startMonitor();
  }
  
  
  ompl::base::PlannerTerminationCondition ptc = ompl::base::plannerOrTerminationCondition(ompl::base::timedPlannerTerminationCondition(30.0), ompl::base::exactSolnPlannerTerminationCondition(pdef));
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

  plan();
    //pub.publish(msg);
    
  ros::spinOnce();
  loop_rate.sleep();
    //}
  
  return 0;
}





