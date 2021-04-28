#include <iostream>
#include <thread>

#include "auvsl_planner_node.h"
#include "JackalStatePropagator.h"
#include "GlobalParams.h"
#include "PlannerVisualizer.h"
#include "VehicleControlSampler.h"
#include "DirectedVehicleControlSampler.h"

#include <ompl/util/RandomNumbers.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

#include <X11/keysymdef.h>
#include <X11/Xutil.h>
#include <X11/Xatom.h>
#include <X11/Xlib.h>



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


std::vector<Rectangle*> obstacles;

float Xmax = 10;
float Xmin = -10;

float Ymax = 10;
float Ymin = -10;

int isPosInBox(float x, float y, Rectangle *rect){
  return (x > rect->x) && (x < (rect->x + rect->width)) &&
         (y > rect->y) && (y < (rect->y + rect->height));
}

bool isStateValid(const ompl::base::State *state){
  const ompl::base::VehicleStateSpace::StateType& state_vector = *state->as<ompl::base::VehicleStateSpace::StateType>();

  //test for roll over
  RigidBodyDynamics::Math::Quaternion quat(state_vector[3], state_vector[4], state_vector[5], state_vector[6]);
  RigidBodyDynamics::Math::Vector3d vec = quat.rotate(RigidBodyDynamics::Math::Vector3d(0,0,1));
  if(vec[2] < 0){ //you could put a number slightly greater than zero here. But I'll leave it as zero for now.
    return false; //if the vehicle has rotated so the z axis of the body frame is pointing down in the world frame, then it fucked up
  }
  
  
  if(isPosInBox(state_vector[0], state_vector[1], obstacles[0])){
    return false;
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






Display *dpy;
Window w;
GC gc;

void init_window(){
  dpy = XOpenDisplay(0);
  w = XCreateSimpleWindow(dpy, DefaultRootWindow(dpy), 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, 0, 0, 0);
  
  XSelectInput(dpy, w, StructureNotifyMask | ExposureMask | KeyPressMask);
  XClearWindow(dpy, w);
  XMapWindow(dpy, w);
  gc = XCreateGC(dpy, w, 0, 0);
  
  XEvent e;
  do{
    XNextEvent(dpy, &e);        
  } while(e.type != MapNotify);
  
  XSetBackground(dpy, gc, 0);
}


void del_window(){
  XDestroyWindow(dpy, w);
  XCloseDisplay(dpy);

}

void drawVertex(float vertex_x, float vertex_y){
  float draw_x, draw_y;
  PlannerVisualizer::scaleXY(vertex_x, vertex_y,  draw_x, draw_y);
  
  XSetForeground(dpy, gc, 0xFF00);
  XFillArc(dpy, w, gc, draw_x, draw_y, 11, 11, 0, 360*64);
}



void test_path(ompl::control::SpaceInformationPtr si, ompl::base::ScopedState<> start, ompl::base::ScopedState<> end){

  XSetForeground(dpy, gc, 0xFF);
  drawVertex(start[0], start[1]);
  drawVertex(end[0], end[1]);
  
  DirectedVehicleControlSampler controlSampler(si.get(), 10);
  JackalStatePropagator statePropagator(si);

  std::vector<ompl::control::Control*> controls;
  std::vector<double> durations;
  std::vector<ompl::base::State*> states;
  std::vector<float> waypoints;
  unsigned num_waypoints;

  ompl::control::Control *control = si->allocControl();


  ompl::base::State* start_state = start.get();//->as<ompl::base::State*>();
  ompl::base::State* end_state = end.get();//->as<ompl::base::State*>();
  
  durations.push_back(10*.1);
  states.push_back(start_state);
  
  
  XSetForeground(dpy, gc, 0xFF0000);
  for(int i = 0; i < 1; i++){    
    controlSampler.sampleControlHeuristic(control, start_state, end_state, nullptr, 10);
    states.push_back(end_state);
    
    controls.clear();
    controls.push_back(control);
    
    statePropagator.getWaypoints(controls, durations, states, waypoints, num_waypoints);

    for(int j = 0; j < num_waypoints-1; j++){
      //ROS_INFO("Waypoint %f %f", waypoints[(j*2)+0], waypoints[(j*2)+1]);
      
      float curr_x, curr_y, prev_x, prev_y;
      
      PlannerVisualizer::scaleXY(waypoints[(j*2)+0], waypoints[(j*2)+1], curr_x, curr_y);
      PlannerVisualizer::scaleXY(waypoints[(j*2)+2], waypoints[(j*2)+3], prev_x, prev_y);
      
      XDrawLine(dpy, w, gc, curr_x, curr_y, prev_x, prev_y);
    }
    XFlush(dpy);
  }

  
  ROS_INFO("Done");
  std::cin.get();

  XClearWindow(dpy, w);
}






void test(){
  Rectangle *rect = new Rectangle();
  rect->width = 3;
  rect->height = 3;
  rect->x = -1.5;
  rect->y = -1.5;
  obstacles.push_back(rect);

  
  
  // construct the state space we are planning in
  ompl::base::VehicleStateSpace space(17);  
  ompl::base::RealVectorBounds bounds(17);
  
  bounds.setLow(0, -10); bounds.setHigh(0, 10); //x  
  bounds.setLow(1, -10); bounds.setHigh(1, 10); //y
  bounds.setLow(2, -10); bounds.setHigh(2, 10); //z
  
  bounds.setLow(3, -1); bounds.setHigh(3, 1); //quaterion has to stay on the 4-ball, so its components max is 1, and min is -1
  bounds.setLow(4, -1); bounds.setHigh(4, 1);
  bounds.setLow(5, -1); bounds.setHigh(5, 1);
  bounds.setLow(6, -1); bounds.setHigh(6, 1);

  bounds.setLow(7, -100); bounds.setHigh(7, 100); //vx
  bounds.setLow(8, -100); bounds.setHigh(8, 100);
  bounds.setLow(9, -100); bounds.setHigh(9, 100);

  bounds.setLow(10, -100); bounds.setHigh(10, 100); //wx
  bounds.setLow(11, -100); bounds.setHigh(11, 100);
  bounds.setLow(12, -100); bounds.setHigh(12, 100);

  bounds.setLow(13, -100); bounds.setHigh(13, 100); //qd1
  bounds.setLow(14, -100); bounds.setHigh(14, 100);
  bounds.setLow(15, -100); bounds.setHigh(15, 100);
  bounds.setLow(16, -100); bounds.setHigh(16, 100);
  
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
  start[0] = -6;
  start[1] = 4;
  start[2] = 0;
  
  start[3] = 0;
  start[4] = 0;
  start[5] = 0;
  start[6] = 1;
  
  start[7] = 0;
  start[8] = 0;
  start[9] = 0;
  
  start[10] = 0;
  start[11] = 0;
  start[12] = 0;
  
  start[13] = 0;
  start[14] = 0;
  start[15] = 0;
  start[16] = 0;
  
  ompl::base::GoalSpace goal(si);
  ompl::base::VehicleStateSpace gspace(17);
  ompl::base::RealVectorBounds gbounds(17);
  gbounds.setLow(0, -6.2);
  gbounds.setHigh(0, -6.0);
  
  gbounds.setLow(1, -0.5);
  gbounds.setHigh(1, 0.5);
  
  gbounds.setLow(2, -1); gbounds.setHigh(2, 1);
  gbounds.setLow(3, -1); gbounds.setHigh(3, 1);
  gbounds.setLow(4, -1); gbounds.setHigh(4, 1);
  gbounds.setLow(5, -1); gbounds.setHigh(5, 1);

  gbounds.setLow(7, -100); gbounds.setHigh(7, 100); //vx
  gbounds.setLow(8, -100); gbounds.setHigh(8, 100);
  gbounds.setLow(9, -100); gbounds.setHigh(9, 100);

  gbounds.setLow(10, -100); gbounds.setHigh(10, 100); //wx
  gbounds.setLow(11, -100); gbounds.setHigh(11, 100);
  gbounds.setLow(12, -100); gbounds.setHigh(12, 100);

  gbounds.setLow(13, -100); gbounds.setHigh(13, 100); //qd1
  gbounds.setLow(14, -100); gbounds.setHigh(14, 100);
  gbounds.setLow(15, -100); gbounds.setHigh(15, 100);
  gbounds.setLow(16, -100); gbounds.setHigh(16, 100);
  
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
  //if(GlobalParams::get_visualize_planner()){
  //  planner_visualizer.startMonitor();
  //}



  ompl::base::ScopedState<> end(space_ptr);
  
  end[0] = 6;
  end[1] = 4;
  end[2] = 0;
  end[3] = 0;
  end[4] = 0;
  end[5] = 0;
  end[6] = 0;
  end[7] = 0;
  end[8] = 0;
  end[9] = 0;
  end[10] = 0;
  end[11] = 0;
  end[12] = 0;
  end[13] = 0;
  end[14] = 0;
  end[15] = 0;
  end[16] = 0;
  
  test_path(si, start, end);

  ROS_INFO("Done");
  std::cin.get();
}





int main(int argc, char **argv){
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::String>("Global_Planner", 1); //buffer size is one. Only one global plan needed
  
  GlobalParams::load_params(&nh);
  
  ros::Rate loop_rate(10);
  
  //while(ros::ok()){
  init_window();
  
  test();
  
  del_window();
    //pub.publish(msg);
    
  ros::spinOnce();
  loop_rate.sleep();
    //}
  
  return 0;
}





