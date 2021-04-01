#include <ompl/util/Time.h>
#include <ompl/control/PathControl.h>
#include <limits>
#include <functional>
#include <ros/ros.h>
#include "PlannerVisualizer.h"
#include "JackalStatePropagator.h"


void PlannerVisualizer::startMonitor(){
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
  
  has_solution = 0;
  
  if (monitorThread_)
    return;
  shouldMonitor_ = true;
  monitorThread_.reset(new std::thread([this]
                                       {
                                         threadFunction();
                                       }));
}
  
void PlannerVisualizer::stopMonitor(){
  XDestroyWindow(dpy, w);
  XCloseDisplay(dpy);
  
  
  if (!monitorThread_)
    return;
  shouldMonitor_ = false;
  monitorThread_->join();
  monitorThread_.reset();
}



void PlannerVisualizer::setSolution(ompl::base::PlannerSolution *solution){
  planner_solution = solution;
  
  ompl::control::PathControl *path = planner_solution->path_->as<ompl::control::PathControl>();
  
  std::vector<ompl::base::State*> states = path->getStates();
  std::vector<ompl::control::Control*> controls = path->getControls();
  std::vector<double> durations = path->getControlDurations();
  
  JackalStatePropagator dynamic_model(sic_);

  num_waypoints_ = controls.size() + 1;//1 + floorf(path->length() / GlobalParams::get_propagation_step_size());
  waypoints_ = new float[num_waypoints_*2];
  dynamic_model.getWaypoints(controls, durations, states, waypoints_);

  has_solution = 1;
}



  
void PlannerVisualizer::threadFunction(){
  ompl::time::point startTime = ompl::time::now();
  ompl::time::point lastOutputTime = startTime;
  
  while (shouldMonitor_){
    double timeSinceOutput = ompl::time::seconds(ompl::time::now() - lastOutputTime);
    if (timeSinceOutput < period_){
      std::this_thread::sleep_for(ompl::time::seconds(0.01));
      continue;
    }
    
    ompl::base::PlannerData planner_data(planner_->getSpaceInformation());
    planner_->getPlannerData(planner_data);
    

    drawTree(planner_data);
    drawObstacles();
    drawGoal();
    
    if(has_solution){
      drawSolution();
    }
    
    XFlush(dpy);
    
    lastOutputTime = ompl::time::now();
    std::this_thread::sleep_for(ompl::time::seconds(0.01));
  }
}

void PlannerVisualizer::drawSolution(){
  float curr_x, curr_y, prev_x, prev_y;
  
  XSetForeground(dpy, gc, 0xFFFFFF);
  for(int i = 0; i < num_waypoints_ - 1; i++){
    scaleXY(waypoints_[(i*2)+0], waypoints_[(i*2)+1], curr_x, curr_y);
    scaleXY(waypoints_[(i*2)+2], waypoints_[(i*2)+3], prev_x, prev_y);
    
    XDrawLine(dpy, w, gc, curr_x, curr_y, prev_x, prev_y);
  }
}

void PlannerVisualizer::drawGoal(){
  float goal_x = -5;
  float goal_y = -8;
  float temp_x, temp_y;
  scaleXY(goal_x, goal_y, temp_x, temp_y);
  
  XSetForeground(dpy, gc, 0xFF0000);
  XFillArc(dpy, w, gc, temp_x, temp_y, 11, 11, 0, 360*64);
}

void PlannerVisualizer::drawObstacles(){
  float obs_x = -3;
  float obs_y = -3;
  float top_right_x = 3;
  float top_right_y = 3;
  float temp_x, temp_y, tr_x, tr_y;
  
  scaleXY(obs_x, obs_y, temp_x, temp_y);
  scaleXY(top_right_x, top_right_y, tr_x, tr_y);
  
  float temp_width = tr_x - temp_x;
  float temp_height = -(tr_y - temp_y);

  //ROS_INFO("width %f height %f blx %f bly %f\n", temp_width, temp_height, temp_x, temp_y);
  
  XSetForeground(dpy, gc, 0xFF0000);
  XFillRectangle(dpy, w, gc, temp_x, temp_y - temp_height, temp_width, temp_height);
}

void PlannerVisualizer::drawTree(const ompl::base::PlannerData &planner_data){
  drawSubTree(planner_data, planner_data.vertexIndex(planner_data.getStartVertex(0)));
}

void PlannerVisualizer::drawSubTree(const ompl::base::PlannerData &planner_data, unsigned v_idx){
  const ompl::base::PlannerDataVertex &vertex = planner_data.getVertex(v_idx);

  if(vertex == ompl::base::PlannerData::NO_VERTEX){
    return; //this shouldn't even happen.
  }

  const ompl::base::State *state = vertex.getState();
  const ompl::base::RealVectorStateSpace::StateType& state_vector = *state->as<ompl::base::RealVectorStateSpace::StateType>();
  
  float vertex_x, vertex_y;
  scaleXY(state_vector[0], state_vector[1], vertex_x, vertex_y);
  
  std::vector<unsigned> edge_list;
  unsigned num_edges = planner_data.getEdges(v_idx, edge_list);

  //ROS_INFO("Current Vertex %u\n", v_idx);
  
  for(unsigned i = 0; i < num_edges; i++){
    unsigned next_v_idx = edge_list[i];
    const ompl::base::PlannerDataVertex &next_vertex = planner_data.getVertex(next_v_idx);
    const ompl::base::State *next_state = next_vertex.getState();
    const ompl::base::RealVectorStateSpace::StateType& next_state_vector = *next_state->as<ompl::base::RealVectorStateSpace::StateType>();

    float next_vertex_x, next_vertex_y;
    scaleXY(next_state_vector[0], next_state_vector[1], next_vertex_x, next_vertex_y);
    
    XSetForeground(dpy, gc, 0xFF00);
    XDrawLine(dpy, w, gc, vertex_x, vertex_y, next_vertex_x, next_vertex_y);

    //ROS_INFO("Current Vertex %u\n", v_idx);
    drawSubTree(planner_data, next_v_idx);
  }
  
  if(planner_data.isGoalVertex(v_idx)){
    XSetForeground(dpy, gc, 0xFF0000);
    XFillArc(dpy, w, gc, vertex_x, vertex_y, 11, 11, 0, 360*64);
  }
  else if(planner_data.isStartVertex(v_idx)){
    XSetForeground(dpy, gc, 0xFF00FF);
    XFillArc(dpy, w, gc, vertex_x, vertex_y, 11, 11, 0, 360*64);
  }
  else{
    XSetForeground(dpy, gc, 0xFF);
    XFillArc(dpy, w, gc, vertex_x, vertex_y, 3, 3, 0, 360*64);
  }
  
  
  
}

void PlannerVisualizer::scaleXY(float state_x, float state_y, float &draw_x, float &draw_y){
  float min_state_x = -10;
  float min_state_y = -10;
  float max_state_x = 10;
  float max_state_y = 10; //hardcoding go brrrr
  
  draw_x = WINDOW_WIDTH*(state_x - min_state_x)/(max_state_x - min_state_x);
  draw_y = WINDOW_HEIGHT*(1 - ((state_y - min_state_y)/(max_state_y - min_state_y)));
}
