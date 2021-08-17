#include <ompl/util/Time.h>
#include <ompl/control/PathControl.h>
#include <limits>
#include <functional>
#include <ros/ros.h>
#include "PlannerVisualizer.h"
#include "JackalStatePropagator.h"
#include <unistd.h>



float PlannerVisualizer::min_state_x_;
float PlannerVisualizer::max_state_x_;
float PlannerVisualizer::min_state_y_;
float PlannerVisualizer::max_state_y_;


void PlannerVisualizer::startMonitor(){
  int argc = 0;
  ros::init(argc, 0, "points_and_lines");
  ros::NodeHandle nh;
  rrt_visual_pub_ = nh.advertise<visualization_msgs::Marker>("rrt_visual", 10);
  
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
  //while(1){
  //  std::this_thread::sleep_for(ompl::time::seconds(1.0));
  //}
  if (!monitorThread_)
    return;
  shouldMonitor_ = false;
  monitorThread_->join();
  monitorThread_.reset();
  
}


void PlannerVisualizer::setSolution(std::vector<RigidBodyDynamics::Math::Vector2d> &waypoints, unsigned num_waypoints){
  waypoints_ = waypoints;
  num_waypoints_ = num_waypoints;
  has_solution = 1;
}




void PlannerVisualizer::threadFunction(){
  ompl::time::point startTime = ompl::time::now();
  ompl::time::point lastOutputTime = startTime;
  
  ompl::base::PlannerData planner_data(planner_->getSpaceInformation());
  
  
  while (shouldMonitor_){
    double timeSinceOutput = ompl::time::seconds(ompl::time::now() - lastOutputTime);
    if (timeSinceOutput < period_){
      std::this_thread::sleep_for(ompl::time::seconds(0.01));
      continue;
    }

    
    planner_->getPlannerData(planner_data);
    
    drawTree(planner_data);
    drawGoal();
    drawElevation();
    drawOccupancy();  
    
    if(has_solution){
      drawSolution();
    }
    
    lastOutputTime = ompl::time::now();
    std::this_thread::sleep_for(ompl::time::seconds(0.01));
  }
}

void PlannerVisualizer::drawSolution(){
  float curr_x, curr_y, prev_x, prev_y;


}

void PlannerVisualizer::setGoal(RigidBodyDynamics::Math::Vector2d goal){
  goal_ = goal;
}

void PlannerVisualizer::drawElevation(){
  std::vector<geometry_msgs::Point> elev_pts;
  float x;
  float y;
  float alt = 0;
  for(int j = 0; j < global_map_->rows_; j++){
      for(int i = 0; i < global_map_->cols_; i++){
        x = (i*global_map_->map_res_) + global_map_->x_origin_;
        y = (j*global_map_->map_res_) + global_map_->y_origin_;
        
        alt = global_map_->getAltitude(x, y, alt);

        geometry_msgs::Point pt;
        pt.x = x;
        pt.y = y;
        pt.z = alt;
        elev_pts.push_back(pt);
      }
  }

  
  visualization_msgs::Marker point_list;
  point_list.header.frame_id = "map";
  point_list.header.stamp = ros::Time::now();
  point_list.ns = "points_and_lines";
  point_list.action = visualization_msgs::Marker::ADD;
  point_list.pose.orientation.w = 1.0;
  point_list.id = 5;
  point_list.type = visualization_msgs::Marker::POINTS;
  point_list.scale.x = 0.1; //line width
  point_list.scale.y = 0.1; //line width
  point_list.color.r = .5;
  point_list.color.a = 1.0;
  point_list.points = elev_pts;
  
  rrt_visual_pub_.publish(point_list);  
}




void PlannerVisualizer::drawOccupancy(){
  std::vector<geometry_msgs::Point> occ_pts;
  float x;
  float y;
  int occ;
  for(int j = 0; j < global_map_->rows_; j++){
      for(int i = 0; i < global_map_->cols_; i++){
        x = (i*global_map_->map_res_) + global_map_->x_origin_;
        y = (j*global_map_->map_res_) + global_map_->y_origin_;
        
        occ = global_map_->isStateValid(x, y);
        
        if(!occ){
          geometry_msgs::Point pt;
          pt.x = x;
          pt.y = y;
          pt.z = 0;
          occ_pts.push_back(pt);
        }
      }
  }

  
  visualization_msgs::Marker point_list;
  point_list.header.frame_id = "map";
  point_list.header.stamp = ros::Time::now();
  point_list.ns = "points_and_lines";
  point_list.action = visualization_msgs::Marker::ADD;
  point_list.pose.orientation.w = 1.0;
  point_list.id = 6;
  point_list.type = visualization_msgs::Marker::POINTS;
  point_list.scale.x = 0.1; //line width
  point_list.scale.y = 0.1; //line width
  point_list.color.g = 1.0;
  point_list.color.a = 1.0;
  point_list.points = occ_pts;
  
  rrt_visual_pub_.publish(point_list);  
}



void PlannerVisualizer::drawGoal(){
  float goal_x = goal_[0];
  float goal_y = goal_[1];
  float temp_x, temp_y;
  
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "map";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "points_and_lines";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = 3;
  line_list.type = visualization_msgs::Marker::SPHERE;
  line_list.scale.x = 0.1;
  line_list.scale.y = 0.1;
  line_list.scale.z = 0.1;
  
  line_list.color.r = 1.0;
  line_list.color.g = 0.0;
  line_list.color.b = 0.0;
  line_list.color.a = 1.0;
  line_list.pose.position.x = goal_x;
  line_list.pose.position.y = goal_y;
  line_list.pose.position.z = 0;
  
  rrt_visual_pub_.publish(line_list);
}



void PlannerVisualizer::drawTree(const ompl::base::PlannerData &planner_data){
  draw_pts_.clear();
  
  ROS_INFO("Num frontier nodes %lu", frontier_nodes_.size());
  
  drawSubTree(planner_data, planner_data.getStartIndex(0));
  
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "map";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "points_and_lines";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = 2;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.03; //line width
  line_list.color.b = 1.0;
  line_list.color.a = 1.0;
  line_list.points = draw_pts_;
  
  rrt_visual_pub_.publish(line_list);
}

void PlannerVisualizer::drawSubTree(const ompl::base::PlannerData &planner_data, unsigned v_idx){
  const ompl::base::PlannerDataVertex &vertex = planner_data.getVertex(v_idx);
  
  if(vertex == ompl::base::PlannerData::NO_VERTEX){
    return; //this shouldn't even happen.
  }
  
  const ompl::base::State *state = vertex.getState();
  const ompl::base::RealVectorStateSpace::StateType& state_vector = *state->as<ompl::base::RealVectorStateSpace::StateType>();
  
  std::vector<unsigned> edge_list;
  unsigned num_edges = planner_data.getEdges(v_idx, edge_list);

  //ROS_INFO("Current Vertex %u\n", v_idx);
  
  for(unsigned i = 0; i < num_edges; i++){
    unsigned next_v_idx = edge_list[i];
    
    const ompl::base::PlannerDataVertex &next_vertex = planner_data.getVertex(next_v_idx);
    const ompl::base::State *next_state = next_vertex.getState();
    const ompl::base::RealVectorStateSpace::StateType& next_state_vector = *next_state->as<ompl::base::RealVectorStateSpace::StateType>();
    
    geometry_msgs::Point parent_point;
    geometry_msgs::Point child_point;
    
    parent_point.x = state_vector[0];
    parent_point.y = state_vector[1];
    parent_point.z = state_vector[2];
    
    child_point.x = next_state_vector[0];
    child_point.y = next_state_vector[1];
    child_point.z = next_state_vector[2];
    
    draw_pts_.push_back(parent_point);
    draw_pts_.push_back(child_point);
    //ROS_INFO("Current Vertex %u\n", v_idx);
    drawSubTree(planner_data, next_v_idx);
  }

}

