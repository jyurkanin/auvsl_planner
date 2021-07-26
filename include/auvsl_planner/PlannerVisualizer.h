#include <iostream>
#include <boost/scoped_ptr.hpp>
#include <thread>
#include <utility>

#include <ompl/base/Planner.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/control/SpaceInformation.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include <rbdl/rbdl.h>

#include "OctoTerrainMap.h"
#include "TerrainMap.h"
#include "VehicleStateSpace.h"

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 640




/*
 * This class header was mostly copy and pasted from PlannerMonitor.h from OMPL
 */

class PlannerVisualizer{
 public:
  // non-copyright
  PlannerVisualizer(const PlannerVisualizer &) = delete;
  PlannerVisualizer &operator=(const PlannerVisualizer &) = delete;

  PlannerVisualizer(ompl::control::SpaceInformationPtr sic, ompl::base::PlannerPtr &planner, const OctoTerrainMap *global_map, double period = 0.5)
    : sic_(sic), planner_(planner),  period_(period), shouldMonitor_(false), global_map_(global_map){
    const ompl::base::RealVectorBounds &bounds = static_cast<const ompl::base::VehicleStateSpace*>(sic_->getStateSpace().get())->getBounds();
    min_state_x_ = bounds.low[0];
    max_state_x_ = bounds.high[0];
    min_state_y_ = bounds.low[1];
    max_state_y_ = bounds.high[1];
  }

  ~PlannerVisualizer(){}

  void drawTree(const ompl::base::PlannerData &planner_data);
  void drawSubTree(const ompl::base::PlannerData &planner_data, unsigned v_idx);
  
  void drawElevation();
  void drawOccupancy();
  
  void setGoal(RigidBodyDynamics::Math::Vector2d);
  void drawGoal();
  
  void startMonitor();
  void stopMonitor();

  void setSolution(std::vector<RigidBodyDynamics::Math::Vector2d> &waypoints, unsigned num_waypoints);
  void drawSolution();

 private:
  void threadFunction();

  const OctoTerrainMap *global_map_;
  
  int has_solution;
  ompl::control::SpaceInformationPtr sic_;
  ompl::base::PlannerSolution *planner_solution;

  unsigned num_waypoints_;
  std::vector<RigidBodyDynamics::Math::Vector2d> waypoints_;

  RigidBodyDynamics::Math::Vector2d goal_;
  
  
  static float min_state_x_;
  static float max_state_x_;
  static float min_state_y_;
  static float max_state_y_;


  std::vector<unsigned> frontier_nodes_;
  std::vector<geometry_msgs::Point> draw_pts_;

  unsigned num_nodes_counter;

  ros::Publisher rrt_visual_pub_;
  
  ompl::base::PlannerPtr planner_;
  double period_;
  bool shouldMonitor_;
  boost::scoped_ptr<std::thread> monitorThread_;
};
