#include <iostream>
#include <boost/scoped_ptr.hpp>
#include <thread>
#include <utility>

#include <ompl/base/Planner.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/control/SpaceInformation.h>


#include <X11/keysymdef.h>
#include <X11/Xutil.h>
#include <X11/Xatom.h>
#include <X11/Xlib.h>

#include <rbdl/rbdl.h>

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

  PlannerVisualizer(ompl::control::SpaceInformationPtr sic, ompl::base::PlannerPtr planner, double period = 0.5)
    : sic_(sic), planner_(std::move(planner)),  period_(period), shouldMonitor_(false){
    const ompl::base::RealVectorBounds &bounds = static_cast<const ompl::base::VehicleStateSpace*>(sic_->getStateSpace().get())->getBounds();
    min_state_x_ = bounds.low[0];
    max_state_x_ = bounds.high[0];
    min_state_y_ = bounds.low[1];
    max_state_y_ = bounds.high[1];
  }

  ~PlannerVisualizer(){
      stopMonitor();
    }

  void drawTree(const ompl::base::PlannerData &planner_data);
  void drawSubTree(const ompl::base::PlannerData &planner_data, unsigned vertex);
  static void scaleXY(float state_x, float state_y, float &draw_x, float &draw_y);

  void setObstacles(std::vector<Rectangle*> obstacles);
  void drawObstacles();
  void drawGoal();

  void startMonitor();
  void stopMonitor();

  void setSolution(ompl::base::PlannerSolution *solution);
  void drawSolution();

 private:
  void threadFunction();

  int has_solution;
  ompl::control::SpaceInformationPtr sic_;
  ompl::base::PlannerSolution *planner_solution;

  unsigned num_waypoints_;
  std::vector<RigidBodyDynamics::Math::Vector2d> waypoints_;

  std::vector<Rectangle*> obstacles_;

  static float min_state_x_;
  static float max_state_x_;
  static float min_state_y_;
  static float max_state_y_;


  Display *dpy;
  Window w;
  GC gc;

  unsigned num_nodes_counter;

  ompl::base::PlannerPtr planner_;
  double period_;
  bool shouldMonitor_;
  boost::scoped_ptr<std::thread> monitorThread_;
};
