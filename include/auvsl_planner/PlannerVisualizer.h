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

 }
  
  ~PlannerVisualizer(){
      stopMonitor();
    }

  void drawTree(const ompl::base::PlannerData &planner_data);
  void drawSubTree(const ompl::base::PlannerData &planner_data, unsigned vertex);
  void scaleXY(float state_x, float state_y, float &draw_x, float &draw_y);
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
  float *waypoints_;
  
  Display *dpy;
  Window w;
  GC gc;

  unsigned num_nodes_counter;
  
  ompl::base::PlannerPtr planner_;
  double period_;
  bool shouldMonitor_;
  boost::scoped_ptr<std::thread> monitorThread_;
};


