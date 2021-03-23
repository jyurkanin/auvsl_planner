#include <iostream>
#include <boost/scoped_ptr.hpp>
#include <thread>
#include <utility>



#include <ompl/base/Planner.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

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
  
  PlannerVisualizer(ompl::base::PlannerPtr planner, double period = 0.5)
   : planner_(std::move(planner)),  period_(period), shouldMonitor_(false){

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

  
  
 private:
  void threadFunction();

  Display *dpy;
  Window w;
  GC gc;

  unsigned num_nodes_counter;
  
  ompl::base::PlannerPtr planner_;
  double period_;
  bool shouldMonitor_;
  boost::scoped_ptr<std::thread> monitorThread_;
};


