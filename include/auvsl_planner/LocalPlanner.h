#include <rbdl/rbdl.h>


class LocalPlanner {
public:
  LocalPlanner(); //list of waypoints from global path planner
  ~LocalPlanner();
  
  void setGlobalPath(const std::vector<Vector2d> &waypoints);
  
  virtual void initPlanner() = 0;
  virtual void stepPlanner() = 0;
  
protected:
  unsigned curr_waypoint_;
  const std::vector<Vector2d> &waypoints_;
};
