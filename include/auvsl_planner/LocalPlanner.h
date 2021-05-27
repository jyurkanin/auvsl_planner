#include <rbdl/rbdl.h>


class LocalPlanner {
public:
  LocalPlanner(); 
  ~LocalPlanner();
  
  void setGlobalPath(const std::vector<Vector2d> &waypoints); //list of waypoints from global path planner
  
  virtual void initPlanner() = 0;
  virtual void stepPlanner() = 0;
  
protected:
  unsigned curr_waypoint_;
  const std::vector<Vector2d> &waypoints_;
};
