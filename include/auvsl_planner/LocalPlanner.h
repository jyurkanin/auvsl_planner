#include <Eigen/Core>

class LocalPlanner {
public:
  LocalPlanner(); 
  ~LocalPlanner();

  //list of waypoints from global path planner
  void setGlobalPath(const std::vector<Vector2f> &waypoints);
  
  virtual void initPlanner() = 0;
  virtual void stepPlanner() = 0;
  
protected:
  unsigned curr_waypoint_;
  const std::vector<Vector2f> &waypoints_;
};
