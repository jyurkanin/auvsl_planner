#include <rbdl/rbdl.h>


class LocalPlanner {
public:
  LocalPlanner(std::vector<Vector2d> waypoints); //list of waypoints from global path planner
  ~LocalPlanner();



private:
  unsigned curr_waypoint_
  std::vector<Vector2d> waypoints_;
};
