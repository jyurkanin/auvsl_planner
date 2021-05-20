
class LocalPlanner {
public:
  LocalPlanner(std::vector<float> waypoints); //list of waypoints from global path planner
  ~LocalPlanner();

  

private:
  unsigned curr_waypoint_
  std::vector<float> waypoints_;
};
