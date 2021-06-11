#include <rbdl/rbdl.h>
#include <vector>
#include <Eigen/Dense>

using namespace Eigen;

class LocalPlanner {
public:
  LocalPlanner(); 
  ~LocalPlanner();

  //list of waypoints from global path planner
  void setGlobalPath(const std::vector<RigidBodyDynamics::Math::Vector2d> &waypoints);
  
  virtual void initPlanner(Vector2f start, Vector2f goal) = 0;
  virtual void stepPlanner() = 0;
  
protected:
  unsigned curr_waypoint_;
  const std::vector<Vector2f> &waypoints_;
};
