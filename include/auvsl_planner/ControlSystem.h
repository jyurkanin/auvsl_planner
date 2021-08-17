#include <Eigen/Dense>
#include <vector>

#include <geometry_msgs/Pose.h>

/*
 * Abstract base class for control systems + simple waypoint follower
 * For use by Woojin, Justin, and Marius in implementing control systems
 * Used by local planner function computeVelocityCommand
 */


using namespace Eigen;
namespace auvsl{

class ControlSystem{
public:
    ControlSystem();
    ~ControlSystem();

 
  virtual int initialize() = 0;
  virtual int computeVelocityCommand(std::vector<Vector2f> waypoints, geometry_msgs::Pose pose, float &v_forward, float &v_angular) = 0;
};



class SimpleControlSystem : public ControlSystem{
public:
  SimpleControlSystem();
  ~SimpleControlSystem();
  int initialize() override;
  int computeVelocityCommand(std::vector<Vector2f> waypoints, geometry_msgs::Pose pose, float &v_forward, float &v_angular) override;
};
  
}
