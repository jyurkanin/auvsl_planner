#include <ompl/control/StatePropagator.h>
#include <ompl/control/Control.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "JackalDynamicSolver.h"

class JackalStatePropagator : public ompl::control::StatePropagator{
 public:
  JackalStatePropagator(ompl::control::SpaceInformationPtr si);
  ~JackalStatePropagator();
  
  virtual void propagate(const ompl::base::State* state, const ompl::control::Control* control, double duration, ompl::base::State *result) const override;
  void getWaypoints(std::vector<ompl::control::Control*> &controls, std::vector<double> &durations, std::vector<ompl::base::State*> states, std::vector<float> &waypoints, unsigned &num_waypoints);
  virtual bool canPropagateBackward() const override;
  virtual bool steer(const ompl::base::State* from, const ompl::base::State* to, ompl::control::Control* result, double &duration) const override;
  virtual bool canSteer() const override;

  static void convert_to_model_space(const double *vehicle_space, float *model_space);
  static void convert_to_vehicle_space(double *vehicle_space, const float *model_space);
  static Vector3d get_base_velocity(float *Xout);
  
 private:
  //JackalDynamicSolver solver;
};
