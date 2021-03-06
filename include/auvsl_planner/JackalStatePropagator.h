#include <ompl/control/StatePropagator.h>
#include <ompl/control/Control.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <rbdl/rbdl.h>

#include "JackalDynamicSolver.h"

class JackalStatePropagator : public ompl::control::StatePropagator{
 public:
  JackalStatePropagator(ompl::control::SpaceInformationPtr si);
  ~JackalStatePropagator();

  virtual void propagate(const ompl::base::State* state, const ompl::control::Control* control, double duration, ompl::base::State *result) const override;
  void getWaypoints(std::vector<ompl::control::Control*> &controls, std::vector<double> &durations, std::vector<ompl::base::State*> states, std::vector<Vector2d> &waypoints, unsigned &num_waypoints);
  virtual bool canPropagateBackward() const override;
  virtual bool steer(const ompl::base::State* from, const ompl::base::State* to, ompl::control::Control* result, double &duration) const override;
  virtual bool canSteer() const override;


  static void convert_to_model_space(const double *planner_state, float *model_state);
  static void convert_to_planner_space(double *planner_state, const float *model_state);
  static Vector3d get_base_velocity(float *Xout);

 private:
  //JackalDynamicSolver solver;
};
