#include <ompl/control/StatePropagator.h>
#include <ompl/control/Control.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "JackalDynamicSolver.h"

class JackalStatePropagator : public ompl::control::StatePropagator{
 public:
  JackalStatePropagator(ompl::control::SpaceInformationPtr si);
  virtual void propagate(const ompl::base::State* state, const ompl::control::Control* control, double duration, ompl::base::State *result) const override;
  virtual bool canPropagateBackward() const override;
  virtual bool steer(const ompl::base::State* from, const ompl::base::State* to, ompl::control::Control* result, double &duration) const override;
  virtual bool canSteer() const override;
  
  
 private:
  //JackalDynamicSolver solver;
};
