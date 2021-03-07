#include <ompl/control/StatePropagator.h>
#include <ompl/control/Control.h>
#include <ompl/control/SpaceInformation.h>

#include "JackalDynamicSolver.h"

class JackalStatePropagator : public StatePropagator{
 public:
  JackalStatePropagator(SpaceInformation si);
  void propagate(const base::State *state, const Control *control, double duration, base::State *result);
  bool canPropagateBackwards();
  void steer(const base::State *from, const base::State *to, Control *result, double &duration);
  bool canSteer();
  
  
 private:
  JackalDynamicSolver solver;
};
