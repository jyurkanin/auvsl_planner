


JackalStatePropagator::JackalStatePropagator(SpaceInformation si) : StatePropagator(si){
  solver = JackalDynamicSolver(.01, 0);
}

void JackalStatePropagator::propagate(const base::State *state, const Control *control, double duration, base::State *result) const override {
  solver.solve(); #
  
}

void JackalStatePropagator::steer(const base::State *from, const base::State *to, Control *result, double &duration){
  
  
}




bool JackalStatePropagator::canPropagateBackwards(){
  return false;
}

bool JackalStatePropagator::canSteer(){
  return true;
}




