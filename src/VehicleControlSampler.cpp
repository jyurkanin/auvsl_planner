/* Author: Justin Yurkanin */

#include "VehicleControlSampler.h"  
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/util/Exception.h"
#include <cstring>
#include <limits>

#include <ros/ros.h>
  
void VehicleControlSampler::sample(ompl::control::Control *control){
  ROS_INFO("VehicleControlSampler::sample");
  
  const unsigned int dim = space_->getDimension();
  const ompl::base::RealVectorBounds &bounds = static_cast<const ompl::control::RealVectorControlSpace *>(space_)->getBounds();
  
  auto *rcontrol = static_cast<ompl::control::RealVectorControlSpace::ControlType *>(control);
  for (unsigned int i = 0; i < dim; ++i)
    rcontrol->values[i] = rng_.uniformReal(bounds.low[i], bounds.high[i]);
}




