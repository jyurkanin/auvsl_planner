/* Software License Agreement (BSD License)
  *
  *  Copyright (c) 2011, Willow Garage, Inc.
  *  All rights reserved.
  *
  *  Redistribution and use in source and binary forms, with or without
  *  modification, are permitted provided that the following conditions
  *  are met:
  *
  *   * Redistributions of source code must retain the above copyright
  *     notice, this list of conditions and the following disclaimer.
  *   * Redistributions in binary form must reproduce the above
  *     copyright notice, this list of conditions and the following
  *     disclaimer in the documentation and/or other materials provided
  *     with the distribution.
  *   * Neither the name of the Willow Garage nor the names of its
  *     contributors may be used to endorse or promote products derived
  *     from this software without specific prior written permission.
  *
  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
                                                               *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
                                                               *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  *  POSSIBILITY OF SUCH DAMAGE.
  *********************************************************************/
  
  /* Author: Ioan Sucan */

#include "DirectedVehicleControlSampler.h"
#include "GlobalParams.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include <rbdl/rbdl.h>

DirectedVehicleControlSampler::DirectedVehicleControlSampler(const ompl::control::SpaceInformation *si, unsigned int k)
  : DirectedControlSampler(si), cs_(si->allocControlSampler()), numControlSamples_(k){}
  
DirectedVehicleControlSampler::~DirectedVehicleControlSampler() = default;
  
unsigned int DirectedVehicleControlSampler::sampleTo(ompl::control::Control *control, const ompl::base::State *source, ompl::base::State *dest){
  return getBestControl(control, source, dest, nullptr);
}
  
unsigned int DirectedVehicleControlSampler::sampleTo(ompl::control::Control *control, const ompl::control::Control *previous, const ompl::base::State *source, ompl::base::State *dest){
  return getBestControl(control, source, dest, previous);
}

unsigned int DirectedVehicleControlSampler::getBestControl(ompl::control::Control *control, const ompl::base::State *source, ompl::base::State *dest, const ompl::control::Control *previous){
  /* Sample the first control
  if (previous != nullptr)
    cs_->sampleNext(control, previous, source);
  else
    cs_->sample(control, source);
  */
  
  const unsigned int minDuration = si_->getMinControlDuration();
  const unsigned int maxDuration = si_->getMaxControlDuration();
  
  unsigned int steps = cs_->sampleStepCount(minDuration, maxDuration);
  
  sampleControlHeuristic(control, source, dest, previous, steps);
  
  // Propagate the first control, and find how far it is from the target state
  ompl::base::State *bestState = si_->allocState();
  steps = si_->propagateWhileValid(source, control, steps, bestState);
  
  if (numControlSamples_ > 1)
    {
      ompl::control::Control *tempControl = si_->allocControl();
      ompl::base::State *tempState = si_->allocState();
      double bestDistance = si_->distance(bestState, dest);
  
      // Sample k-1 more controls, and save the control that gets closest to target
      for (unsigned int i = 1; i < numControlSamples_; ++i)
        {
          unsigned int sampleSteps = cs_->sampleStepCount(minDuration, maxDuration);
          /*
          if (previous != nullptr)
            cs_->sampleNext(tempControl, previous, source);
          else
            cs_->sample(tempControl, source);
          */
          sampleControlHeuristic(tempControl, source, dest, previous, sampleSteps);
          
          sampleSteps = si_->propagateWhileValid(source, tempControl, sampleSteps, tempState);
          double tempDistance = si_->distance(tempState, dest);
          if (tempDistance < bestDistance)
            {
              si_->copyState(bestState, tempState);
              si_->copyControl(control, tempControl);
              
              bestDistance = tempDistance;
              steps = sampleSteps;
            }
        }
  
      si_->freeState(tempState);
      si_->freeControl(tempControl);
    }
  
  si_->copyState(dest, bestState);
  si_->freeState(bestState);
  
  return steps;
}




void DirectedVehicleControlSampler::sampleControlHeuristic(ompl::control::Control *control, const ompl::base::State *source, ompl::base::State *dest, const ompl::control::Control *previous, unsigned steps){
  double *control_vector = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
  const double* source_vector = source->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  const double* dest_vector = dest->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  
  //ROS_INFO("Source Vector %f %f", source_vector[0], source_vector[1]);
  //ROS_INFO("Dest Vector %f %f", dest_vector[0], dest_vector[1]);
  
  double duration = steps * GlobalParams::get_propagation_step_size();

  //subtraction over a ring is always stupid. Max angle between two vectors is going to be [-pi,pi]
  double ang_disp = source_vector[2] - dest_vector[2];
  if(ang_disp < -M_PI){
    ang_disp += (2*M_PI);
  }
  else if(ang_disp > M_PI){
    ang_disp -= (2*M_PI);
  }
    
  float Wz1 = ang_disp / duration;
  
  Eigen::Matrix3d rotz = RigidBodyDynamics::Math::rotz(source_vector[2]);
  Eigen::Vector3d dist(dest_vector[0] - source_vector[0], dest_vector[1] - source_vector[1], 0);
  Eigen::Vector3d dist_body_frame = rotz*dist;
  
  double Vx = dist[0] / duration; //Velocity forward
  double Vy = dist[1] / duration; //Velocity left
  float Vf = Vx;//sqrtf(Vx*Vx + Vy*Vy); //Vx;
  
  float Wz2 = .25*atan2f(Vy, Vx) / duration;

  float mix = GlobalParams::get_heading_heuristic();
  float Wz_sum = Wz1*(mix) + Wz2*(1-mix);

  if(fabs(Wz_sum) > .5){
    Vf = Vf/2.0;
  }
  if(fabs(Wz_sum) > .7){
    Vf = 1.0;//Vf/2.0;
  }
  
  float Vf_var = GlobalParams::get_forward_variance();
  float Wz_var = GlobalParams::get_angular_variance();
  
  const ompl::base::RealVectorBounds &bounds = static_cast<const ompl::control::RealVectorControlSpace*>(si_->getControlSpace().get())->getBounds();

  //ROS_INFO("Wz unbounded %f", Wz_sum);
  
  control_vector[0] = fmax(bounds.low[0], fmin(bounds.high[0], Wz_sum + rng_.uniformReal(-Wz_var, Wz_var)));
  control_vector[1] = fmax(bounds.low[1], fmin(bounds.high[1], Vf + rng_.uniformReal(-Vf_var, Vf_var)));
  
  //ROS_INFO("rng test %f   %f %f", rng_.uniformReal(-Vf_var, Vf_var), bounds.low[0], bounds.high[0]);
  //ROS_INFO("Control Vector %f %f\n\n", control_vector[0], control_vector[1]);
  
}
