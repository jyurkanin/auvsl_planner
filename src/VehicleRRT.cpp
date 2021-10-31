/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
  
#include "VehicleRRT.h"
#include "GlobalParams.h"
#include "JackalDynamicSolver.h"

#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <limits>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <unistd.h>


ompl::control::VehicleRRT::VehicleRRT(const SpaceInformationPtr &si) : base::Planner(si, "VehicleRRT")
{
  specs_.approximateSolutions = true;
  siC_ = si.get();
  
  Planner::declareParam<double>("goal_bias", this, &VehicleRRT::setGoalBias, &VehicleRRT::getGoalBias, "0.:.05:1.");
  Planner::declareParam<bool>("intermediate_states", this, &VehicleRRT::setIntermediateStates, &VehicleRRT::getIntermediateStates, "0,1");
  
  control_system_ = new auvsl::AnfisControlSystem();
  control_system_->initialize();
}
  
ompl::control::VehicleRRT::~VehicleRRT()
{
  freeMemory();
  delete control_system_;
}
  
void ompl::control::VehicleRRT::setup()
{
  base::Planner::setup();
  if (!nn_)
    nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
  nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}
  
void ompl::control::VehicleRRT::clear()
{
  Planner::clear();
  sampler_.reset();
  controlSampler_.reset();
  freeMemory();
  if (nn_)
    nn_->clear();
  lastGoalMotion_ = nullptr;
}
  
void ompl::control::VehicleRRT::freeMemory()
{
  ROS_INFO("FREE Memory");
  if (nn_)
    {
      std::vector<Motion *> motions;
      nn_->list(motions);
      for (auto &motion : motions)
        {
          if (motion->state)
            si_->freeState(motion->state);
          if (motion->control)
            siC_->freeControl(motion->control);
          delete motion;
        }
    }
}

unsigned ompl::control::VehicleRRT::controlWhileValid(const ompl::base::State *state, ompl::base::State *goal, unsigned steps, std::vector<base::State*> &result){
  double signedStepSize = siC_->getPropagationStepSize();
  
  ROS_INFO("Propagation step size %f", signedStepSize);
  
  const ompl::control::StatePropagatorPtr &statePropagator = siC_->getStatePropagator();
  
  result.resize(steps);
  
  int st = 0;
  float Vf, Wz, dx, dy;
  float err, best_err;
  unsigned best_idx = 0;
  
  std::vector<Eigen::Vector2f> waypoints;
  geometry_msgs::Pose pose;
  
  double *result_values;
  ompl::control::Control *control = siC_->allocControl();
  
  const double *start_values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  const double *goal_values = goal->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  
  if(st < steps){
    result[st] = si_->allocState();
    
    waypoints.push_back(Eigen::Vector2f(start_values[0], start_values[1]));
    waypoints.push_back(Eigen::Vector2f(goal_values[0], goal_values[1]));
    waypoints.push_back(Eigen::Vector2f(goal_values[0], goal_values[1]));
    
    pose.position.x = start_values[0];
    pose.position.y = start_values[1];
    pose.position.z = start_values[2];
    
    pose.orientation.x = start_values[3];
    pose.orientation.y = start_values[4];
    pose.orientation.z = start_values[5];
    pose.orientation.w = start_values[6];
    
    control_system_->computeVelocityCommand(waypoints, pose, Vf, Wz);
    
    double *control_vector = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    control_vector[0] = Wz;
    control_vector[1] = Vf;
    statePropagator->propagate(state, control, signedStepSize, result[st]);
    
    if (si_->isValid(result[st])) {
      result_values = result[st]->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      
      dx = result_values[0] - goal_values[0];
      dy = result_values[1] - goal_values[1];
      
      err = sqrtf((dx*dx) + (dy*dy));
      best_idx = st;
      best_err = err;
      
      ++st;
      while (st < steps) {
        result[st] = si_->allocState();
        
        result_values = result[st - 1]->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        
        pose.position.x = result_values[0];
        pose.position.y = result_values[1];
        pose.position.z = result_values[2];
        
        pose.orientation.x = result_values[3];
        pose.orientation.y = result_values[4];
        pose.orientation.z = result_values[5];
        pose.orientation.w = result_values[6];
        
        waypoints[0][0] = pose.position.x;
        waypoints[0][1] = pose.position.y;
        
        //ROS_INFO("result[st-1] Position %f %f", result_values[0], result_values[1]);
        //ROS_INFO("result[st-1] Orientation %f %f %f   %f", result_values[3], result_values[4], result_values[5], result_values[6]);
        
        
        //ROS_INFO("Way Points[0] %f %f", waypoints[0][0], waypoints[0][1]);
        //ROS_INFO("Way Points[1] %f %f", waypoints[1][0], waypoints[1][1]);
        //ROS_INFO("Way Points[2] %f %f", waypoints[2][0], waypoints[2][1]);
        
        control_system_->computeVelocityCommand(waypoints, pose, Vf, Wz);
        control_vector = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
        control_vector[0] = Wz;
        control_vector[1] = Vf;
        
        statePropagator->propagate(result[st - 1], control, signedStepSize, result[st]);

        if(!si_->isValid(result[st])){
          si_->freeState(result[st]);
          result.resize(st);
          break;
        }
        
        result_values = result[st]->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        
        dx = result_values[0] - goal_values[0];
        dy = result_values[1] - goal_values[1];
        
        err = sqrtf((dx*dx) + (dy*dy));
        if(err < best_err){
          best_idx = st;
          best_err = err;
        }
        
        ++st;
      }
      
      /*
      for(unsigned i = best_idx+1; i < st; i++){
        si_->freeState(result[i]);
      }
      st = best_idx+1;
      result.resize(st);
      */
      result_values = result[st-1]->as<ompl::base::RealVectorStateSpace::StateType>()->values;

      //ROS_INFO("Orientation %f %f %f %f", start_values[3], start_values[4], start_values[5], start_values[6]);
      ROS_INFO("Start: <%f %f>", start_values[0], start_values[1]);
      ROS_INFO("Final State at idx %u: <%f %f>", st, result_values[0], result_values[1]);
      ROS_INFO("Goal: <%f %f>", goal_values[0], goal_values[1]);
      ROS_INFO("Best Error %f\n\n\n", best_err);
      
    }
    else {
      si_->freeState(result[st]);
      result.resize(st);
    }
  }
  
  siC_->freeControl(control);
  
  
  
  return st;
}

ompl::base::PlannerStatus ompl::control::VehicleRRT::solve(const base::PlannerTerminationCondition &ptc) {
  checkValidity();
  base::Goal *goal = pdef_->getGoal().get();
  auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
  
  while (const base::State *st = pis_.nextStart())
    {
      auto *motion = new Motion(siC_);
      si_->copyState(motion->state, st);
      siC_->nullControl(motion->control);
      nn_->add(motion);
    }
  
  if (nn_->size() == 0)
    {
      OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
      return base::PlannerStatus::INVALID_START;
    }
  
  if (!sampler_)
    sampler_ = si_->allocStateSampler();
  if (!controlSampler_)
    controlSampler_ = siC_->allocDirectedControlSampler();
  
  OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());
  
  Motion *solution = nullptr;
  Motion *approxsol = nullptr;
  double approxdif = std::numeric_limits<double>::infinity();
  
  auto *rmotion = new Motion(siC_);
  base::State *rstate = rmotion->state;
  Control *rctrl = rmotion->control;
  base::State *xstate = si_->allocState();
  
  float goal_bias2 = GlobalParams::get_goal_bias2();

  while (ptc == false)
    {
      /* sample random state (with goal biasing) */
      if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
        goal_s->sampleGoal(rstate);
      else{
        if(rng_.uniform01() < goal_bias2){
          goal_s->sampleGoal(rstate);
          Motion *temp_motion = nn_->nearest(rmotion);

          const double* val = temp_motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
          //ROS_INFO("sampleUniformNear <%f %f %f %f %f %f>", val[0], val[1], val[2], val[3], val[4], val[5]);
          sampler_->sampleUniformNear(rstate, temp_motion->state, 10);
        }
        else{
          sampler_->sampleUniform(rstate);
        }
      }
      double *control_vector = rctrl->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
      control_vector[0] = 0;
      control_vector[1] = 0; //Not using these because of the control system.
      
      /* find closest state in the tree */
      Motion *nmotion = nn_->nearest(rmotion);
  
      /* sample a random control that attempts to go towards the random state, and also sample a control duration */
      //=//unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);
      //No need to sample a control, we have a control system
      unsigned cd = ceilf(10.0f / siC_->getPropagationStepSize());
      
      if(addIntermediateStates_){
        std::vector<base::State*> pstates;
        //=//cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);
        cd = controlWhileValid(nmotion->state, rmotion->state, cd, pstates);
        
        if (cd >= siC_->getMinControlDuration()){
          Motion *lastmotion = nmotion;
          bool solved = false;
          size_t p = 0;
          for (; p < pstates.size(); ++p){
            /* create a motion */
            auto *motion = new Motion();
            motion->state = pstates[p];
            // we need multiple copies of rctrl
            motion->control = siC_->allocControl();
            siC_->copyControl(motion->control, rctrl);
            motion->steps = 1;
            motion->parent = lastmotion;
            lastmotion = motion;
            nn_->add(motion);
            double dist = 0.0;
            solved = goal->isSatisfied(motion->state, &dist);
            if (solved) {
              approxdif = dist;
              solution = motion;
              break;
            }
            if (dist < approxdif) {
              approxdif = dist;
              approxsol = motion;
            }
          }
  
          // free any states after we hit the goal
          while (++p < pstates.size()){
            si_->freeState(pstates[p]);
          }
          if (solved)
            break;
        }
        else{
          //ROS_INFO("Trajectory too small");
          for (auto &pstate : pstates){
            si_->freeState(pstate);
          }
        }
      }
      else {
        if (cd >= siC_->getMinControlDuration()) {
          /* create a motion */
          auto *motion = new Motion(siC_);
          si_->copyState(motion->state, rmotion->state);
          siC_->copyControl(motion->control, rctrl);
          motion->steps = cd;
          motion->parent = nmotion;
  
          nn_->add(motion);
          double dist = 0.0;
          bool solv = goal->isSatisfied(motion->state, &dist);
          if (solv) {
            approxdif = dist;
            solution = motion;
            break;
          }
          if (dist < approxdif) {
            approxdif = dist;
            approxsol = motion;
          }
        }
      }
    }
  
  bool solved = false;
  bool approximate = false;
  if (solution == nullptr)
    {
      solution = approxsol;
      approximate = true;
    }
  
  if (solution != nullptr)
    {
      lastGoalMotion_ = solution;
  
      /* construct the solution path */
      std::vector<Motion *> mpath;
      while (solution != nullptr)
        {
          mpath.push_back(solution);
          solution = solution->parent;
        }
  
      /* set the solution path */
      auto path(std::make_shared<PathControl>(si_));
      for (int i = mpath.size() - 1; i >= 0; --i)
        if (mpath[i]->parent)
          path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
        else
          path->append(mpath[i]->state);
      solved = true;
      pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

  if (rmotion->state)
    si_->freeState(rmotion->state);
  if (rmotion->control)
    siC_->freeControl(rmotion->control);
  delete rmotion;
  si_->freeState(xstate);
  
  OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
  
  return {solved, approximate};
}
  
void ompl::control::VehicleRRT::getPlannerData(base::PlannerData &data) const
{
  Planner::getPlannerData(data);
  
  std::vector<Motion *> motions;
  if (nn_)
    nn_->list(motions);
  
  double delta = siC_->getPropagationStepSize();
  
  if (lastGoalMotion_)
    data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));
  
  for (auto m : motions)
    {
      if (m->parent)
        {
          if (data.hasControls())
            data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                         control::PlannerDataEdgeControl(m->control, m->steps * delta));
          else
            data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
        }
      else
        data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}
