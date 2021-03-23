/*********************************************************************
  * Software License Agreement (BSD License)
  *
  *  Copyright (c) 2010, Rice University
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
  *   * Neither the name of the Rice University nor the names of its
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
  
#include "VehicleStateProjections.h"
#include "ompl/util/Exception.h"
#include "ompl/tools/config/MagicConstants.h"
#include <cstring>
#include <utility>
  
namespace ompl
{
  namespace base
  {
    static inline void checkSpaceType(const StateSpace *m)
    {
      if (dynamic_cast<const VehicleStateSpace *>(m) == nullptr)
        throw Exception("Expected real vector state space for projection");
    }
  }  // namespace base
}  // namespace ompl
  
ompl::base::VehicleLinearProjectionEvaluator::VehicleLinearProjectionEvaluator(
                                                                                     const StateSpace *space, const std::vector<double> &cellSizes, const ProjectionMatrix::Matrix &projection)
  : ProjectionEvaluator(space)
{
  checkSpaceType(space_);
  projection_.mat = projection;
  setCellSizes(cellSizes);
}
  
ompl::base::VehicleLinearProjectionEvaluator::VehicleLinearProjectionEvaluator(
                                                                                     const StateSpacePtr &space, const std::vector<double> &cellSizes, const ProjectionMatrix::Matrix &projection)
  : ProjectionEvaluator(space)
{
  checkSpaceType(space_);
  projection_.mat = projection;
  setCellSizes(cellSizes);
}
  
ompl::base::VehicleLinearProjectionEvaluator::VehicleLinearProjectionEvaluator(
                                                                                     const StateSpace *space, const ProjectionMatrix::Matrix &projection)
  : ProjectionEvaluator(space)
{
  checkSpaceType(space_);
  projection_.mat = projection;
}
  
ompl::base::VehicleLinearProjectionEvaluator::VehicleLinearProjectionEvaluator(
                                                                                     const StateSpacePtr &space, const ProjectionMatrix::Matrix &projection)
  : ProjectionEvaluator(space)
{
  checkSpaceType(space_);
  projection_.mat = projection;
}
  
ompl::base::VehicleOrthogonalProjectionEvaluator::VehicleOrthogonalProjectionEvaluator(
                                                                                             const StateSpace *space, const std::vector<double> &cellSizes, std::vector<unsigned int> components)
  : ProjectionEvaluator(space), components_(std::move(components))
{
  checkSpaceType(space_);
  setCellSizes(cellSizes);
  copyBounds();
}
  
ompl::base::VehicleOrthogonalProjectionEvaluator::VehicleOrthogonalProjectionEvaluator(
                                                                                             const StateSpacePtr &space, const std::vector<double> &cellSizes, std::vector<unsigned int> components)
  : ProjectionEvaluator(space), components_(std::move(components))
{
  checkSpaceType(space_);
  setCellSizes(cellSizes);
  copyBounds();
}
  
ompl::base::VehicleOrthogonalProjectionEvaluator::VehicleOrthogonalProjectionEvaluator(
                                                                                             const StateSpace *space, std::vector<unsigned int> components)
  : ProjectionEvaluator(space), components_(std::move(components))
{
  checkSpaceType(space_);
}
  
ompl::base::VehicleOrthogonalProjectionEvaluator::VehicleOrthogonalProjectionEvaluator(
                                                                                             const StateSpacePtr &space, std::vector<unsigned int> components)
  : ProjectionEvaluator(space), components_(std::move(components))
{
  checkSpaceType(space_);
}
  
void ompl::base::VehicleOrthogonalProjectionEvaluator::copyBounds()
{
  bounds_.resize(components_.size());
  const RealVectorBounds &bounds = space_->as<VehicleStateSpace>()->getBounds();
  for (unsigned int i = 0; i < components_.size(); ++i)
    {
      bounds_.low[i] = bounds.low[components_[i]];
      bounds_.high[i] = bounds.high[components_[i]];
    }
}
  
void ompl::base::VehicleOrthogonalProjectionEvaluator::defaultCellSizes()
{
  const RealVectorBounds &bounds = space_->as<VehicleStateSpace>()->getBounds();
  bounds_.resize(components_.size());
  cellSizes_.resize(components_.size());
  for (unsigned int i = 0; i < cellSizes_.size(); ++i)
    {
      bounds_.low[i] = bounds.low[components_[i]];
      bounds_.high[i] = bounds.high[components_[i]];
      cellSizes_[i] = (bounds_.high[i] - bounds_.low[i]) / magic::PROJECTION_DIMENSION_SPLITS;
    }
}
  
unsigned int ompl::base::VehicleLinearProjectionEvaluator::getDimension() const
{
  return projection_.mat.rows();
}
  
void ompl::base::VehicleLinearProjectionEvaluator::project(const State *state,
                                                              Eigen::Ref<Eigen::VectorXd> projection) const
{
  projection_.project(state->as<VehicleStateSpace::StateType>()->values, projection);
}
  
unsigned int ompl::base::VehicleOrthogonalProjectionEvaluator::getDimension() const
{
  return components_.size();
}
  
void ompl::base::VehicleOrthogonalProjectionEvaluator::project(const State *state,
                                                                  Eigen::Ref<Eigen::VectorXd> projection) const
{
  for (unsigned int i = 0; i < components_.size(); ++i)
    projection(i) = state->as<VehicleStateSpace::StateType>()->values[components_[i]];
}
  
ompl::base::VehicleIdentityProjectionEvaluator::VehicleIdentityProjectionEvaluator(
                                                                                         const StateSpace *space, const std::vector<double> &cellSizes)
  : ProjectionEvaluator(space)
{
  checkSpaceType(space_);
  setCellSizes(cellSizes);
  copyBounds();
}
  
ompl::base::VehicleIdentityProjectionEvaluator::VehicleIdentityProjectionEvaluator(const StateSpace *space)
  : ProjectionEvaluator(space)
{
  checkSpaceType(space_);
}
  
ompl::base::VehicleIdentityProjectionEvaluator::VehicleIdentityProjectionEvaluator(
                                                                                         const StateSpacePtr &space, const std::vector<double> &cellSizes)
  : ProjectionEvaluator(space)
{
  checkSpaceType(space_);
  setCellSizes(cellSizes);
  copyBounds();
}
  
ompl::base::VehicleIdentityProjectionEvaluator::VehicleIdentityProjectionEvaluator(const StateSpacePtr &space)
  : ProjectionEvaluator(space)
{
  checkSpaceType(space_);
}
  
void ompl::base::VehicleIdentityProjectionEvaluator::copyBounds()
{
  bounds_ = space_->as<VehicleStateSpace>()->getBounds();
}
  
void ompl::base::VehicleIdentityProjectionEvaluator::defaultCellSizes()
{
  bounds_ = space_->as<VehicleStateSpace>()->getBounds();
  cellSizes_.resize(getDimension());
  for (unsigned int i = 0; i < cellSizes_.size(); ++i)
    cellSizes_[i] = (bounds_.high[i] - bounds_.low[i]) / magic::PROJECTION_DIMENSION_SPLITS;
}
  
void ompl::base::VehicleIdentityProjectionEvaluator::setup()
{
  copySize_ = getDimension() * sizeof(double);
  ProjectionEvaluator::setup();
}
  
unsigned int ompl::base::VehicleIdentityProjectionEvaluator::getDimension() const
{
  return space_->getDimension();
}
  
void ompl::base::VehicleIdentityProjectionEvaluator::project(const State *state,
                                                                Eigen::Ref<Eigen::VectorXd> projection) const
{
  projection = Eigen::Map<const Eigen::VectorXd>(state->as<VehicleStateSpace::StateType>()->values, copySize_);
}
