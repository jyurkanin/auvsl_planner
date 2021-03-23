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
  
#pragma once

#include "ompl/base/ProjectionEvaluator.h"
#include "VehicleStateSpace.h"
  
namespace ompl
{
  namespace base
  {
    class VehicleLinearProjectionEvaluator : public ProjectionEvaluator
    {
    public:
      VehicleLinearProjectionEvaluator(const StateSpace *space, const std::vector<double> &cellSizes,
                                          const ProjectionMatrix::Matrix &projection);
  
      VehicleLinearProjectionEvaluator(const StateSpacePtr &space, const std::vector<double> &cellSizes,
                                          const ProjectionMatrix::Matrix &projection);
  
      VehicleLinearProjectionEvaluator(const StateSpace *space, const ProjectionMatrix::Matrix &projection);
  
      VehicleLinearProjectionEvaluator(const StateSpacePtr &space, const ProjectionMatrix::Matrix &projection);
  
      unsigned int getDimension() const override;
  
      void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override;
  
    protected:
      ProjectionMatrix projection_;
    };
  
    class VehicleRandomLinearProjectionEvaluator : public VehicleLinearProjectionEvaluator
    {
    public:
    VehicleRandomLinearProjectionEvaluator(const StateSpace *space, const std::vector<double> &cellSizes)
      : VehicleLinearProjectionEvaluator(
                                            space, cellSizes, ProjectionMatrix::ComputeRandom(space->getDimension(), cellSizes.size()))
        {
        }
  
    VehicleRandomLinearProjectionEvaluator(const StateSpacePtr &space, const std::vector<double> &cellSizes)
      : VehicleLinearProjectionEvaluator(
                                            space, cellSizes, ProjectionMatrix::ComputeRandom(space->getDimension(), cellSizes.size()))
        {
        }
  
    VehicleRandomLinearProjectionEvaluator(const StateSpace *space, unsigned int dim)
      : VehicleLinearProjectionEvaluator(
                                            space,
                                            ProjectionMatrix::ComputeRandom(space->getDimension(), dim,
                                                                            space->as<VehicleStateSpace>()->getBounds().getDifference()))
        {
        }
  
    VehicleRandomLinearProjectionEvaluator(const StateSpacePtr &space, unsigned int dim)
      : VehicleLinearProjectionEvaluator(
                                            space,
                                            ProjectionMatrix::ComputeRandom(space->getDimension(), dim,
                                                                            space->as<VehicleStateSpace>()->getBounds().getDifference()))
        {
        }
    };
  
    class VehicleOrthogonalProjectionEvaluator : public ProjectionEvaluator
    {
    public:
      VehicleOrthogonalProjectionEvaluator(const StateSpace *space, const std::vector<double> &cellSizes,
                                              std::vector<unsigned int> components);
  
      VehicleOrthogonalProjectionEvaluator(const StateSpacePtr &space, const std::vector<double> &cellSizes,
                                              std::vector<unsigned int> components);
  
      VehicleOrthogonalProjectionEvaluator(const StateSpace *space, std::vector<unsigned int> components);
  
      VehicleOrthogonalProjectionEvaluator(const StateSpacePtr &space, std::vector<unsigned int> components);
  
      unsigned int getDimension() const override;
  
      void defaultCellSizes() override;
  
      void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override;
  
    protected:
      void copyBounds();
  
      std::vector<unsigned int> components_;
    };
  
    class VehicleIdentityProjectionEvaluator : public ProjectionEvaluator
    {
    public:
      VehicleIdentityProjectionEvaluator(const StateSpace *space, const std::vector<double> &cellSizes);
  
      VehicleIdentityProjectionEvaluator(const StateSpacePtr &space, const std::vector<double> &cellSizes);
  
      VehicleIdentityProjectionEvaluator(const StateSpace *space);
  
      VehicleIdentityProjectionEvaluator(const StateSpacePtr &space);
  
      unsigned int getDimension() const override;
  
      void defaultCellSizes() override;
  
      void setup() override;
  
      void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override;
  
    private:
      void copyBounds();
  
      std::size_t copySize_;
    };
  }  // namespace base
}  // namespace ompl
