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
// Modified by Justin Yurkanin

#pragma once

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include <vector>
#include <string>
#include <map>

namespace ompl{
  namespace base{
    class VehicleStateSampler : public StateSampler{
    public:
    VehicleStateSampler(const StateSpace *space) : StateSampler(space){}
      void sampleUniform(State *state) override;
      void sampleUniformNear(State *state, const State *near, double distance) override;
      void sampleGaussian(State *state, const State *mean, double stdDev) override;
    };
  
    class VehicleStateSpace : public StateSpace {
    public:
      class StateType : public State {
    public:
      StateType() = default;
  
      double operator[](unsigned int i) const
      {
        return values[i];
      }
  
      double &operator[](unsigned int i)
      {
        return values[i];
      }
  
      double *values;
    };
  
    VehicleStateSpace(unsigned int dim = 0)
      : dimension_(dim), bounds_(dim), stateBytes_(dim * sizeof(double))
      {
        type_ = STATE_SPACE_REAL_VECTOR;
        setName("Vehicle" + getName());
        dimensionNames_.resize(dim, "");
      }
  
    ~VehicleStateSpace() override = default;
  
    void addDimension(double minBound = 0.0, double maxBound = 0.0);
    void addDimension(const std::string &name, double minBound = 0.0, double maxBound = 0.0);
    void setBounds(const RealVectorBounds &bounds);
    void setBounds(double low, double high);
  
    const RealVectorBounds &getBounds() const
    {
      return bounds_;
    }

    bool isMetricSpace() const override;
      
    unsigned int getDimension() const override;
    const std::string &getDimensionName(unsigned int index) const;
  
    int getDimensionIndex(const std::string &name) const;
    void setDimensionName(unsigned int index, const std::string &name);
    double getMaximumExtent() const override;
    double getMeasure() const override;
    void enforceBounds(State *state) const override;
    bool satisfiesBounds(const State *state) const override;
    void copyState(State *destination, const State *source) const override;
    unsigned int getSerializationLength() const override;
    void serialize(void *serialization, const State *state) const override;
    void deserialize(State *state, const void *serialization) const override;
    double distance(const State *state1, const State *state2) const override;
    bool equalStates(const State *state1, const State *state2) const override;
    void interpolate(const State *from, const State *to, double t, State *state) const override;
    StateSamplerPtr allocDefaultStateSampler() const override;
    State *allocState() const override;
    void freeState(State *state) const override;
    double *getValueAddressAtIndex(State *state, unsigned int index) const override;
    void printState(const State *state, std::ostream &out) const override;
    void printSettings(std::ostream &out) const override;
    void registerProjections() override;
    void setup() override;
  
    protected:
    unsigned int dimension_;  
    RealVectorBounds bounds_;  
    std::vector<std::string> dimensionNames_;
    std::map<std::string, unsigned int> dimensionIndex_;
  
    private:
    std::size_t stateBytes_;
    };
  }
}
  

