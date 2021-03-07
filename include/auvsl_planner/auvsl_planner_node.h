
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
//#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <stdio.h>

#include "soil_table.h"




namespace ob = ompl::base;
namespace og = ompl::geometric;



TireSoilData get_soil_data_at(float x, float y);
float get_altitude(float x, float y);




bool isStateValid(const ob::State *state);
ob::ValidStateSamplerPtr allocDynamicVehicleStateSampler(const ob::SpaceInformation *si);
octomap::OcTree* newSimpleOctreeMap();
void plan();





class DynamicVehicleStateSampler : public ob::ValidStateSampler{
public:
  DynamicVehicleStateSampler(const ob::SpaceInformation *si);
  bool sample(ob::State *state) override;
  bool sampleNear(ob::State* state, const ob::State* near, const double distance) override;
protected:
  ompl::RNG rng_;
};
