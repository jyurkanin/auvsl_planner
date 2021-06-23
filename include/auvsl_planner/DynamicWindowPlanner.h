#include "VehicleStateSpace.h"
#include "TerrainMap.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/config.h>

//#include <rbdl/Quaternion.h>
#include <rbdl/rbdl.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <stdio.h>
#include <vector>
#include <iostream>


class DynamicWindowPlanner {
public:
  DynamicWindowPlanner();
  ~DynamicWindowPlanner();

  void initPlanner();
  void stepPlanner();

private:

  ompl::control::StatePropagatorPtr dynamic_model;
};
