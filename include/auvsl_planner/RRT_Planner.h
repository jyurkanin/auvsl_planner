#pragma once


#include <ompl/base/SpaceInformation.h>
#include "VehicleStateSpace.h"
#include "VehicleRRT.h"
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/config.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <stdio.h>


//    <!-- launch-prefix="gdb -ex run --args" -->



typedef struct{
  float x, y; //bottom left
  float width, height;
} Rectangle;


bool isStateValid(const ompl::base::State *state);
void plan();
