
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/config.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <stdio.h>


//    <!-- launch-prefix="gdb -ex run --args" -->





bool isStateValid(const ompl::base::State *state);
void plan();
