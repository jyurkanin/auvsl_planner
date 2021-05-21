#include <iostream>
#include <thread>
#include <vector>

#include "auvsl_planner_node.h"
#include "GlobalParams.h"
#include "PlannerVisualizer.h"
#include "LocalPlanner.h"
#include "RRTGlobalPlanner.h"

#include <ompl/util/RandomNumbers.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

#include <rbdl/Quaternion.h>

//launch-prefix="gdb -ex run --args




int main(int argc, char **argv){
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::String>("Global_Planner", 1); //buffer size is one. Only one global plan needed

  GlobalParams::load_params(&nh);

  ros::Rate loop_rate(10);

  while(ros::ok()){
    ompl::RNG::setSeed(GlobalParams::get_seed());
    //pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
