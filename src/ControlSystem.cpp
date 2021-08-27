#include "ControlSystem.h"
#include "utils.h"
#include <algorithm>
#include <ros/ros.h>

#include <tf/tf.h>

using namespace auvsl;


ControlSystem::ControlSystem(){

}

ControlSystem::~ControlSystem(){

}



SimpleControlSystem::SimpleControlSystem(){

}

SimpleControlSystem::~SimpleControlSystem(){

}



int SimpleControlSystem::initialize(){
  return 1;
}

int SimpleControlSystem::computeVelocityCommand(std::vector<Vector2f> waypoints, geometry_msgs::Pose pose, float &v_forward, float &v_angular){
  tf::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf::Matrix3x3 mat(quat);
  double roll;
  double pitch;
  double yaw;
  mat.getRPY(roll, pitch, yaw);
  float dx = waypoints[0][0] - pose.position.x;
  float dy = waypoints[0][1] - pose.position.y;
  
  float dist = sqrtf(dx*dx + dy*dy);
  
  float target_angle = atan2f(dy, dx);
  
  float ang_disp = target_angle - yaw;
  if(ang_disp < -M_PI){
    ang_disp += (2*M_PI);
  }
  else if(ang_disp > M_PI){
    ang_disp -= (2*M_PI);
  }

  v_angular = std::max(std::min(ang_disp*.5f, 4.0f), -4.0f);
  v_forward = std::max(std::min((dist*.5f)/(1+(ang_disp*ang_disp)), 4.0f), -4.0f);
  if(fabs(ang_disp) > .5){
    v_forward *= .1;
  }
  if(v_forward < 0){
    v_angular *= -1;
  }

  
  return 1;
}







