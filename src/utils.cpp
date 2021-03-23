#include "utils.h"
#include <stdio.h>

Matrix3d get_box_inertia(float mass, Vector3d v_size){
  Matrix3d temp;
  float v1 = mass*(v_size[1]*v_size[1] + v_size[2]*v_size[2])/12;
  float v2 = mass*(v_size[0]*v_size[0] + v_size[2]*v_size[2])/12;
  float v3 = mass*(v_size[0]*v_size[0] + v_size[1]*v_size[1])/12;

  temp << v1, 0, 0,   0,v2, 0,   0,0,v3;
  return temp;
}

//assumes axis of tire is in the y direction.
Matrix3d get_tire_inertia(float mass, float radius, float height){
  Matrix3d temp;
  float ibig = mass*radius*radius/2;
  float ismall = mass*((3*radius*radius) + (height*height))/12.0;

  temp << ismall,0,0,   0,ibig,0,   0,0,ismall;
  return temp;
}

float get_yaw(Vector4d q){
  return get_rpy(q).yaw;
}

EulerAngles get_rpy(Vector4d q){
  EulerAngles ret;
  float siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1]);
  float cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
  ret.yaw = atan2f(siny_cosp, cosy_cosp);
  
  // roll (x-axis rotation)
  float sinr_cosp = 2 * (q[3] * q[0] + q[1] * q[2]);
  float cosr_cosp = 1 - 2 * (q[0] * q[0] + q[1] * q[1]);
  ret.roll = atan2f(sinr_cosp, cosr_cosp);
  
  // pitch (y-axis rotation)
  float sinp = 2 * (q[3] * q[1] - q[2] * q[0]);
  if (fabs(sinp) >= 1)
    ret.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    ret.pitch = asinf(sinp);
  
  return ret;
}

float get_q_norm(Quaternion q){
  return sqrtf((q[0]*q[0]) + (q[1]*q[1]) + (q[2]*q[2]) + (q[3]*q[3]));
}

SpatialVector get_body_vel(SpatialVector base_vel, VectorNd X){
    Quaternion quat(X[3], X[4], X[5], X[10]);
    Vector3d r(X[0], X[1], X[2]);
    SpatialTransform X_base(quat.toMatrix(), r);
    return X_base.apply(base_vel); //transform from world to body coordinates.    
}


//calculate quaternion derivative
Vector4d get_qnd(Quaternion q, Vector3d w){
  float Kstab = .1; //magic number from spatialv2 source. lol.
  Vector4d temp;
  Eigen::Matrix4d Q;
  //Convert 1->3, 2->0, 3->1, 4->2
  Q << q[3], -q[0], -q[1], -q[2],   q[0], q[3], -q[2], q[1],   q[1], q[2], q[3], -q[0],   q[2], -q[1], q[0], q[3];
  temp[0] = Kstab*w.norm()*(1-q.norm());
  temp[1] = w[0];
  temp[2] = w[1];
  temp[3] = w[2];
  return .5*Q*temp;
}
