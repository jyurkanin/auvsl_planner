#include <rbdl/rbdl.h>

#pragma once

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

typedef struct {
  float roll,pitch,yaw;
} EulerAngles;

float get_q_norm(Quaternion q);
float get_yaw(Vector4d q);
EulerAngles get_rpy(Vector4d q);

Matrix3d get_box_inertia(float mass, Vector3d v_size);
Matrix3d get_tire_inertia(float mass, float radius, float height);

SpatialVector get_body_vel(SpatialVector base_vel, VectorNd X);
Vector4d get_qnd(Quaternion q, Vector3d w);
