#pragma once

#include <rbdl/rbdl.h>

namespace rbd = RigidBodyDynamics;
namespace rbdm = RigidBodyDynamics::Math;

using namespace rbd;
using namespace rbdm;

typedef struct {
  float roll,pitch,yaw;
} EulerAngles;


float get_q_norm(rbdm::Quaternion q);
float get_yaw(rbdm::Vector4d q);
EulerAngles get_rpy(rbdm::Vector4d q);

rbdm::Matrix3d get_box_inertia(float mass, rbdm::Vector3d v_size);
rbdm::Matrix3d get_tire_inertia(float mass, float radius, float height);

rbdm::SpatialVector get_body_vel(rbdm::SpatialVector base_vel, rbdm::VectorNd X);
rbdm::Vector4d get_qnd(rbdm::Quaternion q, rbdm::Vector3d w);
