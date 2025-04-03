#include "pronto_biped_commons/forward_kinematics.hpp"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace pronto {
namespace biped {

// Define constants for the foot joint IDs. These depend on the URDF
static const int LEFT_FOOT_JOINT_ID = 15;
static const int RIGHT_FOOT_JOINT_ID = 16;

BipedForwardKinematicsExternal::BipedForwardKinematicsExternal(
    const ros::NodeHandle& nh)
    : nh_(nh) {
  foot_pose_sub_ =
      nh_.subscribe("/biped/cartesian_poses", 1,
                    &BipedForwardKinematicsExternal::footPosesCallback, this);

  // Initialize the stored poses to identity
  left_foot_pose_.setIdentity();
  right_foot_pose_.setIdentity();
  left_received_ = false;
  right_received_ = false;
}

// Callback for BipedCartesianPoses messages
void BipedForwardKinematicsExternal::footPosesCallback(
    const pronto_msgs::BipedCartesianPosesConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  bool left_found = false;
  bool right_found = false;

  // Loop through each CartesianPose in the message
  for (const auto& cp : msg->cartesian_poses) {
    Eigen::Isometry3d iso;
    tf2::fromMsg(cp.cartesian_pose.pose, iso);
    if (cp.joint_id == LEFT_FOOT_JOINT_ID) {
      left_foot_pose_ = iso;
      left_found = true;
    } else if (cp.joint_id == RIGHT_FOOT_JOINT_ID) {
      right_foot_pose_ = iso;
      right_found = true;
    }

    if (left_found && right_found)
      break;
  }
  if (left_found) left_received_ = true;
  if (right_found) right_received_ = true;
}

bool BipedForwardKinematicsExternal::getLeftFootPose(
    const BipedForwardKinematicsExternal::JointState& q,
    BipedForwardKinematicsExternal::Transform& x) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!left_received_)
    return false;
  x = left_foot_pose_;
  return true;
}

bool BipedForwardKinematicsExternal::getRightFootPose(
    const BipedForwardKinematicsExternal::JointState& q,
    BipedForwardKinematicsExternal::Transform& x) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!right_received_)
    return false;
  x = right_foot_pose_;
  return true;
}

BipedForwardKinematicsExternal::Transform
BipedForwardKinematicsExternal::getLeftFootPose(
    const BipedForwardKinematicsExternal::JointState& q) {
  BipedForwardKinematicsExternal::Transform x;
  if (getLeftFootPose(q, x))
    return x;
  else
    return BipedForwardKinematicsExternal::Transform::Identity();
}

BipedForwardKinematicsExternal::Transform
BipedForwardKinematicsExternal::getRightFootPose(
    const BipedForwardKinematicsExternal::JointState& q) {
  BipedForwardKinematicsExternal::Transform x;
  if (getRightFootPose(q, x))
    return x;
  else
    return BipedForwardKinematicsExternal::Transform::Identity();
}

}  // namespace biped
}  // namespace pronto
