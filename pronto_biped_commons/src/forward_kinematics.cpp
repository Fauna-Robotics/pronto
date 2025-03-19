#include "pronto_biped_commons/forward_kinematics.hpp"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

namespace pronto {
namespace biped {

BipedForwardKinematicsExternal::BipedForwardKinematicsExternal(
    const ros::NodeHandle& nh)
    : nh_(nh) {
  foot_pose_sub_ =
      nh_.subscribe("/biped/estimated_cartesian_poses", 1,
                    &BipedForwardKinematicsExternal::footPosesCallback, this);

  // Initialize the stored poses to identity
  left_foot_pose_.setIdentity();
  right_foot_pose_.setIdentity();
  left_received_ = false;
  right_received_ = false;
}

// Callback for BipedCartesianPoses messages.
void BipedForwardKinematicsExternal::footPosesCallback(
    const pronto_msgs::BipedCartesianPosesConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  bool left_found = false;
  bool right_found = false;

  // Loop through each CartesianPose in the message.
  for (const auto& cp : msg->cartesian_poses) {
    // Check joint_id: 15 for left foot, 16 for right foot.
    if (cp.joint_id == 15) {
      tf::Transform tf_transform;
      tf::poseMsgToTF(cp.cartesian_pose.pose, tf_transform);
      Eigen::Isometry3d iso;
      tf::poseTFToEigen(tf_transform, iso);
      left_foot_pose_ = iso;
      left_found = true;
    } else if (cp.joint_id == 16) {
      tf::Transform tf_transform;
      tf::poseMsgToTF(cp.cartesian_pose.pose, tf_transform);
      Eigen::Isometry3d iso;
      tf::poseTFToEigen(tf_transform, iso);
      right_foot_pose_ = iso;
      right_found = true;
    }
  }
  if (left_found)
    left_received_ = true;
  if (right_found)
    right_received_ = true;
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
