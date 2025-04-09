#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <pronto_msgs/BipedCartesianPoses.h>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <mutex>
#include <pronto_biped_core/biped_forward_kinematics.hpp>

namespace pronto {
namespace biped {

class BipedForwardKinematicsExternal : public BipedForwardKinematics {
 public:
  using Transform = Eigen::Isometry3d;

  BipedForwardKinematicsExternal(const ros::NodeHandle& nh);

  // Implement the pure virtual functions from the base class:
  Transform getLeftFootPose(const JointState& q) override;
  Transform getRightFootPose(const JointState& q) override;

  bool getLeftFootPose(const JointState& q, Transform& x) override;
  bool getRightFootPose(const JointState& q, Transform& x) override;

 private:
  void footPosesCallback(const pronto_msgs::BipedCartesianPosesConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber foot_pose_sub_;

  // store the latest foot pose
  std::mutex mtx_;
  Transform left_foot_pose_;
  Transform right_foot_pose_;
  bool left_received_ = false;
  bool right_received_ = false;
};

}  // namespace biped
}  // namespace pronto
