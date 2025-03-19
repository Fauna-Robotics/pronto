#pragma once
#include <pronto_core/sensing_module.hpp>
#include <sensor_msgs/JointState.h>
#include <ros/node_handle.h>
#include <pronto_biped_core/legodo_module.hpp>
#include <pronto_msgs/BipedCartesianPoses.h>
#include <pronto_msgs/FootWrenches.h>
#include <pronto_ros/pronto_ros_conversions.hpp>
#include <pronto_biped_commons/forward_kinematics.hpp>

namespace pronto {
namespace biped {

class LegOdometryHandler : public pronto::SensingModule<sensor_msgs::JointState> {
public:

  LegOdometryHandler() = delete;
  LegOdometryHandler(ros::NodeHandle& nh);

  RBISUpdateInterface* processMessage(const sensor_msgs::JointState *msg,
                                      StateEstimator *est) override;

  bool processMessageInit(const sensor_msgs::JointState *msg,
                          const std::map<std::string, bool> &sensor_initialized,
                          const RBIS &default_state,
                          const RBIM &default_cov,
                          RBIS &init_state,
                          RBIM &init_cov) override;

  void forceTorqueCallback(const pronto_msgs::FootWrenchesConstPtr& msg);
  void forwardKinematicsCallback(
      const pronto_msgs::BipedCartesianPosesConstPtr& msg);

 protected:
  // ROS node
  ros::NodeHandle nh_;

  // Subscribers
  ros::Subscriber force_torque_sub_;
  ros::Subscriber fk_sub_;

  // Publisher
  ros::Publisher primary_foot_pub_;

  // Latest messages
  pronto_msgs::BipedCartesianPoses latest_fk_msg_;
  JointState legodo_msg_;
  pronto::ForceTorqueSensorArray ft_msg_;

  // Parameters and Variables
  bool init = false;
  std::unique_ptr<LegOdometryModule> legodo_module_;
  LegOdometryConfig legodo_cfg_;
  std::vector<std::string> joint_names_;
  int active_joints_ = 15;
  std::unique_ptr<pronto::biped::BipedForwardKinematicsExternal> fk_;
};

}
}
