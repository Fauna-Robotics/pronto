#include "pronto_biped_ros/legodo_handler_ros.hpp"
#include <pronto_ros/pronto_ros_conversions.hpp>
#include <std_msgs/String.h>

namespace pronto {
namespace biped {

LegOdometryHandler::LegOdometryHandler(ros::NodeHandle& nh)
    : nh_(nh), legodo_msg_(15) {
  std::string prefix = "/pronto/legodo/";
  if(!nh_.getParam(prefix + "torque_adjustment", legodo_cfg_.use_torque_adjustment_)){
    ROS_WARN_STREAM("Couldn't find parameter \"torque_adjustment\"."
                    << " Using default : "
                    << legodo_cfg_.use_torque_adjustment_);
  }

  if(!nh_.getParam(prefix + "adjustment_gain", legodo_cfg_.torque_adj_gains_)){
    ROS_WARN_STREAM("Couldn't find parameter \"adjustment_gain\"."
                    << " Not using torque adjustment");
    legodo_cfg_.use_torque_adjustment_ = false;
  }

  if(!nh_.getParam(prefix + "adjustment_joints", legodo_cfg_.torque_adj_names_)){
    ROS_WARN_STREAM("Couldn't find parameter \"adjustment_joints\"."
                    << " Not using torque adjustment");
    legodo_cfg_.use_torque_adjustment_ = false;
  }

  if(!nh_.getParam(prefix + "zero_initial_velocity", legodo_cfg_.zero_initial_velocity)){
    ROS_WARN_STREAM("Couldn't find parameter \"torque_adjustment\"."
                    << " Using default : "
                    << legodo_cfg_.zero_initial_velocity);
  }
  legodo_cfg_.common_cfg.mode_ = LegOdometryMode::LIN_RATE;
  std::string mode;
  if(!nh_.getParam(prefix + "mode", mode)){
    ROS_WARN_STREAM("Couldn't find parameter \"torque_adjustment\"."
                    << " Using default : lin_rate");
  } else if(mode.compare("pos_and_lin_rate") == 0){
    legodo_cfg_.common_cfg.mode_ = LegOdometryMode::POSITION_AND_LIN_RATE;
  } else if(mode.compare("lin_rot_rate") == 0){
    legodo_cfg_.common_cfg.mode_ = LegOdometryMode::LIN_AND_ROT_RATE;
  } else if(mode.compare("rot_rate") == 0){
    legodo_cfg_.common_cfg.mode_ = LegOdometryMode::ROT_RATE;
  }

  if(!nh_.getParam(prefix + "r_vang", legodo_cfg_.common_cfg.R_legodo_vang_)){
    ROS_WARN_STREAM("Couldn't find parameter \"r_vang\"."
                    << " Using default : "
                    << legodo_cfg_.common_cfg.R_legodo_vang_);
  }
  if(!nh_.getParam(prefix + "r_vang_uncertain", legodo_cfg_.common_cfg.R_legodo_vang_uncertain_))
  {
    ROS_WARN_STREAM("Couldn't find parameter \"r_vang_uncertain\"."
                    << " Using default : "
                    << legodo_cfg_.common_cfg.R_legodo_vang_uncertain_);
  }
  if(!nh_.getParam(prefix + "r_vxyz", legodo_cfg_.common_cfg.R_legodo_vxyz_))
  {
    ROS_WARN_STREAM("Couldn't find parameter \"r_vxyz\"."
                    << " Using default : "
                    << legodo_cfg_.common_cfg.R_legodo_vxyz_);
  }
  if(!nh_.getParam(prefix + "r_vxyz_uncertain", legodo_cfg_.common_cfg.R_legodo_vxyz_uncertain_))
  {
    ROS_WARN_STREAM("Couldn't find parameter \"r_vxyz_uncertain\"."
                    << " Using default : "
                    << legodo_cfg_.common_cfg.R_legodo_vxyz_uncertain_);
  }
  if(!nh_.getParam(prefix + "r_xyz", legodo_cfg_.common_cfg.R_legodo_xyz_))
  {
    ROS_WARN_STREAM("Couldn't find parameter \"r_xyz\"."
                    << " Using default : "
                    << legodo_cfg_.common_cfg.R_legodo_xyz_);
  }
  if(!nh_.getParam(prefix + "verbose", legodo_cfg_.common_cfg.verbose))
  {
    ROS_WARN_STREAM("Couldn't find parameter \"verbose\"."
                    << " Using default : "
                    << std::boolalpha <<  legodo_cfg_.common_cfg.verbose);
  }
  if(!nh_.getParam(prefix + "verbose", legodo_cfg_.common_cfg.verbose))
  {
    ROS_WARN_STREAM("Couldn't find parameter \"verbose\"."
                    << " Using default : "
                    << std::boolalpha << legodo_cfg_.common_cfg.verbose);
  }
  legodo_cfg_.odometer_cfg.filter_mode = FilterJointMode::NONE;
  if(!nh_.getParam(prefix + "filter_joint_positions", mode))
  {
    ROS_WARN_STREAM("Couldn't find parameter \"filter_joint_positions\"."
                    << " Using default : none");
  } else if(mode.compare("lowpass") == 0){
    legodo_cfg_.odometer_cfg.filter_mode = FilterJointMode::LOWPASS;
  } else if (mode.compare("kalman") == 0){
    legodo_cfg_.odometer_cfg.filter_mode = FilterJointMode::KALMAN;
  }
  if(!nh_.getParam(prefix + "use_controller_input", legodo_cfg_.odometer_cfg.use_controller_input))
  {
    ROS_WARN_STREAM("Couldn't find parameter \"use_controller_input\"."
                    << " Using default: "
                    << std::boolalpha << legodo_cfg_.odometer_cfg.use_controller_input);
  }
  // Schmitt Trigger Parameters
  if (!nh_.getParam(prefix + "schmitt_low_threshold", legodo_cfg_.odometer_cfg.schmitt_low_threshold)) {
    ROS_WARN_STREAM(
        "Couldn't find parameter \"schmitt_low_threshold\". Using default: "
        << legodo_cfg_.odometer_cfg.schmitt_low_threshold);
  }

  if (!nh_.getParam(prefix + "schmitt_high_threshold", legodo_cfg_.odometer_cfg.schmitt_high_threshold)) {
    ROS_WARN_STREAM(
        "Couldn't find parameter \"schmitt_high_threshold\". Using default: "
        << legodo_cfg_.odometer_cfg.schmitt_high_threshold);
  }

  if (!nh_.getParam(prefix + "schmitt_low_delay", legodo_cfg_.odometer_cfg.schmitt_low_delay)) {
    ROS_WARN_STREAM(
        "Couldn't find parameter \"schmitt_low_delay\". Using default: "
        << legodo_cfg_.odometer_cfg.schmitt_low_delay);
  }

  if (!nh_.getParam(prefix + "schmitt_high_delay", legodo_cfg_.odometer_cfg.schmitt_high_delay)) {
    ROS_WARN_STREAM(
        "Couldn't find parameter \"schmitt_high_delay\". Using default: "
        << legodo_cfg_.odometer_cfg.schmitt_high_delay);
  }

  // Foot Wrench Subscriber
  force_torque_sub_ =
      nh_.subscribe("/biped/estimated_foot_wrench", 10,
                    &LegOdometryHandler::forceTorqueCallback, this);

  // Forward Kinematics Subscriber
  fk_sub_ = nh_.subscribe("/biped/cartesian_poses", 10,
                          &LegOdometryHandler::forwardKinematicsCallback, this);

  if(!nh_.getParam(prefix + "initialization_mode", legodo_cfg_.odometer_cfg.initialization_mode)){
    ROS_WARN_STREAM("Couldn't find parameter \"initialization_mode\".");
  }

  if(!nh_.getParam(prefix + "left_foot_name", legodo_cfg_.odometer_cfg.left_foot_name)){
    ROS_WARN_STREAM("Couldn't find parameter \"left_foot_name\".");
  }

  if(!nh_.getParam(prefix + "right_foot_name", legodo_cfg_.odometer_cfg.right_foot_name)){
    ROS_WARN_STREAM("Couldn't find parameter \"right_foot_name\".");
  }


  std::string filter_mode;

  if(!nh_.getParam(prefix + "filter_joint_positions", filter_mode)){
    ROS_WARN_STREAM("Couldn't find parameter \"filter_joint_positions\".");
  }

  if(filter_mode.compare("lowpass") == 0){
    legodo_cfg_.odometer_cfg.filter_mode = FilterJointMode::LOWPASS;
  } else if(filter_mode.compare("kalman") == 0){
    legodo_cfg_.odometer_cfg.filter_mode = FilterJointMode::KALMAN;
  } else {
    legodo_cfg_.odometer_cfg.filter_mode = FilterJointMode::NONE;
  }

  if(!nh_.getParam(prefix + "filter_contact_events", legodo_cfg_.odometer_cfg.filter_contact_events)){
    ROS_WARN_STREAM("Couldn't find parameter \"filter_contact_events\".");
  }

  if(!nh_.getParam(prefix + "publish_diagnostics", legodo_cfg_.odometer_cfg.publish_diagnostics)){
    ROS_WARN_STREAM("Couldn't find parameter \"publish_diagnostics\".");
  }

  if(!nh_.getParam(prefix + "active_joints", active_joints_)){
    ROS_WARN_STREAM("Couldn't find parameter \"active_joints\".");
  }

  fk_.reset(new pronto::biped::BipedForwardKinematicsExternal(nh));

  legodo_module_.reset(new LegOdometryModule(legodo_cfg_));

  // Create a publisher for the primary foot ID
  primary_foot_pub_ = nh_.advertise<std_msgs::String>("/biped/primary_foot", 1);
}

RBISUpdateInterface* LegOdometryHandler::processMessage(const sensor_msgs::JointState *msg,
                                                        StateEstimator *est)
{
  if(msg->position.size() < active_joints_){
    // different publisher might send messages to the /joint_state topic
    // we use the expected length of the joint vector to check that we
    // are processing the right one
    return nullptr;
  }
  if(!init){
    joint_names_ = msg->name;
    for(const auto& el : joint_names_){
      std::cerr << "Joint " << el << std::endl;
    }
    init = true;
  }
  jointStateFromROS(*msg, legodo_msg_);
  RBISUpdateInterface* meas = legodo_module_->processMessage(&legodo_msg_, est);

  // Publish primary foot
  int foot_id = legodo_module_->getPrimaryFootID();
  std::string primary_foot_id;
  switch (foot_id) {
    case 0:  // FootID::LEFT
      primary_foot_id = "LEFT";
      break;
    case 1:  // FootID::RIGHT
      primary_foot_id = "RIGHT";
      break;
    default:
      primary_foot_id = "UNKNOWN";
      break;
  }
  std_msgs::String foot_msg;
  foot_msg.data = primary_foot_id;
  primary_foot_pub_.publish(foot_msg);

  return meas;
}

bool LegOdometryHandler::processMessageInit(const sensor_msgs::JointState *msg,
                                            const std::map<std::string, bool> &sensor_initialized,
                                            const RBIS &default_state,
                                            const RBIM &default_cov,
                                            RBIS &init_state,
                                            RBIM &init_cov)
{
  if(!init){
    joint_names_ = msg->name;
    init = true;
  }
  jointStateFromROS(*msg, legodo_msg_);
  return legodo_module_->processMessageInit(&legodo_msg_, sensor_initialized, default_state, default_cov, init_state, init_cov);
}


void LegOdometryHandler::forceTorqueCallback(const pronto_msgs::FootWrenchesConstPtr& msg){
  forceTorqueFromROS(*msg, ft_msg_);
  legodo_module_->setForceTorque(ft_msg_);
}

void LegOdometryHandler::forwardKinematicsCallback(
    const pronto_msgs::BipedCartesianPosesConstPtr& msg) {
  // Pass the FK message to the leg odometry module.
  legodo_module_->updateForwardKinematics(*msg);
  // Store the latest FK measurement.
  latest_fk_msg_ = *msg;
  ROS_DEBUG("Received new forward kinematics data");
}

}  // namespace biped
}  // namespace pronto
