#include "pronto_ros/zero_vel_msg_ros_handler.hpp"
#include "pronto_ros/pronto_ros_conversions.hpp"

namespace pronto {

ZeroVelHandlerROS::ZeroVelHandlerROS(ros::NodeHandle &nh) : nh_(nh) {
    ZeroVelMeasConfig cfg;
    std::string prefix = "zero_vel_meas/";
    if(!nh_.getParam(prefix + "r_zero_vel_meas", cfg.r_zero_vel_meas)){
        ROS_WARN("Param \"r_zero_vel_meas\" not found. Setting to 0.0001");
        cfg.r_zero_vel_meas = 0.0001;  // default covariance
    }

    zero_vel_module_.reset(new ZeroVelMeasModule(cfg));
}

RBISUpdateInterface* ZeroVelHandlerROS::processMessage(const geometry_msgs::TwistStamped *msg,
                                    StateEstimator *est)
{
    zeroVelMsgFromROS(*msg, zero_vel_meas_);
    return zero_vel_module_->processMessage(&zero_vel_meas_,est);
}

bool ZeroVelHandlerROS::processMessageInit(const geometry_msgs::TwistStamped *msg,
                        const std::map<std::string, bool> &sensor_initialized,
                        const RBIS &default_state,
                        const RBIM &default_cov,
                        RBIS &init_state,
                        RBIM &init_cov)
{
    zeroVelMsgFromROS(*msg, zero_vel_meas_);
    return zero_vel_module_->processMessageInit(&zero_vel_meas_,
                                            sensor_initialized,
                                            default_state,
                                            default_cov,
                                            init_state,
                                            init_cov);
}
}
