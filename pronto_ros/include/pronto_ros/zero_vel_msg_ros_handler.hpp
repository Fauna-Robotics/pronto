#pragma once
#include <ros/node_handle.h>
#include <pronto_core/zero_vel_meas_module.hpp>
#include <geometry_msgs/TwistStamped.h>

namespace pronto {

class ZeroVelHandlerROS : public SensingModule<geometry_msgs::TwistStamped> {
public:
    ZeroVelHandlerROS(ros::NodeHandle& nh);

    RBISUpdateInterface* processMessage(const geometry_msgs::TwistStamped *msg,
                                        StateEstimator *est);

    bool processMessageInit(const geometry_msgs::TwistStamped *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov);
private:
    ros::NodeHandle& nh_;
    std::shared_ptr<ZeroVelMeasModule> zero_vel_module_;
    PoseMeasurement zero_vel_meas_;
};

}