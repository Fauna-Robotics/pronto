#include "pronto_core/zero_vel_meas_module.hpp"
#include <iostream>

namespace pronto {

ZeroVelMeasModule::ZeroVelMeasModule(const ZeroVelMeasConfig &cfg)
{
    std::cout << "ZeroVelMeasModule::ZeroVelMeasModule" << std::endl;

    // Set indices
    z_meas.resize(6);
    z_indices.resize(6);
    z_indices.head<3>() = RBIS::velocityInds();
    z_indices.tail<3>() = RBIS::angularVelocityInds();

    // Set covariance
    cov_zero_vel_meas.resize(6, 6);
    cov_zero_vel_meas.setZero();
    cov_zero_vel_meas = std::pow(cfg.r_zero_vel_meas, 2) * Eigen::MatrixXd::Identity(6, 6);
}

RBISUpdateInterface* ZeroVelMeasModule::processMessage(const PoseMeasurement *msg, StateEstimator *est)
{
    z_meas.head<3>() = msg->linear_vel;
    z_meas.tail<3>() = msg->angular_vel;

    return new RBISIndexedMeasurement(z_indices,
                                      z_meas,
                                      cov_zero_vel_meas,
                                      RBISUpdateInterface::zero_vel_meas,
                                      msg->utime);
}

bool ZeroVelMeasModule::processMessageInit(const PoseMeasurement *msg,
                                        const std::map<std::string, bool> &sensor_initialized,
                                        const RBIS &default_state,
                                        const RBIM &default_cov,
                                        RBIS &init_state,
                                        RBIM &init_cov)
{
    init_state.utime = msg->utime;
    init_state.velocity() = msg->linear_vel;
    init_state.angularVelocity() = msg->angular_vel;
    init_cov.block<3, 3>(RBIS::velocity_ind, RBIS::velocity_ind) = cov_zero_vel_meas.topLeftCorner<3, 3>();
    init_cov.block<3, 3>(RBIS::angular_velocity_ind, RBIS::angular_velocity_ind) = cov_zero_vel_meas.bottomRightCorner<3, 3>();
    return true;
}
} // namespace pronto