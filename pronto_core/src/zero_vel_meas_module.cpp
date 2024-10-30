#include "pronto_core/zero_vel_meas_module.hpp"
#include <iostream>

namespace pronto {

ZeroVelMeasModule::ZeroVelMeasModule(const ZeroVelMeasConfig &cfg)
{
    std::cout << "ZeroVelMeasModule::ZeroVelMeasModule" << std::endl;  // this prints

    // Set indices
    z_meas.resize(3);
    z_indices.resize(3);
    z_indices.head<3>() = RBIS::velocityInds();

    // Set covariance
    cov_zero_vel_meas.resize(3, 3);
    cov_zero_vel_meas.setZero();
    cov_zero_vel_meas.topLeftCorner<3, 3>() = std::pow(cfg.r_zero_vel_meas, 2) * Eigen::Matrix3d::Identity();
}

RBISUpdateInterface* ZeroVelMeasModule::processMessage(const PoseMeasurement *msg,
                                                    StateEstimator *est)
{
    z_meas.head<3>() = msg->linear_vel;
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
    init_cov.block<3, 3>(RBIS::velocity_ind, RBIS::velocity_ind) = cov_zero_vel_meas.topLeftCorner<3, 3>();

    std::cout << "Initialized ZeroVel velocity: "
              << init_state.velocity().transpose() << std::endl;

    return true;
}
} // namespace pronto