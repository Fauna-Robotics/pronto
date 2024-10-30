#pragma once

#include "pronto_core/sensing_module.hpp"
#include "pronto_core/definitions.hpp"

namespace pronto {

struct ZeroVelMeasConfig {
    double r_zero_vel_meas;
};

class ZeroVelMeasModule : SensingModule<PoseMeasurement> {

public:
    // the indices and matrix will be 6 dimensional tops
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 6, 1> MeasVector;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, 6, 6> MeasCovMatrix;
    typedef Eigen::Matrix<int, Eigen::Dynamic, 1, 0, 6, 1> MeasIndices;

public:
    ZeroVelMeasModule(const ZeroVelMeasConfig& cfg);

    RBISUpdateInterface* processMessage(const PoseMeasurement *msg,
                                        StateEstimator *est);

    bool processMessageInit(const PoseMeasurement *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov);

protected:
  MeasIndices z_indices;
  MeasVector z_meas;
  MeasCovMatrix cov_zero_vel_meas;
};
} // namespace pronto