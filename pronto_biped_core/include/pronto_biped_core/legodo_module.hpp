#pragma once

#include <pronto_core/sensing_module.hpp>
#include <pronto_core/definitions.hpp>
#include "pronto_biped_core/leg_estimate.hpp"
#include "pronto_biped_core/legodo_common.hpp"
#include "pronto_msgs/BipedCartesianPoses.h"
#include <pronto_utils/torque_adjustment.hpp> // torque adjustment

namespace pronto {
namespace biped {

struct LegOdometryConfig {
    LegOdoCommonConfig common_cfg;
    LegOdometerConfig odometer_cfg;
    bool use_torque_adjustment_ = false;
    std::vector<float> torque_adj_gains_;
    std::vector<std::string> torque_adj_names_;
    int zero_initial_velocity = 0;
};

class LegOdometryModule : SensingModule<JointState> {
public:
    LegOdometryModule(const LegOdometryConfig& cfg);

    RBISUpdateInterface* processMessage(const JointState *msg,
                                        StateEstimator *est);

    bool processMessageInit(const JointState *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov);

    void setControllerInput(const int& n_contacts_left,
                            const int& n_contacts_right);

    void setForceTorque(const ForceTorqueSensorArray& array);

    void updateForwardKinematics(const pronto_msgs::BipedCartesianPoses& fk_msg);

    int getPrimaryFootID() const {
        if (leg_est_) {
        return leg_est_->getPrimaryFootID();
        }
        return -1;
    }

protected:
    std::shared_ptr<LegEstimator> leg_est_;
    std::shared_ptr<LegOdoCommon> leg_odo_common_;
    bool force_torque_init_ = false;
    PoseMeasurement world_to_body_full_;
    ForceTorqueSensorArray force_torque_;
    bool use_torque_adjustment_ = false;
    std::shared_ptr<EstimateTools::TorqueAdjustment> torque_adjustment_;
    int zero_initial_velocity = 0;

    // Contact points of the feet deemed to be in contact:
    int n_control_contacts_left_ = -1;
    int n_control_contacts_right_ = -1;
    int verbose_ = 2;

};

} // namespace biped
} // namespace pronto
