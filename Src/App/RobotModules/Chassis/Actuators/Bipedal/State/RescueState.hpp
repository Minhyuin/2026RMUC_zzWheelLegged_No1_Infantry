#pragma once

#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace CHASSIS {

class RescueState : public FSMState<FSMState_e> {
    static constexpr float RESCUE_L = 0.35f;
    static constexpr float RESCUE_DPHI = -0.002f;

    static constexpr uint16_t MAX_RESCUE_COUNT = 1000; // 1s

public:
    RescueState(Bipedal *_chassis) : FSMState<FSMState_e>(FSMState_e::RESCUE), chassis_(_chassis) {}

    void enter() final
    {
        rescueFinishFlag_ = false;
        LOG::info("ChassisRescue", "enter");
    }

    void run() final
    {
        if (chassis_->isOverturn()) {
            chassis_->setLengthSmooth(RESCUE_L, 0.05f);
            chassis_->setHipPhiRot(RESCUE_DPHI);
            rescueFinishCount_ = 0;
        } else {
            rescueFinishCount_++;
        }

        chassis_->ctrl();
    }

    void exit() final { LOG::info("ChassisRescue", "exit"); }

    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (rescueFinishCount_ > MAX_RESCUE_COUNT)
            return FSMState_e::LAUNCH;
        // TODO: the other state
        else
            return FSMState_e::RESCUE;
    }

private:
    Bipedal *chassis_;
    uint16_t rescueFinishCount_ = 0;
    bool rescueFinishFlag_ = false;
};

} // namespace CHASSIS
