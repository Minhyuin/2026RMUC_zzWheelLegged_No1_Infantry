#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

static PositionalPid spinExitingFollowVelPid{ 7.0f, 0.f, 0.1f, 0.001f, 0.f, 2.f, 0.1f };

namespace CHASSIS {

class SpinExitingState : public FSMState<FSMState_e> {
public:
    SpinExitingState(Bipedal *_chassis)
            : FSMState<FSMState_e>(FSMState_e::SPIN_EXITING), chassis_(_chassis), wzAngPid_(&spinExitingFollowVelPid)
    {
    }

    void enter() final
    {
        chassis_->enter();
        float &y = chassis_->deltaYaw;
        float &&w = chassis_->wz();
        if (y < 0.f && y > -PI / 2.f) {
            if (w > 0.f)
                targetDeltaYaw_ = PI;
            else
                targetDeltaYaw_ = 0;
        } else if (y >= 0.f && y < PI / 2.f) {
            if (w > 0.f)
                targetDeltaYaw_ = 0;
            else
                targetDeltaYaw_ = PI;
        } else if (y <= -PI / 2.f && y > -PI) {
            if (w > 0.f)
                targetDeltaYaw_ = PI;
            else
                targetDeltaYaw_ = 0;
        } else {
            if (w > 0.f)
                targetDeltaYaw_ = 0;
            else
                targetDeltaYaw_ = PI;
        }

        LOG::info("ChassisSpinExiting", " enter");
    }

    void run() final
    {
        float wz = -wzAngPid_->calc(0, PINYMOTOR::getMinorArc(chassis_->deltaYaw, targetDeltaYaw_));
        refYaw_ = chassis_->isFloating() ? chassis_->yaw() : (PINYMOTOR::rangeMap(chassis_->yaw() + wz, -PI, PI));
        // WBR
        WBRController::States_s ref{};
        ref.yaw = refYaw_;
        chassis_->bal(ref);

        chassis_->ctrl();
    }

    void exit() final
    {
        wzAngPid_->reset();
        LOG::info("ChassisSpinExiting", " exit");
    }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP || chassis_->isTumble())
            return FSMState_e::STOP;
        else if (std::fabs(chassis_->wz()) < 0.1f)
            return FSMState_e::IDLE;
        else
            return FSMState_e::SPIN_EXITING;
    }

private:
    Bipedal *chassis_;

    PID *wzAngPid_;

    float refYaw_ = 0.f;
    float targetDeltaYaw_ = 0.f;
};
} // namespace CHASSIS
