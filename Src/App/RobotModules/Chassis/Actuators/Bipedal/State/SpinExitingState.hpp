#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

static PositionalPid spinExitingFollowVelPid{ 8.0f, 0.f, 0.1f, 0.001f, 0.f, 2.5f, 0.1f };

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

        const auto y = chassis_->deltaYaw;
        const auto w = chassis_->wz();

        // Equivalent to the original quadrant branches: choose PI when yaw half-plane and spin direction match.
        const auto isNegativeHalfPlane = (y < 0.f && y > -PI);
        targetDeltaYaw_ = ((w > 0.f) == isNegativeHalfPlane) ? PI : 0.f;

        LOG::info("ChassisSpinExiting", " enter");
    }

    void run() final
    {
        chassis_->setFdFeedforward();

        const auto wz = wzAngPid_->calc(PINYMOTOR::getMinorArc(chassis_->deltaYaw, targetDeltaYaw_), 0);
        refYaw_ = chassis_->isFloating() ? chassis_->yaw() : (PINYMOTOR::rangeMap(chassis_->yaw() + wz, -PI, PI));
        // WBR
        WBRController::States_s ref{};
        ref.yaw = refYaw_;
        chassis_->bal<WBRController::WBRControlMode_e::BALANCE>(ref);

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
        else if (std::fabs(chassis_->wz()) < 0.5f)
            return FSMState_e::IDLE;
        else
            return FSMState_e::SPIN_EXITING;
    }

    bool change() final
    {
        WBRController::States_s ref{};
        ref.yaw = chassis_->yaw() + chassis_->wz() * 0.001f;
        ref.dyaw = chassis_->wz();
        chassis_->bal<WBRController::WBRControlMode_e::SPIN>(ref);
        chassis_->ctrl();
        return std::fabs(chassis_->wz()) < 2.5f;
    }

private:
    Bipedal *chassis_;

    PID *wzAngPid_;

    float refYaw_ = 0.f;
    float targetDeltaYaw_ = 0.f;
};
} // namespace CHASSIS
