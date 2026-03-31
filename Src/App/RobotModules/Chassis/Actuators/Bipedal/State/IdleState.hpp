#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "PidBasic.hpp"
#include "Smooth.hpp"

static PositionalPid idleFollowVelPid{ 7.0f, 1.f, 0.1f, 0.001f, 0.05f, 3.f, 0.1f };

namespace CHASSIS {
class IdleState : public FSMState<FSMState_e> {
    static constexpr float OFFSET_Q = 0.f;
    static constexpr float MAX_ACC_BACK = 0.7f;
    static constexpr float MAX_ACC_FRONT = 0.7f;
    static constexpr float IDLE_L = WheelLegged::MIN_L;

public:
    IdleState(Bipedal *_chassis)
            : FSMState<FSMState_e>(FSMState_e::IDLE), chassis_(_chassis), wzAngPid_(&idleFollowVelPid)
    {
    }

    void enter() final
    {
        chassis_->enter();

        chassis_->enableRoll();
        refYaw_ = chassis_->yaw();
        refRoll_ = 0.f;
        refLength_ = chassis_->length();
        targetRefLength_ = IDLE_L;
        isJumpReady_ = false;
        needJumpPrev_ = chassis_->msg.needJump;
        chassis_->needGimbalLaunch(false);
        chassis_->allowGimbalExitLaunch(true);

        LOG::info("ChassisIdle", " enter");
    }

    void run() final
    {
        if (chassis_->msg.reset) {
            targetRefLength_ = IDLE_L;
            refRoll_ = 0.f;
        }
        targetRefLength_ += chassis_->msg.dl * 0.001f;
        refLength_ = SMOOTH::slopeFollowing(targetRefLength_, refLength_, 0.0002f);
        chassis_->setLength(refLength_);
        chassis_->setFdFeedforward();
        refRoll_ += chassis_->msg.droll * 0.001f;
        chassis_->setRoll(refRoll_);

        float targetYaw = (fabs(chassis_->deltaYaw) > PI / 2.f) ? PI : 0.f;
        float wz = -wzAngPid_->calc(0, PINYMOTOR::getMinorArc(chassis_->deltaYaw, targetYaw));
        // float wz = 0;
        float ds = (chassis_->msg.vx * chassis_->cosYaw) - (chassis_->msg.vy * chassis_->sinYaw);

        // WBR
        WBRController::States_s ref{};
        refYaw_ = chassis_->isFloating() ? chassis_->yaw() : (PINYMOTOR::rangeMap(chassis_->yaw() + wz, -PI, PI));
        ref.ds = std::clamp(ds, chassis_->vel() - MAX_ACC_BACK, chassis_->vel() + MAX_ACC_FRONT);
        ref.yaw = refYaw_;
        ref.q = OFFSET_Q;
        chassis_->bal<WBRController::WBRControlMode_e::BALANCE>(ref);

        chassis_->ctrl();

        isJumpReady_ = needJumpPrev_ && !chassis_->msg.needJump;
        needJumpPrev_ = chassis_->msg.needJump;
    }

    void exit() final
    {
        wzAngPid_->reset();
        LOG::info("ChassisIdle", " exit");
    }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP || chassis_->isTumble())
            return FSMState_e::STOP;
        if (isJumpReady_)
            return FSMState_e::JUMP;
        else if (chassis_->msg.state == FSMState_e::IDLE)
            return FSMState_e::IDLE;
        else if (chassis_->msg.state == FSMState_e::TANK)
            return FSMState_e::TANK;
        else if (chassis_->msg.state == FSMState_e::SPIN)
            return FSMState_e::SPIN;
        else if (chassis_->msg.state == FSMState_e::TEST)
            return FSMState_e::TEST;
        else if (chassis_->msg.state == FSMState_e::CLIMB_PREPARE)
            return FSMState_e::CLIMB_PREPARE;
        else
            return FSMState_e::IDLE;
    }

private:
    Bipedal *chassis_;
    PositionalPid *wzAngPid_;
    float refYaw_ = 0;
    float refLength_ = 0;
    float targetRefLength_ = IDLE_L;
    float refRoll_ = 0;

    bool isJumpReady_ = false;
    bool needJumpPrev_ = false;
};

} // namespace CHASSIS
