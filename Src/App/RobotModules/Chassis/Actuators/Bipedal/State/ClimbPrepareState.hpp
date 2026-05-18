// TODO: merge IdleSate
#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "PidBasic.hpp"
#include "Smooth.hpp"

static PositionalPid climbPrepareFollowVelPid{ 2.f, 0.f, 0.05f, 0.001f, 0.f, 0.1f, 0.1f };

namespace CHASSIS {
class ClimbPrepareState : public FSMState<FSMState_e> {
    static constexpr float OFFSET_PHI = 0.25f;
    static constexpr float MAX_ACC_BACK = 0.5f;
    static constexpr float MAX_ACC_FRONT = 0.7f;
    static constexpr float MAX_SPEED = 1.2f;
    static constexpr float CLIMB_PREPARE_LENGTH = 0.31f;

public:
    ClimbPrepareState(Bipedal *_chassis)
            : FSMState<FSMState_e>(FSMState_e::CLIMB_PREPARE), chassis_(_chassis), wzAngPid_(&climbPrepareFollowVelPid)
    {
    }

    void enter() final
    {
        chassis_->enter();
        isPrepareFinish_ = false;
        stretchFinish_ = false;
        refYaw_ = chassis_->yaw();
        chassis_->setRoll(0);
        refLength_ = chassis_->length();
        targetRefLength_ = CLIMB_PREPARE_LENGTH;
        chassis_->needGimbalLaunch(true);
        chassis_->allowGimbalExitLaunch(false);
        LOG::info("ChassisClimbPrepare", " enter");
    }

    void run() final
    {
        targetRefLength_ += chassis_->msg.dl * 0.001f;
        refLength_ = SMOOTH::slopeFollowing(targetRefLength_, refLength_, 0.00035f);
        chassis_->setLength(refLength_);
        chassis_->setFdFeedforward();

        if (chassis_->detectLengthRange(targetRefLength_, 0.025f) || stretchFinish_)
            stretchFinish_ = true;

        if (chassis_->isFloating() && (chassis_->phiL() + chassis_->phiR()) / 2.f > 0.3f)
            isPrepareFinish_ = true;

        float targetYaw = 0.f;
        float wz = -wzAngPid_->calc(0, PINYMOTOR::getMinorArc(chassis_->deltaYaw, targetYaw));
        // float wz = 0;
        float ds = (chassis_->msg.vx * chassis_->cosYaw) - (chassis_->msg.vy * chassis_->sinYaw);

        // WBR
        WBRController::States_s ref{};
        refYaw_ = chassis_->isFloating() ? chassis_->yaw() : (PINYMOTOR::rangeMap(refYaw_ + wz, -PI, PI));
        chassis_->msg.vx = std::clamp(chassis_->msg.vx, -MAX_SPEED, MAX_SPEED);
        ref.ds = std::clamp(ds, chassis_->vel() - MAX_ACC_BACK, chassis_->vel() + MAX_ACC_FRONT);
        ref.yaw = refYaw_;
        ref.phir = ref.phil = stretchFinish_ ? OFFSET_PHI : 0.f;
        chassis_->bal<WBRController::WBRControlMode_e::CLIMB>(ref);

        chassis_->ctrl();
    }

    void exit() final
    {
        wzAngPid_->reset();
        LOG::info("ChassisClimbPrepare", " exit");
    }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        if (isPrepareFinish_)
            return FSMState_e::CLIMB;
        else if (!chassis_->msg.needClimb)
            return FSMState_e::IDLE;
        else
            return FSMState_e::CLIMB_PREPARE;
    }

private:
    Bipedal *chassis_;
    PositionalPid *wzAngPid_;
    float refYaw_ = 0;
    float refLength_ = 0;
    float targetRefLength_ = CLIMB_PREPARE_LENGTH;
    bool stretchFinish_ = false;
    bool isPrepareFinish_ = false;
};

} // namespace CHASSIS
