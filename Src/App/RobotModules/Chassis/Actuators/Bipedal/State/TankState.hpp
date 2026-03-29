#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "PidBasic.hpp"
#include "Smooth.hpp"

static PositionalPid tankFollowVelPid{ 5.0f, 1.f, 0.05f, 0.001f, 0.1f, 1.5f, 0.1f };

namespace CHASSIS {
class TankState : public FSMState<FSMState_e> {
    static constexpr float OFFSET_Q = 0.f;
    static constexpr float MAX_ACC_BACK = 1.f;
    static constexpr float MAX_ACC_FRONT = 1.f;

public:
    TankState(Bipedal *_chassis) : FSMState<FSMState_e>(FSMState_e::TANK), chassis_(_chassis) {}

    void enter() final
    {
        chassis_->enter();

        chassis_->enableRoll();
        chassis_->setRoll(0);
        enterYaw_ = refYaw_ = chassis_->yaw();
        refRoll_ = 0.f;
        refLength_ = chassis_->length();
        targetRefLength_ = refLength_;
        chassis_->needGimbalLaunch(false);
        chassis_->allowGimbalExitLaunch(true);

        wzAngPid_ = &tankFollowVelPid;
        LOG::info("ChassisTank", " enter");
    }

    void run() final
    {
        targetRefLength_ += chassis_->msg.dl * 0.001f;
        refLength_ = SMOOTH::slopeFollowing(targetRefLength_, refLength_, 0.0002f);
        chassis_->setLength(refLength_);
        chassis_->setFdFeedforward();

        float wz = -wzAngPid_->calc(0, PINYMOTOR::getMinorArc(chassis_->yaw(), enterYaw_));
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
    }

    void exit() final
    {
        wzAngPid_->reset();
        LOG::info("ChassisTank", " exit");
    }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP || chassis_->isTumble())
            return FSMState_e::STOP;
        else if (chassis_->msg.state == FSMState_e::IDLE)
            return FSMState_e::IDLE;
        else if (chassis_->msg.state == FSMState_e::TANK)
            return FSMState_e::TANK;
        else if (chassis_->msg.state == FSMState_e::SPIN)
            return FSMState_e::SPIN;
        else if (chassis_->msg.state == FSMState_e::CLIMB_PREPARE)
            return FSMState_e::CLIMB_PREPARE;
        else
            return FSMState_e::TANK;
    }

private:
    Bipedal *chassis_;
    PositionalPid *wzAngPid_;
    float enterYaw_ = 0;
    float refYaw_ = 0;
    float refLength_ = 0;
    float targetRefLength_ = refLength_;
    float refRoll_ = 0;
};

} // namespace CHASSIS
