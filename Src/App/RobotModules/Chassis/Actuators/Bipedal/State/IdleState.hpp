#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "PidBasic.hpp"
#include "Smooth.hpp"

static PositionalPid idleFollowVelPid{ 15.f, 0.f, 3.0f, 0.001f, 0.f, 5.f, 0.f };

namespace CHASSIS {
class IdleState : public FSMState<FSMState_e> {
    static constexpr float OFFSET_Q = 0.f;
    static constexpr float MAX_ACC_BACK = 0.7f;
    static constexpr float MAX_ACC_FRONT = 1.2f;
    static constexpr float IDLE_L = 0.2f;

public:
    IdleState(Bipedal *_chassis) : FSMState<FSMState_e>(FSMState_e::IDLE), chassis_(_chassis) {}

    void enter() final
    {
        chassis_->enter();
        chassis_->enableRoll();
        refYaw_ = chassis_->yaw();
        refRoll_ = 0.f;
        refLength_ = chassis_->length();
        wzAngPid_ = &idleFollowVelPid;
        LOG::info("ChassisIdle", " enter");
    }

    void run() final
    {
        // refLength_ += chassis_->msg.dl * 0.001f;
        refLength_ = SMOOTH::slopeFollowing(IDLE_L, refLength_, 0.00015f);
        chassis_->setLength(refLength_);
        chassis_->setFdFeedforward();
        refRoll_ += chassis_->msg.droll * 0.001f;
        chassis_->setRoll(refRoll_);

        float targetYaw = (fabs(chassis_->deltaYaw) > PI / 2.f) ? PI : 0.f;
        float wz = -wzAngPid_->calc(0, PINYMOTOR::getMinorArc(chassis_->deltaYaw, targetYaw)) * 0.001f;
        // float wz = 0;
        float ds = (chassis_->msg.vx * chassis_->cosYaw) - (chassis_->msg.vy * chassis_->sinYaw);

        // WBR
        WBRController::States_s ref{};
        refYaw_ = chassis_->isFloating() ? chassis_->yaw() : (PINYMOTOR::rangeMap(refYaw_ + wz, -PI, PI));
        ref.ds = std::clamp(ds, chassis_->vel() - MAX_ACC_BACK, chassis_->vel() + MAX_ACC_FRONT);
        ref.yaw = refYaw_;
        ref.dyaw = wz;
        ref.q = OFFSET_Q;
        chassis_->bal(ref);

        chassis_->ctrl();
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
        else if (chassis_->msg.state == FSMState_e::IDLE)
            return FSMState_e::IDLE;
        else if (chassis_->msg.state == FSMState_e::SPIN)
            return FSMState_e::SPIN;
        else if (chassis_->msg.state == FSMState_e::CLIMB_PREPARE)
            return FSMState_e::CLIMB_PREPARE;
        else
            return FSMState_e::IDLE;
    }

private:
    Bipedal *chassis_;
    PositionalPid *wzAngPid_;
    float refYaw_ = 0;
    float refLength_ = Bipedal::DEFAULT_L;
    float refRoll_ = 0;
};

} // namespace CHASSIS
