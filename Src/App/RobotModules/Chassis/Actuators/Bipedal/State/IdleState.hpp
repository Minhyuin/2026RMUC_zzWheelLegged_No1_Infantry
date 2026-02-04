#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "PidBasic.hpp"
namespace CHASSIS {
class IdleState : public FSMState<FSMState_e> {
    static constexpr float OFFSET_Q = 0.f;
    static constexpr float DEFAULT_L = 0.2f;
    static constexpr float MAX_ACC_BACK = 0.4f;
    static constexpr float MAX_ACC_FRONT = 2.f;

public:
    IdleState(Bipedal *_chassis) : FSMState<FSMState_e>(FSMState_e::IDLE), chassis_(_chassis) {}

    void enter() final
    {
        chassis_->enter();
        refYaw_ = chassis_->yaw();
        refRoll_ = chassis_->roll();
        refLength_ = (chassis_->lengthL() + chassis_->lengthR()) / 2.f;
        LOG::info("ChassisIdle", " enter");
    }

    void run() final
    {
        refLength_ += chassis_->msg.dl * 0.001f / 5.f;
        chassis_->setLength(refLength_);
        chassis_->setFdForward();
        refRoll_ += chassis_->msg.droll * 0.001f;
        chassis_->setRoll(refRoll_);

        // WBR
        WBRController::States_u ref{};
        refYaw_ = chassis_->isFloating() ? chassis_->yaw() :
                                           (PINYMOTOR::rangeMap(refYaw_ + (chassis_->msg.wz * 0.001f), -PI, PI));
        ref.ds = std::clamp(chassis_->msg.vx, chassis_->vel() - MAX_ACC_BACK, chassis_->vel() + MAX_ACC_FRONT);
        ref.yaw = refYaw_;
        ref.dyaw = chassis_->msg.wz / 3.f;
        ref.q = OFFSET_Q;
        chassis_->bal(ref);

        chassis_->ctrl();
    }

    void exit() final
    {
        wzAngPid_.reset();
        LOG::info("ChassisIdle", " exit");
    }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP || chassis_->isTumble())
            return FSMState_e::STOP;
        else if (chassis_->msg.state == FSMState_e::IDLE)
            return FSMState_e::IDLE;
        // TODO: the other state
        else
            return FSMState_e::IDLE;
    }

private:
    Bipedal *chassis_;
    PositionalPid wzAngPid_{ 0, 0, 0, 0.001f, 0, 0, 0 };
    float refYaw_ = 0;
    float refLength_ = DEFAULT_L;
    float refRoll_ = 0;
};

} // namespace CHASSIS
