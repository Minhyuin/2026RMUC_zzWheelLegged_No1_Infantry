// TODO: merge IdleSate
#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "PidBasic.hpp"

namespace CHASSIS {
class ClimbPrepareState : public FSMState<FSMState_e> {
    static constexpr float OFFSET_Q = 0.f;
    static constexpr float MAX_ACC_BACK = 0.4f;
    static constexpr float MAX_ACC_FRONT = 2.f;

public:
    ClimbPrepareState(Bipedal *_chassis) : FSMState<FSMState_e>(FSMState_e::CLIMB_PREPARE), chassis_(_chassis) {}

    void enter() final
    {
        chassis_->enter();
        refYaw_ = chassis_->yaw();
        chassis_->setRoll(0);
        refLength_ = (chassis_->lengthL() + chassis_->lengthR()) / 2.f;
        LOG::info("ChassisClimbPrepare", " enter");
    }

    void run() final
    {
        refLength_ += chassis_->msg.dl * 0.001f;
        chassis_->setLength(refLength_);
        chassis_->setFdFeedforward();

        // WBR
        WBRController::States_u ref{};
        float wz = chassis_->msg.wz * 0.001f;
        refYaw_ = chassis_->isFloating() ? chassis_->yaw() : (PINYMOTOR::rangeMap(refYaw_ + wz, -PI, PI));
        ref.ds = std::clamp(chassis_->msg.vx, chassis_->vel() - MAX_ACC_BACK, chassis_->vel() + MAX_ACC_FRONT);
        ref.yaw = refYaw_;
        ref.dyaw = wz;
        ref.q = OFFSET_Q;
        chassis_->bal(ref);

        chassis_->ctrl();
    }

    void exit() final
    {
        wzAngPid_.reset();
        LOG::info("ChassisClimbPrepare", " exit");
    }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        if (chassis_->isFloating())
            return FSMState_e::CLIMB;
        else if (chassis_->msg.state == FSMState_e::IDLE)
            return FSMState_e::IDLE;
        else if (chassis_->msg.state == FSMState_e::SPIN)
            return FSMState_e::SPIN;
        else
            return FSMState_e::CLIMB_PREPARE;
    }

private:
    Bipedal *chassis_;
    PositionalPid wzAngPid_{ 0, 0, 0, 0.001f, 0, 0, 0 };
    float refYaw_ = 0;
    float refLength_ = Bipedal::DEFAULT_L;
};

} // namespace CHASSIS
