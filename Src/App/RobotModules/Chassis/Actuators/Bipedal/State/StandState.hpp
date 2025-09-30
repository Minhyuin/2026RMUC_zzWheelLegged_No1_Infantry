#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace CHASSIS {

class StandState : public FSMState<FSMState_e> {
public:
    static constexpr float STAND_PHI = 0.f;

public:
    StandState(Bipedal *_chassis)
            : FSMState<FSMState_e>(FSMState_e ::STAND), chassis_(_chassis)
    {
    }

    void enter() final
    {
        isBalance_ = false;
        chassis_->enter();
        startLength_ = (chassis_->lengthL() + chassis_->lengthR()) / 2.f;
        startYaw_ = chassis_->yaw();
        LOG::info("ChassisStand", "enter");
    }

    void run() final
    {
        chassis_->setLength(startLength_);
        chassis_->setHipPhi(0.f);

        STController::States_u ref{};
        float vx = chassis_->msg.vx;
        ref.ds = vx;
        chassis_->bal(ref, startYaw_);

        isBalance_ = chassis_->detectPitch(0.1f, -0.1f);

        chassis_->ctrl();
    }

    void exit() final { LOG::info("ChassisStand", "exit"); }

    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        // else if (isBalance_)
        //     return FSMState_e::IDLE;
        // TODO: the other state
        else
            return FSMState_e::STAND;
    }

private:
    Bipedal *chassis_;
    bool isBalance_ = false;
    float startLength_ = 0;
    float startYaw_ = 0;
};

} // namespace CHASSIS
