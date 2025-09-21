#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "PidBasic.hpp"

namespace CHASSIS {
class TestState : public FSMState<FSMState_e> {
public:
    TestState(Bipedal *_chassis)
            : FSMState<FSMState_e>(FSMState_e::TEST), chassis_(_chassis)
    {
    }

    void enter() final
    {
        chassis_->enter();
        LOG::info("ChassisTest", " enter");
        phi_ = PI;
    }

    void run() final
    {
        phi_ += chassis_->msg.vx / 500.f;
        phi_ = PINYMOTOR::rangeMap(phi_);
        chassis_->setFdForward(chassis_->msg.dl);
        chassis_->ctrlHip(phi_, phi_);
    }

    void exit() final { LOG::info("ChassisTest", " exit"); }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (chassis_->msg.state == FSMState_e::RUN)
            return FSMState_e::TEST;
        // TODO: the other state
        else
            return FSMState_e::TEST;
    }

private:
    Bipedal *chassis_;
    float phi_;
};

} // namespace CHASSIS
