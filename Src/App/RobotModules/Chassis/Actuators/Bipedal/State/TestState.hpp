#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace CHASSIS {
class TestState : public FSMState<FSMState_e> {
public:
    TestState(Bipedal *_chassis) : FSMState<FSMState_e>(FSMState_e::TEST), chassis_(_chassis) {}

    void enter() final
    {
        chassis_->enter();
        LOG::info("ChassisTest", " enter");
        phi_ = chassis_->phiL();
        l0_ = chassis_->lengthL();
    }

    void run() final
    {
        phi_ += chassis_->msg.dphi * 0.002f;
        l0_ += chassis_->msg.dl * 0.001f;
        chassis_->setLength(l0_);
        chassis_->setHipPhi(phi_);
        chassis_->noBal(chassis_->msg.vx, -chassis_->msg.wz);
        chassis_->ctrl();
    }

    void exit() final { LOG::info("ChassisTest", " exit"); }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (chassis_->msg.state == FSMState_e::IDLE)
            return FSMState_e::TEST;
        // TODO: the other state
        else
            return FSMState_e::TEST;
    }

private:
    Bipedal *chassis_;
    float phi_;
    float l0_;
};

} // namespace CHASSIS
