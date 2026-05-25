#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#if EXTENSION_BUZZER
#include "Buzzer.hpp"
#endif

namespace CHASSIS {
class TestState : public FSMState<FSMState_e> {
public:
    TestState(Bipedal *_chassis) : FSMState<FSMState_e>(FSMState_e::TEST), chassis_(_chassis) {}

    void enter() final
    {
        chassis_->enter();
        LOG::info("ChassisTest", " enter");

#if EXTENSION_BUZZER
        BUZZER::buzz->playNote(BUZZER::Tone_e::E5, 50);
        BUZZER::buzz->playNote(BUZZER::Tone_e::C6, 50);
#endif
    }

    void run() final
    {
        chassis_->noBal(chassis_->msg.vx, -chassis_->msg.wz);
        chassis_->disableHip();
        chassis_->ctrl();
    }

    void exit() final { LOG::info("ChassisTest", " exit"); }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (chassis_->msg.state == FSMState_e::TEST)
            return FSMState_e::TEST;
        else if (chassis_->msg.state == FSMState_e::IDLE)
            return FSMState_e::LAUNCH;
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
