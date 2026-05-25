#pragma once

#include "Standard.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#if EXTENSION_BUZZER
#include "Buzzer.hpp"
#endif

namespace GIMBAL {

class StopState : public FSMState<FSMState_e> {
public:
    StopState(Standard *_gimbal) : FSMState<FSMState_e>(FSMState_e::STOP), gimbal_(_gimbal) {}

    void enter() final
    {
        gimbal_->loadResetState(false);

        LOG::info("GimbalStop", "enter");

#if EXTENSION_BUZZER
        BUZZER::buzz->playNote(BUZZER::Tone_e::FS7, 50);
        BUZZER::buzz->playNote(BUZZER::Tone_e::C7, 50);
#endif
    }

    void run() final { gimbal_->stop(); }

    void exit() final { LOG::info("GimbalStop", "exit"); }

    FSMState_e checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (gimbal_->msg.state == FSMState_e::RUN || gimbal_->msg.state == FSMState_e::LAUNCH)
            return FSMState_e::LAUNCH;
        else
            return FSMState_e::STOP;
    }

private:
    Standard *gimbal_;
};

} // namespace GIMBAL
