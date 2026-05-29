#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#if EXTENSION_BUZZER
#include "Buzzer.hpp"
#endif

namespace CHASSIS {

class StopState : public FSMState<FSMState_e> {
public:
    StopState(Bipedal *_chassis) : FSMState<FSMState_e>(FSMState_e::STOP), chassis_(_chassis) {}

    void enter() final
    {
        chassis_->stop();
        if (chassis_->msg.noBalance)
            chassis_->enableWheels();

        chassis_->setFdFeedforward(0, 0);
        freezeFlag_ = false;
        LOG::info("ChassisStop", " enter");

#if EXTENSION_BUZZER
        BUZZER::buzz->playNote(BUZZER::Tone_e::FS5, 50);
        BUZZER::buzz->playNote(BUZZER::Tone_e::C5, 50);
#endif
    }

    void run() final
    {
        if (!freezeFlag_)
            freezeFlag_ = chassis_->isFreeze();

        if (!noBalancePrev_ && chassis_->msg.noBalance) {
            chassis_->enableWheels();
            noBalancePrev_ = chassis_->msg.noBalance;
            return;
        }
        noBalancePrev_ = chassis_->msg.noBalance;

        if (chassis_->msg.noBalance) {
            chassis_->noBal(chassis_->msg.vx, chassis_->msg.vy);
            chassis_->ctrl();
        } else
            chassis_->stop();
    }

    void exit() final { LOG::info("ChassisStop", " exit"); }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (chassis_->msg.state != FSMState_e::STOP) {
            chassis_->needGimbalLaunch(true); // wait for gimbal launch
            chassis_->allowGimbalExitLaunch(false);
            if (freezeFlag_) {
                if (chassis_->isOverturn())
                    return FSMState_e::RESCUE;
                if (chassis_->isGimbalReset()) {
                    return FSMState_e::LAUNCH;
                }
            }
            return FSMState_e::STOP;
        }
        // if (chassis_->msg.state != FSMState_e::STOP)
        //     return FSMState_e::IDLE;
        // return FSMState_e::TEST;
        else
            return FSMState_e::STOP;
    }

private:
    Bipedal *chassis_;
    bool freezeFlag_ = false;

    bool noBalancePrev_;
};

} // namespace CHASSIS
