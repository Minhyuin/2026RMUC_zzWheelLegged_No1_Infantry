#pragma once
#include "Dial.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "PidBasic.hpp"
#include "MotorCommonMacros.hpp"

static PositionalPid autoDialVelPID(0.13f, 0.1f, 0.002f, 0.002f, 0.2f, 2.49f);

namespace ARMORLOADER {

class AutoState : public FSMState<FSMState_e> {
    struct Pids_s {
        PID *dialVelPid;
    };

    static constexpr float getDialVel(float _rate)
    {
        return _rate / static_cast<float>(Dial::BULLET_NUM_ROUND) * 2.f * PI;
    }

public:
    AutoState(Dial *_armorLoader) : FSMState<FSMState_e>(FSMState_e::AUTO), loader_(_armorLoader)
    {
        pids_.dialVelPid = &autoDialVelPID;
    }

    void enter() final
    {
        loader_->enter();
        LOG::info("ArmorLoaderAuto", " enter");
    }

    void run() final
    {
        if (loader_->msg.fire)
            loader_->ctrl(pids_.dialVelPid, getDialVel(15.f));
        else
            loader_->ctrl(pids_.dialVelPid, 0.f);
    }

    void exit() final
    {
        pids_.dialVelPid->reset();
        loader_->lastState = FSMState_e::AUTO;
        LOG::info("ArmorLoaderAuto", " exit");
    }


    FSMState_e checkChange() final
    {
        if (loader_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;

        if (loader_->isBlock())
            return FSMState_e::BLOCK;
        else if (loader_->msg.state == FSMState_e::SINGLE)
            return FSMState_e::SINGLE;
        else
            return FSMState_e::AUTO;
    }

private:
    Dial *loader_;

    Pids_s pids_;
};
} // namespace ARMORLOADER
