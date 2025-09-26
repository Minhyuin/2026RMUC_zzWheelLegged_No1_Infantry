#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "PidBasic.hpp"

namespace CHASSIS {
class RunState : public FSMState<FSMState_e> {
    static constexpr float DEFAULT_L = 0.25586f;

public:
    RunState(Bipedal *_chassis)
            : FSMState<FSMState_e>(FSMState_e::RUN), chassis_(_chassis)
    {
    }

    void enter() final
    {
        chassis_->enter();
        refYaw_ = chassis_->yaw();
        refLength_ = DEFAULT_L;
        LOG::info("ChassisRun", " enter");
    }

    void run() final
    {
        refLength_ += chassis_->msg.dl * 0.001f;
        chassis_->setLength(refLength_);

        // WL
        // WLController::States_u ref{};
        // ref.ds = chassis_->msg.vx;
        // chassis_->bal(ref, minorArc(chassis_->msg.yaw,
        //                                           chassis_->yaw()) /
        //                                0.001f);

        // WBR
        WBRController::States_u ref{};
        refYaw_ = chassis_->isFloating() ?
                          chassis_->yaw() :
                          (PINYMOTOR::rangeMap(
                                  refYaw_ + (chassis_->msg.wz * 0.001f), -PI,
                                  PI));
        ref.ds = chassis_->msg.vx;
        ref.yaw = refYaw_;
        ref.dyaw = chassis_->msg.wz;
        chassis_->bal(ref);

        // TestRun
        // chassis_->setHipPhiSmooth((PI / 4.f) - 0.25f, 0.05f);
        // chassis_->noBal(chassis_->msg.vx, chassis_->msg.wz);

        chassis_->ctrl();
    }

    void exit() final
    {
        wzAngPid_.reset();
        LOG::info("ChassisRun", " exit");
    }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP || chassis_->isTumble())
            return FSMState_e::STOP;
        else if (chassis_->msg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        // TODO: the other state
        else
            return FSMState_e::RUN;
    }

private:
    Bipedal *chassis_;
    PositionalPid wzAngPid_{ 0, 0, 0, 0.001f, 0, 0, 0 };
    float refYaw_ = 0;
    float refLength_ = DEFAULT_L;
};

} // namespace CHASSIS
