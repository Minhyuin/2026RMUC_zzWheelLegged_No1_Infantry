#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"
namespace CHASSIS {
class LaunchState : public FSMState<FSMState_e> {
    static constexpr float LAUNCH_DPHI = -0.002f; // per ms
    static constexpr float LAUNCH_L = 0.25586f;

    static constexpr float LAUNCH_ROT_TO_PHI1 = (PI / 4.f) - 0.25f;
    static constexpr float LAUNCH_ROT_TO_PHI2 = (PI / 4.f) - 0.4f;

    static constexpr float PRE_RUN_DPHI = 0.1f;
    static constexpr float PRE_RUN_PHI = LAUNCH_ROT_TO_PHI1;
    static constexpr float PRE_RUN_L = 0.25586f;

    // static constexpr float L_THRESHOLD = 0.05f;
    // static constexpr float PHI_THRESHOLD = 0.1f;

public:
    LaunchState(Bipedal *_chassis)
            : FSMState<FSMState_e>(FSMState_e::LAUNCH), chassis_(_chassis)
    {
    }

    void enter() final
    {
        isLaunchFinish_ = false;
        stop_ = false;
        chassis_->enter();
        LOG::info("ChassisLaunch", "enter");
    }

    void run() final
    {
        chassis_->setLength(LAUNCH_L);
        bool rotFinish[2] = { false, false };
        if (chassis_->isOverturn()) {
            // chassis_->setHipPhi(LAUNCH_DPHI);
        } else {
            // 定方向旋到指定角度再复位
            if (chassis_->detectPhiL(LAUNCH_ROT_TO_PHI1, LAUNCH_ROT_TO_PHI2)) {
                chassis_->setHipPhiSmoothL(PRE_RUN_PHI, PRE_RUN_DPHI);
                rotFinish[0] = true;
            } else {
                chassis_->setHipPhiRotL(LAUNCH_DPHI);
                rotFinish[0] = false;
            }
            if (chassis_->detectPhiR(LAUNCH_ROT_TO_PHI1, LAUNCH_ROT_TO_PHI2)) {
                chassis_->setHipPhiSmoothR(PRE_RUN_PHI, PRE_RUN_DPHI);
                rotFinish[1] = true;
            } else {
                chassis_->setHipPhiRotR(LAUNCH_DPHI);
                rotFinish[1] = false;
            }

            // 若都在范围内即可运作
            if (rotFinish[0] && rotFinish[1]) {
                isLaunchFinish_ = true;
            }
        }
        chassis_->ctrl();
        // isLaunchFinish_ = true; // debug
    }

    void exit() final { LOG::info("ChassisLaunch", " exit"); }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (isLaunchFinish_)
            return FSMState_e::RUN;
        // TODO: the other state
        else
            return FSMState_e::LAUNCH;
    }

private:
    Bipedal *chassis_;
    bool isLaunchFinish_ = false;
    bool stop_ = false;
};

} // namespace CHASSIS
