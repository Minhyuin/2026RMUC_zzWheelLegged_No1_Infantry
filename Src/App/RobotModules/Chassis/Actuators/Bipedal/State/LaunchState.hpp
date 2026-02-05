#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"
namespace CHASSIS {
class LaunchState : public FSMState<FSMState_e> {
    static constexpr float LAUNCH_DPHI = 5.f * 0.001f; // per ms
    static constexpr float LAUNCH_L = 0.2f;

    static constexpr float LAUNCH_ROT_TO_PHI1 = 0.1f;
    static constexpr float LAUNCH_ROT_TO_PHI2 = -0.001f;

public:
    LaunchState(Bipedal *_chassis) : FSMState<FSMState_e>(FSMState_e::LAUNCH), chassis_(_chassis) {}

    void enter() final
    {
        isRotFinish_[0] = isRotFinish_[1] = false;
        chassis_->enter();
        refPhiL_ = chassis_->phiL();
        refPhiR_ = chassis_->phiR();
        LOG::info("ChassisLaunch", "enter");
    }

    void run() final
    {
        chassis_->setLength(LAUNCH_L);

        // 定方向旋到指定角度再复位
        if (chassis_->detectPhiLRange(LAUNCH_ROT_TO_PHI1, LAUNCH_ROT_TO_PHI2)) {
            isRotFinish_[0] = true;
        } else {
            if (chassis_->phiL() > 0) {
                refPhiL_ -= LAUNCH_DPHI;
            } else {
                refPhiL_ += LAUNCH_DPHI;
            }
        }
        chassis_->setHipPhiL(refPhiL_);

        if (chassis_->detectPhiRRange(LAUNCH_ROT_TO_PHI1, LAUNCH_ROT_TO_PHI2)) {
            isRotFinish_[1] = true;
        } else {
            if (chassis_->phiR() > 0) {
                refPhiR_ -= LAUNCH_DPHI;
            } else {
                refPhiR_ += LAUNCH_DPHI;
            }
        }
        chassis_->setHipPhiR(refPhiR_);

        chassis_->noBal(chassis_->msg.vx, 0);

        chassis_->ctrl();
    }

    void exit() final { LOG::info("ChassisLaunch", " exit"); }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (isRotFinish_[0] && isRotFinish_[1])
            return FSMState_e::STAND;
        // TODO: the other state
        else
            return FSMState_e::LAUNCH;
    }

private:
    Bipedal *chassis_;
    float refPhiL_ = 0;
    float refPhiR_ = 0;
    bool isRotFinish_[2] = { false, false };
};

} // namespace CHASSIS
