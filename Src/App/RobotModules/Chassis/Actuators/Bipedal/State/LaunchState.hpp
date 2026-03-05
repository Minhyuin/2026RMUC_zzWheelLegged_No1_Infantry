#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"
namespace CHASSIS {
class LaunchState : public FSMState<FSMState_e> {
    static constexpr float LAUNCH_DPHI = 4.f * 0.001f; // per ms
    static constexpr float LAUNCH_L = Bipedal::DEFAULT_L;

    static constexpr float LAUNCH_ROT_TO_PHI1 = 0.01f;
    static constexpr float LAUNCH_ROT_TO_PHI2 = -0.1f;

public:
    LaunchState(Bipedal *_chassis) : FSMState<FSMState_e>(FSMState_e::LAUNCH), chassis_(_chassis) {}

    void enter() final
    {
        chassis_->enter();
        isRotFinish_[0] = isRotFinish_[1] = false;
        isLengthFinish_[0] = isLengthFinish_[1] = false;
        isOverturn_ = chassis_->isOverturn();
        chassis_->disableRoll();
        refPhiL_ = chassis_->phiL();
        refPhiR_ = chassis_->phiR();

        // 腿在前面则直接旋转+缩腿，在后面则先缩腿再旋转
        if (refPhiL_ < 0)
            legInFront_[0] = true;
        else
            legInFront_[0] = false;
        if (refPhiR_ < 0)
            legInFront_[1] = true;
        else
            legInFront_[1] = false;

        LOG::info("ChassisLaunch", "enter");
    }

    void run() final
    {
        chassis_->setLength(LAUNCH_L);

        // 缩腿
        if (legInFront_[0])
            isLengthFinish_[0] = true;
        else {
            if (chassis_->detectLengthLRange(LAUNCH_L, 0.015f) || isLengthFinish_[0]) {
                isLengthFinish_[0] = true;
            }
        }
        if (legInFront_[1])
            isLengthFinish_[1] = true;
        else {
            if (chassis_->detectLengthRRange(LAUNCH_L, 0.015f) || isLengthFinish_[1]) {
                isLengthFinish_[1] = true;
            }
        }

        // 定方向旋到指定角度再复位
        if (isLengthFinish_[0]) {
            if (chassis_->detectPhiLRange(LAUNCH_ROT_TO_PHI1, LAUNCH_ROT_TO_PHI2) || isRotFinish_[0]) {
                isRotFinish_[0] = true;
                refPhiL_ = chassis_->phiL();
            } else {
                refPhiL_ -= LAUNCH_DPHI;
            }
        }
        chassis_->setHipPhiL(refPhiL_);

        if (isLengthFinish_[1]) {
            if (chassis_->detectPhiRRange(LAUNCH_ROT_TO_PHI1, LAUNCH_ROT_TO_PHI2) || isRotFinish_[1]) {
                isRotFinish_[1] = true;
                refPhiR_ = chassis_->phiR();
            } else {
                refPhiR_ -= LAUNCH_DPHI;
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
        else if (isOverturn_)
            return FSMState_e::RESCUE;
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
    bool isLengthFinish_[2] = { false, false };
    bool legInFront_[2] = { false, false }; // true则腿在前方
    bool isOverturn_ = false;
};

} // namespace CHASSIS
