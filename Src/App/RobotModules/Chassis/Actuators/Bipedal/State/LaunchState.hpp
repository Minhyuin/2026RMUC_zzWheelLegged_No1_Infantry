#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

static float launchLegLengthAngPidPD[2]{ 20.f, 0.0f };
static float launchLegLengthVelPidPD[2]{ 55.f, 0.0f };

namespace CHASSIS {
class LaunchState : public FSMState<FSMState_e> {
    static constexpr float LAUNCH_DPHI = 4.f * 0.001f; // per ms
    static constexpr float LAUNCH_L = WheelLegged::MIN_L;
    static constexpr float LAUNCH_PHI_THRESHOLD = 0.1f;
    static constexpr float LAUNCH_L_THRESHOLD = 0.05f;

    static constexpr float LAUNCH_PHI_FORNT = 0.25f;
    static constexpr float LAUNCH_PHI_BACK = -0.1f;

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
        chassis_->setLegLengthAngPidPD(launchLegLengthAngPidPD[0], launchLegLengthAngPidPD[1]);
        chassis_->setLegLengthVelPidPD(launchLegLengthVelPidPD[0], launchLegLengthVelPidPD[1]);

        // 腿在前面则直接旋转+缩腿，在后面则先缩腿再旋转
        if (refPhiL_ > 0) {
            legPos_[0] = LegPos_e::BACK;
            launchPhi_ = LAUNCH_PHI_BACK;
        } else if (refPhiL_ < -1.2f) {
            legPos_[0] = LegPos_e::FRONT;
        } else {
            legPos_[0] = LegPos_e::BOTTOM;
        }
        if (refPhiR_ > 0) {
            legPos_[1] = LegPos_e::BACK;
        } else if (refPhiR_ < -1.2f) {
            legPos_[1] = LegPos_e::FRONT;
        } else {
            legPos_[1] = LegPos_e::BOTTOM;
        }
        if ((chassis_->phiL() + chassis_->phiR()) / 2.f > 0.f) {
            launchPhi_ = LAUNCH_PHI_BACK;
        } else {
            launchPhi_ = LAUNCH_PHI_FORNT;
        }

        LOG::info("ChassisLaunch", "enter");
    }

    void run() final
    {
        chassis_->setLength(LAUNCH_L);
        chassis_->setFdFeedforward(0.f);

        // 缩腿
        if (legPos_[0] == LegPos_e::FRONT)
            // 左腿在前
            isLengthFinish_[0] = true;
        else {
            if (chassis_->detectLengthLRange(LAUNCH_L, LAUNCH_L_THRESHOLD) || isLengthFinish_[0]) {
                isLengthFinish_[0] = true;
            }
        }
        if (legPos_[1] == LegPos_e::FRONT)
            // 右腿在前
            isLengthFinish_[1] = true;
        else {
            if (chassis_->detectLengthRRange(LAUNCH_L, LAUNCH_L_THRESHOLD) || isLengthFinish_[1]) {
                isLengthFinish_[1] = true;
            }
        }

        // 定方向旋到指定角度再复位
        if (isLengthFinish_[0]) {
            if (chassis_->detectPhiLRange(launchPhi_ + LAUNCH_PHI_THRESHOLD, launchPhi_ - LAUNCH_PHI_THRESHOLD) ||
                isRotFinish_[0]) {
                isRotFinish_[0] = true;
                refPhiL_ = chassis_->phiL();
            } else {
                if (legPos_[0] == LegPos_e::BOTTOM)
                    refPhiL_ += LAUNCH_DPHI;
                else
                    refPhiL_ -= LAUNCH_DPHI;
            }
        }
        chassis_->setHipPhiL(refPhiL_);

        if (isLengthFinish_[1]) {
            if (chassis_->detectPhiRRange(launchPhi_ + LAUNCH_PHI_THRESHOLD, launchPhi_ - LAUNCH_PHI_THRESHOLD) ||
                isRotFinish_[1]) {
                isRotFinish_[1] = true;
                refPhiR_ = chassis_->phiR();
            } else {
                if (legPos_[1] == LegPos_e::BOTTOM)
                    refPhiR_ += LAUNCH_DPHI;
                else
                    refPhiR_ -= LAUNCH_DPHI;
            }
        }
        chassis_->setHipPhiR(refPhiR_);

        chassis_->noBal(chassis_->msg.vx == 0.f ? chassis_->wheelVel() : chassis_->msg.vx, 0);

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
    float launchPhi_ = 0;
    bool isRotFinish_[2] = { false, false };
    bool isLengthFinish_[2] = { false, false };
    enum class LegPos_e : uint8_t { FRONT, BOTTOM, BACK } legPos_[2]; // true则腿在前方
    bool isOverturn_ = false;
};

} // namespace CHASSIS
