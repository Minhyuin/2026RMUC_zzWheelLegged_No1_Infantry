#pragma once

#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#if EXTENSION_BUZZER
#include "Buzzer.hpp"
#endif

namespace CHASSIS {

class RescueState : public FSMState<FSMState_e> {
    static constexpr float RESCUE_L = 0.35f;
    static constexpr float RESCUE_DPHI = 4.f * 0.001f;

    static constexpr float SYNC_PHI = -PI / 2.f;
    static constexpr float RESTORE_PHI = PI / 2.f;

    static constexpr uint16_t MAX_RESCUE_COUNT = 350;

public:
    RescueState(Bipedal *_chassis) : FSMState<FSMState_e>(FSMState_e::RESCUE), chassis_(_chassis) {}

    void enter() final
    {
        chassis_->enter();
        isSyncFinish_[0] = isSyncFinish_[1] = false;
        isRotFinish_[0] = isRotFinish_[1] = false;
        rescueFinishCount_ = 0;
        rescueFinishFlag_ = false;
        freezeFlag_ = false;
        chassis_->disableRoll();
        refPhiL_ = chassis_->phiL();
        refPhiR_ = chassis_->phiR();
        LOG::info("ChassisRescue", "enter");

#if EXTENSION_BUZZER
        BUZZER::buzz->playNote(BUZZER::Tone_e::G4, 50);
        BUZZER::buzz->playNote(BUZZER::Tone_e::C5, 50);
        BUZZER::buzz->playNote(BUZZER::Tone_e::E5, 100);
#endif
    }

    void run() final
    {
        if (!freezeFlag_ && !chassis_->isOverturn())
            freezeFlag_ = chassis_->isFreeze();

        chassis_->setLength(RESCUE_L);

        // 先同步摆到合适的位置
        if (!isSyncFinish_[0]) {
            if (chassis_->detectPhiLRange(SYNC_PHI + 0.1f, SYNC_PHI - 0.1f)) {
                isSyncFinish_[0] = true;
                refPhiL_ = chassis_->phiL();
            } else {
                refPhiL_ -= RESCUE_DPHI;
            }
        } else {
            if (!isSyncFinish_[1])
                refPhiL_ = chassis_->phiL();
        }
        if (!isSyncFinish_[1]) {
            if (chassis_->detectPhiRRange(SYNC_PHI + 0.1f, SYNC_PHI - 0.1f)) {
                isSyncFinish_[1] = true;
                refPhiR_ = chassis_->phiR();
            } else {
                refPhiR_ -= RESCUE_DPHI;
            }
        } else {
            if (!isSyncFinish_[0])
                refPhiR_ = chassis_->phiR();
        }

        // 一起摆到目标位置以翻转机体
        if (isSyncFinish_[0] && isSyncFinish_[1]) {
            isRotFinish_[0] = isRotFinish_[0] | chassis_->detectPhiLRange(RESTORE_PHI + 0.1f, RESTORE_PHI - 0.1f);
            isRotFinish_[1] = isRotFinish_[1] | chassis_->detectPhiRRange(RESTORE_PHI + 0.1f, RESTORE_PHI - 0.1f);
            if (isRotFinish_[0]) {
                refPhiL_ = chassis_->phiL();
            } else {
                refPhiL_ -= RESCUE_DPHI;
            }
            if (isRotFinish_[1]) {
                refPhiR_ = chassis_->phiR();
            } else {
                refPhiR_ -= RESCUE_DPHI;
            }
        }

        if (!chassis_->isOverturn()) {
            isRotFinish_[0] = isRotFinish_[1] = isSyncFinish_[0] = isSyncFinish_[1] = true;
        }
        rescueFinishCount_ += (isRotFinish_[0] && isRotFinish_[1]) ? 1 : 0;

        chassis_->setHipPhi(refPhiL_, refPhiR_);
        chassis_->noBal(chassis_->msg.vx, 0);

        chassis_->ctrl();
    }

    void exit() final { LOG::info("ChassisRescue", "exit"); }

    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (rescueFinishCount_ > MAX_RESCUE_COUNT && freezeFlag_)
            return FSMState_e::LAUNCH;
        // TODO: the other state
        else
            return FSMState_e::RESCUE;
    }

private:
    Bipedal *chassis_;
    uint16_t rescueFinishCount_ = 0;
    float refPhiL_ = 0;
    float refPhiR_ = 0;
    bool rescueFinishFlag_ = false;
    bool freezeFlag_ = false;
    bool isSyncFinish_[2] = { false, false };
    bool isRotFinish_[2] = { false, false };
};

} // namespace CHASSIS
