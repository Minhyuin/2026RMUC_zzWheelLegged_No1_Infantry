#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "Smooth.hpp"

#if EXTENSION_BUZZER
#include "Buzzer.hpp"
#endif

static PositionalPid turnFollowVelPid{ 8.0f, 0.f, 0.1f, 0.001f, 0.f, 3.f, 0.1f };

namespace CHASSIS {
class TurnState : public FSMState<FSMState_e> {
public:
    TurnState(Bipedal *_chassis)
            : FSMState<FSMState_e>(FSMState_e::TURN), chassis_(_chassis), wzAngPid_(&turnFollowVelPid)
    {
    }

    void enter() final
    {
        chassis_->enter();

        const float enterTargetDeltaYaw = (fabs(chassis_->deltaYaw) > PI / 2.f) ? PI : 0.f;
        targetDeltaYaw_ = enterTargetDeltaYaw == PI ? 0 : PI;

        LOG::info("ChassisTurn", " enter");

#if EXTENSION_BUZZER
        BUZZER::buzz->playNote(BUZZER::Tone_e::FS5, 50);
        BUZZER::buzz->playNote(BUZZER::Tone_e::C5, 50);
#endif
    }

    void run() final
    {
        const auto &rotationAcc = chassis_->getRotationAcc();
        float wz = std::clamp(wzAngPid_->calc(PINYMOTOR::getMinorArc(chassis_->deltaYaw, targetDeltaYaw_), 0),
                              -rotationAcc, rotationAcc);

        WBRController::States_s ref{};
        refYaw_ = chassis_->isFloating() ? chassis_->yaw() : (PINYMOTOR::rangeMap(chassis_->yaw() + wz, -PI, PI));
        ref.yaw = refYaw_;
        chassis_->bal<WBRController::WBRControlMode_e::BALANCE>(ref);

        chassis_->ctrl();
    }

    void exit() final
    {
        wzAngPid_->reset();
        LOG::info("ChassisTurn", " exit");
    }

    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP || chassis_->isTumble())
            return FSMState_e::STOP;
        else if (std::fabs(PINYMOTOR::getMinorArc(chassis_->deltaYaw, targetDeltaYaw_)) < 0.1f)
            return FSMState_e::IDLE;
        else
            return FSMState_e::TURN;
    }

    bool change() final
    {
        WBRController::States_s ref{};
        ref.ds = SMOOTH::slopeFollowing(0.f, chassis_->vel(), 1.f);
        ref.yaw = chassis_->yaw();
        chassis_->bal<WBRController::WBRControlMode_e::BALANCE>(ref);
        chassis_->ctrl();
        return std::fabs(chassis_->vel()) < 1.5f;
    }

private:
    Bipedal *chassis_;
    PositionalPid *wzAngPid_;

    float refYaw_ = 0;
    float targetDeltaYaw_ = 0;
};
}