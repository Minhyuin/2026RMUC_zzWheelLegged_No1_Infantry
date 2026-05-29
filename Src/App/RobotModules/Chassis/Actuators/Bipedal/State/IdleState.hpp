#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "PidBasic.hpp"
#include "Smooth.hpp"

#if EXTENSION_BUZZER
#include "Buzzer.hpp"
#endif

static float idleFollowVelPidPID[7]{ 7.0f, 1.f, 0.1f, 0.001f, 0.05f, 2.f, 0.05f };
static float idleFollowVelInVisionPidPID[7]{ 5.0f, 1.f, 0.1f, 0.001f, 0.05f, 1.f, 0.5f };
static PositionalPid idleFollowVelPid{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f };
volatile static float idleLegLengthAngPidPD[3]{ 15.f, 0.5f, 1.f };
volatile static float idleLegLengthVelPidPD[2]{ 125.f, 0.0f };

namespace CHASSIS {
class IdleState : public FSMState<FSMState_e> {
    static constexpr float OFFSET_Q = 0.f;
    static constexpr float IDLE_L = WheelLegged::MIN_L;

    static constexpr float PITCH_GAIN = 0.4f;

public:
    IdleState(Bipedal *_chassis)
            : FSMState<FSMState_e>(FSMState_e::IDLE), chassis_(_chassis), wzAngPid_(&idleFollowVelPid)
    {
        setParamFromArray(wzAngPid_, idleFollowVelPidPID);
    }

    void enter() final
    {
        chassis_->enter();

        chassis_->enableRoll();
        refYaw_ = chassis_->yaw();
        refRoll_ = 0.f;
        refLength_ = chassis_->length();
        targetRefLength_ = IDLE_L;
        isJumpReady_ = false;
        needJumpPrev_ = chassis_->msg.needJump;
        needStoolPrev_ = chassis_->msg.needStool;
        strechPrev_ = chassis_->msg.needStretch;
        chassis_->setLegLengthAngPidPD(idleLegLengthAngPidPD[0], idleLegLengthAngPidPD[1], idleLegLengthAngPidPD[2]);
        chassis_->setLegLengthVelPidPD(idleLegLengthVelPidPD[0], idleLegLengthVelPidPD[1]);
        chassis_->needGimbalLaunch(false);
        chassis_->allowGimbalExitLaunch(true);

        LOG::info("ChassisIdle", " enter");

#if EXTENSION_BUZZER
        BUZZER::buzz->playNote(BUZZER::Tone_e::FS5, 50);
        BUZZER::buzz->playNote(BUZZER::Tone_e::C5, 50);
#endif
    }

    void run() final
    {
        if (!strechPrev_ && chassis_->msg.needStretch) {
            targetRefLength_ = Bipedal::STRETCH_L;
        } else if (strechPrev_ && !chassis_->msg.needStretch) {
            targetRefLength_ = IDLE_L;
            refRoll_ = 0.f;
        } else
            targetRefLength_ += chassis_->msg.dl * 0.001f;
        strechPrev_ = chassis_->msg.needStretch;
        bool strechDir = (targetRefLength_ - refLength_) > 0;
        refLength_ = SMOOTH::slopeFollowing(targetRefLength_, refLength_, strechDir ? 999.f : 0.00025f);
        chassis_->setLength(refLength_);
        chassis_->setFdFeedforward();
        refRoll_ += chassis_->msg.droll * 0.001f;
        chassis_->setRoll(refRoll_);

        if (chassis_->gimbalState() == GIMBAL::FSMState_e::AUTO)
            setParamFromArray(wzAngPid_, idleFollowVelInVisionPidPID);
        else
            setParamFromArray(wzAngPid_, idleFollowVelPidPID);

        float targetDeltaYaw = (fabs(chassis_->deltaYaw) > PI / 2.f) ? PI : 0.f;
        const auto &rotationAcc = chassis_->getRotationAcc();
        float wz = std::clamp(wzAngPid_->calc(PINYMOTOR::getMinorArc(chassis_->deltaYaw, targetDeltaYaw), 0),
                              -rotationAcc, rotationAcc);
        // float wz = 0;
        float ds = ((chassis_->msg.vx * chassis_->cosYaw) - (chassis_->msg.vy * chassis_->sinYaw)) *
                   (std::max(1.f - (std::fabs(chassis_->wz()) / rotationAcc), 0.4f));
        float deltaDs = powf((ds - chassis_->vel()) /
                                     (chassis_->getTranslationMaxFrontVel() + chassis_->getTranslationMaxBackVel()),
                             9);
        refPitch_ = std::clamp(deltaDs * PITCH_GAIN, -0.15f, 0.10f);

        if (chassis_->msg.needStool != needStoolPrev_) {
            if (chassis_->msg.needStool) {
#if EXTENSION_BUZZER
                BUZZER::buzz->playNote(BUZZER::Tone_e::G5, 150);
                BUZZER::buzz->playNote(BUZZER::Tone_e::C6, 300);
#endif
            } else {
                chassis_->resetPhiBalancePid();
#if EXTENSION_BUZZER
                BUZZER::buzz->playNote(BUZZER::Tone_e::C6, 150);
                BUZZER::buzz->playNote(BUZZER::Tone_e::G5, 300);
#endif
            }
        }
        // ST
        if (chassis_->msg.needStool) {
            STController::States_s ref{};
            refYaw_ = chassis_->isFloating() ? chassis_->yaw() : (PINYMOTOR::rangeMap(chassis_->yaw() + wz, -PI, PI));
            ref.ds = std::clamp(ds, chassis_->vel() - chassis_->getTranslationBackAcc(),
                                chassis_->vel() + chassis_->getTranslationFrontAcc());
            ref.q = OFFSET_Q;
            chassis_->bal<STController::STControlMode_e::BALANCE>(ref, refYaw_);
        }
        // WBR
        else {
            WBRController::States_s ref{};
            refYaw_ = chassis_->isFloating() ? chassis_->yaw() : (PINYMOTOR::rangeMap(chassis_->yaw() + wz, -PI, PI));
            ref.ds = std::clamp(ds, chassis_->vel() - chassis_->getTranslationBackAcc(),
                                chassis_->vel() + chassis_->getTranslationFrontAcc());
            ref.yaw = refYaw_;
            ref.q = OFFSET_Q + refPitch_;
            chassis_->bal<WBRController::WBRControlMode_e::BALANCE>(ref);
        }

        chassis_->ctrl();

        isJumpReady_ = needJumpPrev_ && !chassis_->msg.needJump;
        needJumpPrev_ = chassis_->msg.needJump;

        needStoolPrev_ = chassis_->msg.needStool;
    }

    void exit() final
    {
        wzAngPid_->reset();
        LOG::info("ChassisIdle", " exit");
    }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP || chassis_->isTumble())
            return FSMState_e::STOP;
        if (isJumpReady_)
            return FSMState_e::JUMP;
        else if (chassis_->msg.state == FSMState_e::TANK)
            return FSMState_e::TANK;
        else if (chassis_->msg.state == FSMState_e::SPIN)
            return FSMState_e::SPIN;
        else if (chassis_->msg.state == FSMState_e::TEST)
            return FSMState_e::TEST;
        else if (chassis_->msg.needClimb)
            return FSMState_e::CLIMB_PREPARE;
        else
            return FSMState_e::IDLE;
    }

private:
    Bipedal *chassis_;
    PositionalPid *wzAngPid_;

    static void setParamFromArray(PositionalPid *_pid, const float *_params)
    {
        _pid->setParam(_params[0], _params[1], _params[2], _params[3], _params[4], _params[5], _params[6]);
    }

    float refYaw_ = 0;
    float refLength_ = 0;
    float targetRefLength_ = IDLE_L;
    float refRoll_ = 0;
    float refPitch_ = 0;

    bool isJumpReady_ = false;
    bool needJumpPrev_ = false;

    bool needStoolPrev_ = false;

    bool strechPrev_ = false;
};

} // namespace CHASSIS
