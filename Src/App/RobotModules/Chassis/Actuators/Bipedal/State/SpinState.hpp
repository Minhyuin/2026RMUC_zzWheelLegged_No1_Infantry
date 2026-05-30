#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "Smooth.hpp"

#if EXTENSION_BUZZER
#include "Buzzer.hpp"
#endif

static float spinLegLengthAngPidPD[3]{ 40.f, 0.3f, 3.f };
static float spinLegLengthVelPidPD[2]{ 110.f, 0.0f };

namespace CHASSIS {

class SpinState : public FSMState<FSMState_e> {
    static constexpr float SPIN_SPEED_CONST = 11.f;
    static constexpr float SPIN_SPEED_BASE = 6.f; // TODO: follow power
    static constexpr float SPIN_SPEED_VAR = 5.f;
    static constexpr float SPIN_L = 0.17f;
    static constexpr float SPIN_STRETCH_L = 0.25f;

    static constexpr float LOW_POWER_SPEED = 8.f;

    static constexpr float MAX_RUSH_DS = 0.2f;
    static constexpr float MAX_DS = 0.5f;
    static constexpr float DIR_OFFSET_ANG_IN_RUSH = (PI / 2.f) - (PI / 4.f);
    static constexpr float DIR_OFFSET_ANG = (PI / 2.f) - (PI / 10.f);


public:
    SpinState(Bipedal *_chassis) : FSMState<FSMState_e>(FSMState_e::SPIN), chassis_(_chassis) {}

    void enter() final
    {
        chassis_->enter();
        refYaw_ = chassis_->yaw();
        refLength_ = chassis_->length();
        targetRefLength_ = SPIN_L;
        strechPrev_ = chassis_->msg.needStretch;
        refRoll_ = 0.f;
        spinCnt_ = 0;
        chassis_->setLegLengthAngPidPD(spinLegLengthAngPidPD[0], spinLegLengthAngPidPD[1], spinLegLengthAngPidPD[2]);
        chassis_->setLegLengthVelPidPD(spinLegLengthVelPidPD[0], spinLegLengthVelPidPD[1]);

        LOG::info("ChassisSpin", " enter");

#if EXTENSION_BUZZER
        BUZZER::buzz->playNote(BUZZER::Tone_e::FS5, 50);
        BUZZER::buzz->playNote(BUZZER::Tone_e::C5, 50);
#endif
    }

    void run() final
    {
        if (!strechPrev_ && chassis_->msg.needStretch) {
            targetRefLength_ = SPIN_STRETCH_L;
        } else if (strechPrev_ && !chassis_->msg.needStretch) {
            targetRefLength_ = SPIN_L;
        } else
            targetRefLength_ += chassis_->msg.dl * 0.001f;
        strechPrev_ = chassis_->msg.needStretch;

        refLength_ = SMOOTH::slopeFollowing(targetRefLength_, refLength_, 0.00015f);
        chassis_->setLength(refLength_);
        chassis_->setFdFeedforward();
        refRoll_ += chassis_->msg.droll * 0.001f;
        chassis_->setRoll(refRoll_);

        float ds, wz;

        // calculate wz in spin
        arm_sqrt_f32((chassis_->msg.vx * chassis_->msg.vx) + (chassis_->msg.vy * chassis_->msg.vy), &ds);
        if (chassis_->msg.needRush) {
            accelFactor_ = SMOOTH::slopeFollowing(1.f, accelFactor_, 0.0005f);
            ds = std::clamp(ds, -MAX_RUSH_DS, MAX_RUSH_DS);
        } else {
            accelFactor_ = SMOOTH::slopeFollowing(1.f, accelFactor_, 0.001f);
            ds = std::clamp(ds, -MAX_DS, MAX_DS);
        }
        if (ds == 0 && chassis_->detectLengthRange(refLength_))
            wz = getVariableSpinVel() * accelFactor_;
        else
            wz = getConstantSpinVel() * accelFactor_;
        // ds = 0;
        // wz = getConstantSpinVelForDebug() * accelFactor_;
        if (!chassis_->msg.needRush)
            wz = std::clamp(wz, -LOW_POWER_SPEED, LOW_POWER_SPEED);
        float newRefYaw = refYaw_ + (wz * 0.001f);
        refYaw_ = chassis_->isFloating() ? chassis_->yaw() : (PINYMOTOR::rangeMap(newRefYaw, -PI, PI));

        // calculate ds in spin
        const float constDirOffsetAng = chassis_->msg.needRush ? DIR_OFFSET_ANG_IN_RUSH : DIR_OFFSET_ANG;
        float dirOffsetAng = wz > 0 ? -constDirOffsetAng : constDirOffsetAng;
        float moveAng = atan2f(-chassis_->msg.vy, chassis_->msg.vx) + dirOffsetAng;
        float deltaAng = PINYMOTOR::getMinorArc(moveAng, chassis_->deltaYaw);
        if (deltaAng < PI / 2.f && deltaAng > -PI / 2.f) {
            ds = -ds;
        }

        // WBR
        WBRController::States_s ref{};
        ref.ds = ds;
        ref.yaw = refYaw_;
        ref.dyaw = wz;
        chassis_->bal<WBRController::WBRControlMode_e::SPIN>(ref);

        chassis_->ctrl();
    }

    void exit() final
    {
        spinCnt_ = 0;
        accelFactor_ = 0.f;
        spinDir_ = !spinDir_;
        LOG::info("ChassisSpin", " exit");
    }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP || chassis_->isTumble())
            return FSMState_e::STOP;
        else if (chassis_->msg.state != FSMState_e::SPIN)
            if (std::fabs(chassis_->wz()) < 2.f)
                return FSMState_e::IDLE;
            else
                return FSMState_e::SPIN_EXITING;
        else
            return FSMState_e::SPIN;
    }

    bool change() final
    {
        WBRController::States_s ref{};
        ref.yaw = chassis_->yaw();
        chassis_->bal<WBRController::WBRControlMode_e::BALANCE>(ref);
        chassis_->ctrl();
        return std::fabs(chassis_->phiGlobalL()) < 0.3f && std::fabs(chassis_->phiGlobalR()) < 0.3f;
    }

private:
    Bipedal *chassis_;

    float refYaw_ = 0.f;
    float refLength_ = 0.f;
    float targetRefLength_ = SPIN_L;

    bool strechPrev_ = false;

    float refRoll_ = 0;

    float accelFactor_ = 0.f;
    bool spinDir_ = false; // true : left, false: right
    uint32_t spinCnt_ = 0;

    float getVariableSpinVel()
    {
        float x = 2.f * (static_cast<float>(spinCnt_++) * 0.001f);
        if (chassis_->msg.needRush)
            x *= 2.f;
        const auto rslt = (SPIN_SPEED_VAR * sinf(x) * sinf(x)) + SPIN_SPEED_BASE;
        return spinDir_ ? rslt : -rslt;
    }
    float getConstantSpinVel() { return spinDir_ ? SPIN_SPEED_CONST : -SPIN_SPEED_CONST; }
};
} // namespace CHASSIS
