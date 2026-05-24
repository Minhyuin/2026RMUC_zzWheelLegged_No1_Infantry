#pragma once

#include "Standard.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

#if EXTENSION_BUZZER
#include "Buzzer.hpp"
#endif

static volatile float launchPitchStep = 0.6f * 0.001f;
static volatile float launchYawStep = 0.5f * 0.001f;

static PositionalPid launchPitchAngPID(50.f, 20.f, 0.1f, 0.001f, 1.0f, 5.f, 0.f);
static PositionalPid launchPitchVelPID(1.f, 1.f, 0.f, 0.001f, 7.f, 7.f, 0.f);
static PositionalPid launchYawAngPID(50.f, 100.f, 0.3f, 0.001f, 1.f, 3.5f, 0.f);
static PositionalPid launchYawVelPID(1.f, 1.f, 0.f, 0.001f, 7.f, 7.f, 0.f);

namespace GIMBAL {

class LaunchState : public FSMState<FSMState_e> {
    struct Pids_s {
        PositionalPid *pitchAngPid;
        PositionalPid *pitchVelPid;

        PositionalPid *yawAngPid;
        PositionalPid *yawVelPid;
    };

    static constexpr uint16_t MAX_LAUNCH_TIME = 50; // 0.3s

    static constexpr float PITCH_LAUNCH_ANG_DEADBAND = 0.1f;
    static constexpr float YAW_LAUNCH_ANG_DEADBAND = 0.07f;

    static constexpr float TARGET_YAW_CHENGE_THESHOLD = 1.9625f; // rad

    static constexpr float PITCH_LAUNCH_ANG = -0.15f;

public:
    LaunchState(Standard *_gimbal) : FSMState<FSMState_e>(FSMState_e::LAUNCH), gimbal_(_gimbal)
    {
        pids_.pitchAngPid = &launchPitchAngPID;
        pids_.pitchVelPid = &launchPitchVelPID;
        pids_.yawAngPid = &launchYawAngPID;
        pids_.yawVelPid = &launchYawVelPID;
    }

    void enter() final
    {
        launchFinish_[0] = launchFinish_[1] = false;

        cmdMotorPitchAng_ = gimbal_->pitch()->posNorm();
        cmdMotorYawAng_ = gimbal_->yaw()->posNorm();

        // pitch target
        targetPitch_ = PITCH_LAUNCH_ANG;

        // yaw target
        if (fabs(PINYMOTOR::getMinorArc(gimbal_->yaw()->posNorm(), 0)) > TARGET_YAW_CHENGE_THESHOLD) {
            targetYaw_ = PI;
        } else {
            targetYaw_ = 0;
        }

        launchStrategy_ = gimbal_->generateLaunchStrategy();

        gimbal_->enter();


        LOG::info("GimbalLaunch", "enter");

#if EXTENSION_BUZZER
        BUZZER::buzz->playNote(BUZZER::Tone_e::FS7, 50);
        BUZZER::buzz->playNote(BUZZER::Tone_e::C7, 50);
#endif
    }

    void run() final
    {
        // determine if the target position (0) has been reached
        switch (launchStrategy_) {
        case Gimbal::LaunchStrategy_e::NORMAL: {
            // no need to change cmd angle
            cmdMotorPitchAng_ = targetPitch_;
            cmdMotorYawAng_ = targetYaw_;
        }
        case Gimbal::LaunchStrategy_e::ANTI_CLOCKWISE: {
            cmdMotorYawAng_ += launchYawStep;
            break;
        }
        case Gimbal::LaunchStrategy_e::CLOCKWISE: {
            cmdMotorYawAng_ -= launchYawStep;
            break;
        }
        }

        if (std::fabs(PINYMOTOR::getMinorArc(targetPitch_, gimbal_->pitch()->posNorm())) < PITCH_LAUNCH_ANG_DEADBAND ||
            launchFinish_[0])
            launchFinish_[0] = true;
        else
            launchFinish_[0] = false;
        if (std::fabs(PINYMOTOR::getMinorArc(targetYaw_, gimbal_->yaw()->posNorm())) < YAW_LAUNCH_ANG_DEADBAND ||
            launchFinish_[1])
            launchFinish_[1] = true;
        else
            launchFinish_[1] = false;

        if (launchFinish_[0] && launchFinish_[1])
            gimbal_->loadResetState(true);

        gimbal_->ctrlMotorPitch(pids_.pitchAngPid, pids_.pitchVelPid, cmdMotorPitchAng_);
        if (launchFinish_[0])
            gimbal_->ctrlMotorYaw(pids_.yawAngPid, pids_.yawVelPid, cmdMotorYawAng_);
    }

    void exit() final
    {
        pids_.pitchAngPid->reset();
        pids_.pitchVelPid->reset();
        pids_.yawAngPid->reset();
        pids_.yawVelPid->reset();

        LOG::info("GimbalLaunch", "exit");
    }

    FSMState_e checkChange() final
    {
        bool launchFinish = launchFinish_[0] && launchFinish_[1] && gimbal_->isAllowExitLaunch();
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (launchFinish && gimbal_->msg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else if (launchFinish && gimbal_->msg.state == FSMState_e::AUTO)
            return FSMState_e::AUTO;
        else
            return FSMState_e::LAUNCH;
    }

private:
    Standard *gimbal_;

    Pids_s pids_;

    Gimbal::LaunchStrategy_e launchStrategy_ = Gimbal::LaunchStrategy_e::NORMAL;

    float cmdMotorPitchAng_;
    float cmdMotorYawAng_;

    float targetPitch_;
    float targetYaw_;

    bool launchFinish_[2] = { false, false };
};

} // namespace GIMBAL
