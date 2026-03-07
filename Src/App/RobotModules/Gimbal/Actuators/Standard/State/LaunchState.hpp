#pragma once

#include "Standard.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

static volatile float launchPitchStep = 0.6f;
static volatile float launchYawStep = 0.5f;

static PositionalPid launchPitchAngPID(60.f, 500.f, 1.f, 0.001f, 3.0f, 7.f, 0.f);
static PositionalPid launchPitchVelPID(0.5f, 0.f, 0.f, 0.001f, 0.f, 7.f, 0.f);
static PositionalPid launchYawAngPID(80.f, 600.f, 2.f, 0.001f, 3.f, 7.f, 0.f);
static PositionalPid launchYawVelPID(0.4f, 0.f, 0.f, 0.001f, 0.f, 7.f, 0.f);

namespace GIMBAL {

class LaunchState : public FSMState<FSMState_e> {
    struct Pids_s {
        PositionalPid *pitchAngPid;
        PositionalPid *pitchVelPid;

        PositionalPid *yawAngPid;
        PositionalPid *yawVelPid;
    };

    static constexpr uint16_t MAX_LAUNCH_TIME = 50; // 0.3s

    static constexpr float PITCH_LAUNCH_ANG_DEADBAND = 0.2f;
    static constexpr float YAW_LAUNCH_ANG_DEADBAND = 0.2f;

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
        launchFinish[0] = launchFinish[1] = false;

        cmdImuPitchAng_ = gimbal_->ins().pitch;
        cmdImuYawAng_ = gimbal_->ins().yaw;

        // pitch target
        // targetPitch_ = gimbal_->ins().pitch - gimbal_->pitch()->posNorm();
        targetPitch_ = PITCH_LAUNCH_ANG;

        // yaw target
        if (fabs(PINYMOTOR::getMinorArc(gimbal_->yaw()->posNorm(), 0)) > PI / 2.f) {
            targetYaw_ = gimbal_->ins().yaw - gimbal_->yaw()->posNorm() + PI;
        } else {
            targetYaw_ = gimbal_->ins().yaw - gimbal_->yaw()->posNorm();
        }

        launchStrategy_ = gimbal_->generateLaunchStrategy();

        gimbal_->enter();


        LOG::info("GimbalLaunch", "enter");
    }

    void run() final
    {
        // determine if the target position (0) has been reached
        float targetYawDeltaAng = 0;

        float pitchInsAng = gimbal_->ins().pitch;
        float yawMotorAng = gimbal_->yaw()->posNorm();
        switch (launchStrategy_) {
        case Gimbal::LaunchStrategy_e::NORMAL: {
            // no need to change cmd angle
            cmdImuPitchAng_ = targetPitch_;
            cmdImuYawAng_ = targetYaw_;
            if (yawMotorAng > PI / 2.f && yawMotorAng < 3.f * PI / 2.f)
                targetYawDeltaAng = PI;
            else
                targetYawDeltaAng = 0;
            break;
        }
        case Gimbal::LaunchStrategy_e::ANTI_CLOCKWISE: {
            cmdImuYawAng_ += launchYawStep;
            break;
        }
        case Gimbal::LaunchStrategy_e::CLOCKWISE: {
            cmdImuYawAng_ -= launchYawStep;
            break;
        }
        }

        if (std::fabs(PINYMOTOR::getMinorArc(PITCH_LAUNCH_ANG, pitchInsAng)) < PITCH_LAUNCH_ANG_DEADBAND ||
            launchFinish[0])
            launchFinish[0] = true;
        else
            launchFinish[0] = false;
        if (std::fabs(PINYMOTOR::getMinorArc(targetYawDeltaAng, yawMotorAng)) < YAW_LAUNCH_ANG_DEADBAND ||
            launchFinish[1])
            launchFinish[1] = true;
        else
            launchFinish[1] = false;


        gimbal_->ctrlPitchSmooth(pids_.pitchAngPid, pids_.pitchVelPid, cmdImuPitchAng_, launchPitchStep);
        if (launchFinish[0])
            gimbal_->ctrlYawSmooth(pids_.yawAngPid, pids_.yawVelPid, cmdImuYawAng_, launchYawStep);
    }

    void exit() final
    {
        gimbal_->loadResetState(true);

        pids_.pitchAngPid->reset();
        pids_.pitchVelPid->reset();
        pids_.yawAngPid->reset();
        pids_.yawVelPid->reset();

        LOG::info("GimbalLaunch", "exit");
    }

    FSMState_e checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (launchFinish[0] && launchFinish[1] && gimbal_->msg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else if (launchFinish[0] && launchFinish[1] && gimbal_->msg.state == FSMState_e::AUTO)
            return FSMState_e::AUTO;
        else
            return FSMState_e::LAUNCH;
    }

private:
    Standard *gimbal_;

    Pids_s pids_;

    Gimbal::LaunchStrategy_e launchStrategy_ = Gimbal::LaunchStrategy_e::NORMAL;

    float cmdImuPitchAng_;
    float cmdImuYawAng_;

    float targetPitch_;
    float targetYaw_;

    bool launchFinish[2] = { false, false };
};

} // namespace GIMBAL
