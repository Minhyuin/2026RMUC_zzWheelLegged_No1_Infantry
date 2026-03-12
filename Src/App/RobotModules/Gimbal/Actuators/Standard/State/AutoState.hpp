#pragma once

#include "FSMState.hpp"
#include "Standard.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "Smooth.hpp"
#include "PidBasic.hpp"

static PositionalPid autoPitchAngPID(110.f, 250.f, 0.01f, 0.001f, 3.f, 1.5f, 0.f);
static PositionalPid autoPitchVelPID(1.35f, 0.f, 0.f, 0.001f, 0.f, 7.f, 0.f);

static PositionalPid autoYawAngPID(100.f, 150.f, 0.7f, 0.001f, 0.5f, 30.f, 0.f);
static PositionalPid autoYawVelPID(13.0f, 0.f, 0.f, 0.001f, 0.f, 100.f, 0.f);
static PositionalPid autoYawVelFeedbackPID(100.0f, 0.f, 0.f, 0.001f, 0.f, 20.f, 0.f);

namespace GIMBAL {

class AutoState : public FSMState<FSMState_e> {
    struct Pids_s {
        PositionalPid *pitchAngPid;
        PositionalPid *pitchVelPid;
        PositionalPid *yawAngPid;
        PositionalPid *yawVelPid;
        PositionalPid *yawVelFbPid;
    };

    static constexpr float IMU_PITCH_START_ANG = -0.35f;
    static constexpr float IMU_PITCH_END_ANG = 0.1f;

    static constexpr float IMU_YAW_START_ANG = -PI / 1.f;
    static constexpr float IMU_YAW_END_ANG = PI / 1.f;

    static constexpr uint16_t MAX_NEED_LAUNCH_CNT = 5000;

public:
    AutoState(Standard *_gimbal) : FSMState<FSMState_e>(FSMState_e::AUTO), gimbal_(_gimbal)
    {
        pids_.pitchAngPid = &autoPitchAngPID;
        pids_.pitchVelPid = &autoPitchVelPID;
        pids_.yawAngPid = &autoYawAngPID;
        pids_.yawVelPid = &autoYawVelPID;
        pids_.yawVelFbPid = &autoYawVelFeedbackPID;
    }

    void enter() final
    {
        gimbal_->enter();
        needLaunchCounter_ = 0;

        LOG::info("GimbalAuto", "enter");
    }

    void run() final
    {
        cmdImuPitchAng_ =
                std::clamp(gimbal_->vision.data().pitch, gimbal_->ins().pitch - 0.018f, gimbal_->ins().pitch + 0.018f);

        cmdImuYawAng_ = gimbal_->vision.data().yaw;

        cmdImuPitchAng_ = PINYMOTOR::clampArc(cmdImuPitchAng_, IMU_PITCH_START_ANG, IMU_PITCH_END_ANG);

        needLaunchCounter_ = needLaunchCounter_ > MAX_NEED_LAUNCH_CNT ? MAX_NEED_LAUNCH_CNT : needLaunchCounter_ + 1;

        gimbal_->ctrlPitch(pids_.pitchAngPid, pids_.pitchVelPid, cmdImuPitchAng_);
        gimbal_->ctrlYawAuto(pids_.yawAngPid, pids_.yawVelPid, pids_.yawVelFbPid, cmdImuYawAng_);
    }

    void exit() final
    {
        pids_.pitchAngPid->reset();
        pids_.pitchVelPid->reset();
        pids_.yawAngPid->reset();
        pids_.yawVelPid->reset();
        pids_.yawVelFbPid->reset();
        LOG::info("GimbalAuto", "exit");
    }
    FSMState_e checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;

        if (gimbal_->isNeedLaunch() && needLaunchCounter_ >= MAX_NEED_LAUNCH_CNT)
            return FSMState_e::LAUNCH;

        if (gimbal_->msg.state == FSMState_e::RUN || !gimbal_->vision.data().control)
            return FSMState_e::RUN;
        else
            return FSMState_e::AUTO;
    }

private:
    Standard *gimbal_;

    Pids_s pids_;

    uint16_t needLaunchCounter_ = 0;

    float cmdImuPitchAng_;

    float cmdImuYawAng_;
};

} // namespace GIMBAL
