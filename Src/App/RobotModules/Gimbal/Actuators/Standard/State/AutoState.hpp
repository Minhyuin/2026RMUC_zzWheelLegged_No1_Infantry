#pragma once

#include "FSMState.hpp"
#include "Standard.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

static PositionalPid autoPitchAngPID(70.f, 300.f, 0.5f, 0.001f, 1.f, 1.5f, 0.f);
static PositionalPid autoPitchVelPID(0.7f, 0.f, 0.f, 0.001f, 0.f, 7.f, 0.f);

static PositionalPid autoYawAngPID(600.f, 1000.f, 11.f, 0.001f, 15.f, 30.f, 0.f);
static PositionalPid autoYawVelPID(5.0f, 1.f, 0.f, 0.001f, 10.f, 100.f, 0.f);
static PositionalPid autoYawVelFeedbackPID(15.0f, 5.f, 1.6f, 0.001f, 10.f, 50.f, 0.f);

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
    static constexpr float IMU_PITCH_END_ANG = 0.19f;

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

        LOG::info("GimbalAuto", "enter");
    }

    void run() final
    {
        cmdImuPitchAng_ = // gimbal_->vision.data().pitch;
                std::clamp(gimbal_->vision.data().pitch, gimbal_->ins().pitch - 0.1f, gimbal_->ins().pitch + 0.1f);

        cmdImuYawAng_ = gimbal_->vision.data().yaw;

        cmdImuPitchAng_ = PINYMOTOR::clampArc(cmdImuPitchAng_, IMU_PITCH_START_ANG, IMU_PITCH_END_ANG);

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

        if (gimbal_->isNeedLaunch())
            return FSMState_e::LAUNCH;

        if (gimbal_->msg.state == FSMState_e::RUN || !gimbal_->vision.data().control)
            return FSMState_e::RUN;
        else
            return FSMState_e::AUTO;
    }

private:
    Standard *gimbal_;

    Pids_s pids_;

    float cmdImuPitchAng_;

    float cmdImuYawAng_;
};

} // namespace GIMBAL
