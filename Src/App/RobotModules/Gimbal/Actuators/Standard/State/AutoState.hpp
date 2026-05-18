#pragma once

#include "FSMState.hpp"
#include "Standard.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

static PositionalPid autoPitchAngPID(0, 0, 0, 0, 0, 0, 0);
static PositionalPid autoPitchVelPID(0, 0, 0, 0, 0, 0, 0);

static float autoPitchArmorAngPidPID[7]{ 60.f, 500.f, 0.f, 0.001f, 1.f, 6.f, 0.f };
static float autoPitchArmorVelPidPID[7]{ 1.f, 1.f, 0.008f, 0.001f, 0.5f, 7.f, 0.f };

static float autoPitchBuffAngPidPID[7]{ 50.f, 700.f, 0.f, 0.001f, 1.f, 1.5f, 0.f };
static float autoPitchBuffVelPidPID[7]{ 0.8f, 1.f, 0.002f, 0.001f, 1.f, 7.f, 0.f };

static PositionalPid autoYawAngPID{ 0, 0, 0, 0, 0, 0, 0 };
static PositionalPid autoYawVelPID(0, 0, 0, 0, 0, 0, 0);
static PositionalPid autoYawVelFeedbackPID(0, 0, 0, 0, 0, 0, 0);

static float autoYawArmorAngPidPID[7]{ 450.f, 1000.f, 1.f, 0.001f, 20.f, 30.f, 0.f };
static float autoYawArmorVelPidPID[7]{ 6.f, 10.f, 0.02f, 0.001f, 20.f, 999.f, 0.f };
static float autoYawVelFeedbackPidPID[7]{ 50.0f, 30.f, 0.3f, 0.001f, 20.f, 60.f, 0.f };

static float autoYawBuffAngPidPID[7]{ 150.f, 1000.f, 6.f, 0.001f, 15.f, 3.f, 0.f };
static float autoYawBuffVelPidPID[7]{ 2.f, 1.f, 0.007f, 0.001f, 10.f, 100.f, 0.f };
static float autoYawBuffVelFeedbackPidPID[7]{ 5.0f, 5.f, 0.5f, 0.001f, 10.f, 50.f, 0.f };

namespace GIMBAL {

class AutoState : public FSMState<FSMState_e> {
    struct Pids_s {
        PositionalPid *pitchAngPid;
        PositionalPid *pitchVelPid;
        PositionalPid *yawAngPid;
        PositionalPid *yawVelPid;
        PositionalPid *yawVelFbPid;
    };

    static constexpr float IMU_PITCH_START_ANG = Standard::IMU_PITCH_START_ANG_MECH;
    static constexpr float IMU_PITCH_END_ANG = Standard::IMU_PITCH_END_ANG_MECH;

    static constexpr uint16_t MAX_NEED_LAUNCH_CNT = 5000;

public:
    AutoState(Standard *_gimbal) : FSMState<FSMState_e>(FSMState_e::AUTO), gimbal_(_gimbal)
    {
        pids_.pitchAngPid = &autoPitchAngPID;
        pids_.pitchVelPid = &autoPitchVelPID;
        pids_.yawAngPid = &autoYawAngPID;
        pids_.yawVelPid = &autoYawVelPID;
        pids_.yawVelFbPid = &autoYawVelFeedbackPID;
        loadPidParam();
    }

    void enter() final
    {
        gimbal_->enter();

        loadPidParam();

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
        else if (gimbal_->msg.state == FSMState_e::U_TURN)
            return FSMState_e::U_TURN;
        else
            return FSMState_e::AUTO;
    }

private:
    static void setParamFromArray(PositionalPid *_pid, const float *_params)
    {
        _pid->setParam(_params[0], _params[1], _params[2], _params[3], _params[4], _params[5], _params[6]);
    }

    void loadPidParam()
    {
        if (gimbal_->targetInBuff()) {
            setParamFromArray(pids_.pitchAngPid, autoPitchBuffAngPidPID);
            setParamFromArray(pids_.pitchVelPid, autoPitchBuffVelPidPID);
            setParamFromArray(pids_.yawAngPid, autoYawBuffAngPidPID);
            setParamFromArray(pids_.yawVelPid, autoYawBuffVelPidPID);
            setParamFromArray(pids_.yawVelFbPid, autoYawBuffVelFeedbackPidPID);
        } else {
            setParamFromArray(pids_.pitchAngPid, autoPitchArmorAngPidPID);
            setParamFromArray(pids_.pitchVelPid, autoPitchArmorVelPidPID);
            setParamFromArray(pids_.yawAngPid, autoYawArmorAngPidPID);
            setParamFromArray(pids_.yawVelPid, autoYawArmorVelPidPID);
            setParamFromArray(pids_.yawVelFbPid, autoYawVelFeedbackPidPID);
        }
    }

    Standard *gimbal_;

    Pids_s pids_;

    float cmdImuPitchAng_;

    float cmdImuYawAng_;
};

} // namespace GIMBAL
