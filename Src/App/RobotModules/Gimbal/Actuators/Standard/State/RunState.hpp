#pragma once

#include "FSMState.hpp"
#include "Standard.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

// #include "YawSI.hpp"
// YawSI yawLink{ 1000.f, 10.f };

static volatile float pitchGimbalExpOffset = 0.f;
static volatile float yawGimbalExpOffset = 0.f;

static volatile float yawMotorAng = 0.f;
static volatile float pitchMotorAng = 0.f;

static PositionalPid runPitchAngPID(60.f, 500.f, 0.65f, 0.001f, 4.f, 15.f, 0.f);
static PositionalPid runPitchVelPID(0.8f, 0.f, 0.f, 0.001f, 0.f, 7.f, 0.f);
static PositionalPid runYawAngPID(30.f, 50.f, 0.5f, 0.001f, 0.5f, 10.f, 0.f);
static PositionalPid runYawVelPID(1.f, 0.f, 0.f, 0.001f, 0.f, 7.f, 0.f);

static PositionalPid runYawAngForWeightlessnessPID(20.f, 50.f, 0.4f, 0.001f, 0.5f, 3.f, 0.f);
static PositionalPid runYawVelForWeightlessnessPID(0.65f, 0.f, 0.f, 0.001f, 0.f, 7.f, 0.f);

namespace GIMBAL {

class RunState : public FSMState<FSMState_e> {
    struct Pids_s {
        PositionalPid *pitchAngPid;
        PositionalPid *pitchVelPid;

        PositionalPid *yawAngPid;
        PositionalPid *yawVelPid;

        PositionalPid *yawAngForWeightlessnessPid;
        PositionalPid *yawVelForWeightlessnessPid;
    };

    static constexpr float IMU_PITCH_START_ANG = Standard::IMU_PITCH_START_ANG_MECH;
    static constexpr float IMU_PITCH_END_ANG = Standard::IMU_PITCH_END_ANG_MECH;

    static constexpr float IMU_YAW_START_ANG = -PI / 1.f;
    static constexpr float IMU_YAW_END_ANG = PI / 1.f;

    static constexpr uint16_t MAX_NEED_LAUNCH_CNT = 5000;

public:
    RunState(Standard *_gimbal) : FSMState<FSMState_e>(FSMState_e::RUN), gimbal_(_gimbal)
    {
        pids_.pitchAngPid = &runPitchAngPID;
        pids_.pitchVelPid = &runPitchVelPID;
        pids_.yawAngPid = &runYawAngPID;
        pids_.yawVelPid = &runYawVelPID;
        pids_.yawAngForWeightlessnessPid = &runYawAngForWeightlessnessPID;
        pids_.yawVelForWeightlessnessPid = &runYawVelForWeightlessnessPID;
    }

    void enter() final
    {
        cmdImuPitchAng_ = gimbal_->ins().pitch;
        cmdImuYawAng_ = gimbal_->ins().yaw;

        gimbal_->enter();

        LOG::info("GimbalRun", "enter");
    }

    void run() final
    {
        // yawLink.update(gimbal_->yaw()->data().torq, gimbal_->ins().body.gz);

        // update ctrl ang
        cmdImuPitchAng_ = PINYMOTOR::clampArc(cmdImuPitchAng_ + (gimbal_->msg.wPitch * 0.001f), IMU_PITCH_START_ANG,
                                              IMU_PITCH_END_ANG);
        cmdImuPitchAngWithOffset_ = cmdImuPitchAng_ + pitchGimbalExpOffset;
        cmdImuPitchAngWithOffset_ =
                PINYMOTOR::clampArc(cmdImuPitchAngWithOffset_, IMU_PITCH_START_ANG, IMU_PITCH_END_ANG);

        // update yaw range
        cmdImuYawAng_ = cmdImuYawAng_ + (gimbal_->msg.wYaw * 0.001f);
        cmdImuYawAngWithOffset_ = cmdImuYawAng_ + yawGimbalExpOffset;

        gimbal_->ctrlPitch(pids_.pitchAngPid, pids_.pitchVelPid, cmdImuPitchAngWithOffset_);
        if (gimbal_->isChassisWeightlessness() && gimbal_->chassisState() != CHASSIS::FSMState_e::STOP) {
            if (fabs(PINYMOTOR::getMinorArc(gimbal_->yaw()->posNorm(), 0)) > PI / 2.f) {
                gimbal_->ctrlMotorYaw(pids_.yawAngForWeightlessnessPid, pids_.yawVelForWeightlessnessPid, PI);
            } else {
                gimbal_->ctrlMotorYaw(pids_.yawAngForWeightlessnessPid, pids_.yawVelForWeightlessnessPid, 0.f);
            }
            cmdImuYawAng_ = gimbal_->ins().yaw;
        } else {
            gimbal_->ctrlYaw(pids_.yawAngPid, pids_.yawVelPid, cmdImuYawAngWithOffset_);
        }
    }

    void exit() final
    {
        pids_.pitchAngPid->reset();
        pids_.pitchVelPid->reset();
        pids_.yawAngPid->reset();
        pids_.yawVelPid->reset();
        pids_.yawAngForWeightlessnessPid->reset();
        pids_.yawVelForWeightlessnessPid->reset();
        LOG::info("GimbalRun", "exit");
    }

    FSMState_e checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;

        if (gimbal_->isNeedLaunch())
            return FSMState_e::LAUNCH;

        if (gimbal_->msg.state == FSMState_e::AUTO && gimbal_->vision.data().control)
            return FSMState_e::AUTO;
        else if (gimbal_->msg.state == FSMState_e::U_TURN)
            return FSMState_e::U_TURN;
        else
            return FSMState_e::RUN;
    }

private:
    Standard *gimbal_;

    Pids_s pids_;

    // float imuYawMinAng_ = IMU_YAW_START_ANG;
    // float imuYawMaxAng_ = IMU_YAW_END_ANG;

    float cmdImuPitchAng_;
    float cmdImuPitchAngWithOffset_;

    float cmdImuYawAng_;
    float cmdImuYawAngWithOffset_;
};

} // namespace GIMBAL
