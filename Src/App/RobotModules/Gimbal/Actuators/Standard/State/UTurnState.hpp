#pragma once

#include "FSMState.hpp"
#include "Standard.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

#if EXTENSION_BUZZER
#include "Buzzer.hpp"
#endif

static PositionalPid uTurnPitchAngPID(60.f, 500.f, 0.65f, 0.001f, 4.f, 15.f, 0.f);
static PositionalPid uTurnPitchVelPID(0.8f, 0.f, 0.f, 0.001f, 0.f, 7.f, 0.f);
static PositionalPid uTurnYawAngPID(50.f, 50.f, 1.7f, 0.001f, 0.5f, 10.f, 0.f);
static PositionalPid uTurnYawVelPID(1.5f, 0.f, 0.f, 0.001f, 0.f, 7.f, 0.f);

namespace GIMBAL {

class UTurnState : public FSMState<FSMState_e> {
    struct Pids_s {
        PositionalPid *pitchAngPid;
        PositionalPid *pitchVelPid;

        PositionalPid *yawAngPid;
        PositionalPid *yawVelPid;
    };

    static constexpr float IMU_PITCH_START_ANG = Standard::IMU_PITCH_START_ANG_MECH;
    static constexpr float IMU_PITCH_END_ANG = Standard::IMU_PITCH_END_ANG_MECH;

    static constexpr uint16_t MAX_TURN_OVER_TIME_CNT = 2000;
    static constexpr uint16_t MAX_WAIT_OVER_TIME_CNT = 3000;

public:
    UTurnState(Standard *_gimbal) : FSMState<FSMState_e>(FSMState_e::U_TURN), gimbal_(_gimbal)
    {
        pids_.pitchAngPid = &uTurnPitchAngPID;
        pids_.pitchVelPid = &uTurnPitchVelPID;
        pids_.yawAngPid = &uTurnYawAngPID;
        pids_.yawVelPid = &uTurnYawVelPID;
    }

    void enter() final
    {
        cmdImuPitchAng_ = gimbal_->ins().pitch;
        cmdImuYawAng_ = gimbal_->ins().yaw + PI;

        turnFinish_ = false;

        gimbal_->enter();

        LOG::info("GimbalUTurn", "enter");

#if EXTENSION_BUZZER
        BUZZER::buzz->playNote(BUZZER::Tone_e::FS7, 50);
        BUZZER::buzz->playNote(BUZZER::Tone_e::C7, 50);
#endif
    }

    void run() final
    {
        // update finish status
        if (fabsf(PINYMOTOR::getMinorArc(gimbal_->ins().yaw, cmdImuYawAng_)) < 0.1f || turnFinish_) {
            turnFinish_ = true;
        }
        turnOvertimeCnt_++;

        // update ctrl ang
        gimbal_->ctrlPitch(pids_.pitchAngPid, pids_.pitchVelPid, cmdImuPitchAng_);
        gimbal_->ctrlYaw(pids_.yawAngPid, pids_.yawVelPid, cmdImuYawAng_);
    }

    void exit() final
    {
        pids_.pitchAngPid->reset();
        pids_.pitchVelPid->reset();
        pids_.yawAngPid->reset();
        pids_.yawVelPid->reset();
        turnOvertimeCnt_ = 0;
        waitOvertimeCnt_ = 0;
        LOG::info("GimbalUTurn", "exit");
    }

    FSMState_e checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;

        if (gimbal_->isNeedLaunch())
            return FSMState_e::LAUNCH;

        if (turnFinish_ || turnOvertimeCnt_ >= MAX_TURN_OVER_TIME_CNT) {
            if (gimbal_->msg.state == FSMState_e::AUTO && gimbal_->vision.data().control)
                return FSMState_e::AUTO;
            else
                return FSMState_e::RUN;
        } else
            return FSMState_e::U_TURN;
    }

    bool change() final
    {
        gimbal_->ctrlPitch(pids_.pitchAngPid, pids_.pitchVelPid, gimbal_->ins().pitch);
        gimbal_->ctrlYaw(pids_.yawAngPid, pids_.yawVelPid, gimbal_->ins().yaw);
        if (gimbal_->chassisState() == CHASSIS::FSMState_e ::TANK ||
            gimbal_->chassisState() == CHASSIS::FSMState_e ::STOP) {
            return true;
        } else if (waitOvertimeCnt_ >= MAX_WAIT_OVER_TIME_CNT) {
            turnOvertimeCnt_ = MAX_TURN_OVER_TIME_CNT;
            return true;
        } else {
            waitOvertimeCnt_++;
            return false;
        }
    }

private:
    Standard *gimbal_;

    Pids_s pids_;

    uint16_t turnOvertimeCnt_ = 0;
    uint16_t waitOvertimeCnt_ = 0;
    bool turnFinish_ = false;

    float cmdImuPitchAng_;
    float cmdImuYawAng_;
};

} // namespace GIMBAL
