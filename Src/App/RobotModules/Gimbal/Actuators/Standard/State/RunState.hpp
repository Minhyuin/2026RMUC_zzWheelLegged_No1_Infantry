#pragma once

#include "FSMState.hpp"
#include "Standard.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

namespace GIMBAL {


class RunState : public FSMState<FSMState_e> {
    static constexpr float IMU_PITCH_MIN_ANG = -0.5f;
    static constexpr float IMU_PITCH_MAX_ANG = 0.3f;

public:
    RunState(Standard *_gimbal)
            : FSMState<FSMState_e>(FSMState_e::RUN), gimbal_(_gimbal)
    {
    }

    void enter() final
    {
        cmdImuPitchAng_ = gimbal_->ins().pitch;
        cmdImuYawAng_ = gimbal_->ctrlYawAng();

        gimbal_->enter();

        LOG::info("GimbalRun", "enter");
    }

    void run() final
    {
        cmdImuPitchAng_ = PINYMOTOR::clampArc(
                cmdImuPitchAng_ + (gimbal_->msg.wPitch * 0.001f),
                IMU_PITCH_MIN_ANG, IMU_PITCH_MAX_ANG);
        cmdImuYawAng_ += gimbal_->msg.wYaw * 0.001f;
        cmdImuYawAng_ = cmdImuYawAng_ > PI ? cmdImuYawAng_ - (2 * PI) :
                                             (cmdImuYawAng_ < -PI ?
                                                      cmdImuYawAng_ + (2 * PI) :
                                                      cmdImuYawAng_);

        float insPitchDelArc =
                PINYMOTOR::getMinorArc(cmdImuPitchAng_, gimbal_->ins().pitch);
        float insYawDelArc =
                PINYMOTOR::getMinorArc(cmdImuYawAng_, gimbal_->ctrlYawAng());

        // joints' target pos
        cmdMotorPitchAng_ = PINYMOTOR::rangeMap(
                gimbal_->pitch()->posNorm() +
                (Standard::PITCH_REVERSE ? -insPitchDelArc : insPitchDelArc));
        cmdMotorYawAng_ = PINYMOTOR::rangeMap(
                gimbal_->yaw()->posNorm() +
                (Standard::YAW_REVERSE ? -insYawDelArc : insYawDelArc));

        // ctrl
        gimbal_->ctrlPitch(cmdMotorPitchAng_);
        gimbal_->ctrlYaw(cmdMotorYawAng_);
    }

    void exit() final { LOG::info("GimbalRun", "exit"); }

    FSMState_e checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (gimbal_->msg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else
            return FSMState_e::RUN;
    }

private:
    Standard *gimbal_;

    float cmdImuPitchAng_;
    float cmdImuYawAng_;

    float cmdMotorPitchAng_;
    float cmdMotorYawAng_;
};
} // namespace GIMBAL
