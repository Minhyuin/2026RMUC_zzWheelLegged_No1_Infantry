#pragma once

#include "VMC.hpp"
#include "GasSpring.hpp"
#include "IMotor.hpp"
#include "PidBasic.hpp"
#include "LPF.hpp"

namespace CHASSIS {

class WheelLegged {
    // 4个电机输出电流过大达到电池负载阈值，会严重影响姿态，需要并入超电来提高物理上限
    static constexpr float MAX_HIP_CTRL_TORQ = 30.f;
    static constexpr float MAX_WHEEL_CTRL_TORQ = 3.9f;

    static constexpr float LENGTH_ANG_PID_OUTMAX = 6.f;
    static constexpr float LENGTH_ANG_PID_KI = 10.f;
    static constexpr float LENGTH_ANG_PID_IMAX = 1.f;
    static constexpr float LENGTH_VEL_PID_OUTMAX = 200.f;

public:
    static constexpr float MAX_L = 0.325f; // 0.337014049
    static constexpr float MIN_L = 0.170f; // 0.16051326

    static constexpr float MASS_WHEEL = 0.334f + 0.179f;
    static constexpr float MASS_LEG = 1.43744054f;

    static constexpr float GRAVITY = 9.7833f;
    static constexpr float MASS = MASS_WHEEL + MASS_LEG;

    static constexpr float FLOATING_THRESHOLD = MASS_WHEEL * GRAVITY;

    WheelLegged(float _springFd, float _springRb = 0.f) : spring_{ _springFd, _springRb } {}

    /**
     * @brief WheelLegged singleton initialize
     * 
     * @param _hip1 input the pointer of motor which is responsible for hip1
     * @param _hip4  input the pointer of motor which is responsible for hip4
     * @param _wheel 
     */
    void init(PINYMOTOR::IMotor *_hip1, PINYMOTOR::IMotor *_hip4, PINYMOTOR::IMotor *_wheel);
    /**
     * @brief disable all motors and reset PIDs
     */
    void stop();
    /**
     * @brief enable all motors
     */
    void enter();

    /**
     * @brief set the flag to limit fd
     */
    void setFdLimitFlag(bool _flag);

    /**
     * @brief run the vmc solver, update leg state
     * 
     * @param _dt unit: s
     */
    void stateUpdate(float _dt);

    /**
     * @brief set the Leg Length AngPid params
     * 
     * @param _kp 
     * @param _kd 
     */
    void setLegLengthAngPidPD(float _kp, float _kd);

    /**
     * @brief set the Leg Length VelPid params
     * 
     * @param _kp 
     * @param _kd 
     */
    void setLegLengthVelPidPD(float _kp, float _kd);

    /**
     * @brief set the leg length, automatically clamp to the range [MIN_L, MAX_L]
     * 
     * @param _l unit: m
     */
    void setL(float _l);
    /**
     * @brief Set the additional feedforward force
     * 
     * @param _fd unit: N
     */
    void setFdFeedforward(float _fd);
    /**
     * @brief Set the additional feedforward torque
     * 
     * @param _tp unit: Nm
     */
    void setTpFeedforward(float _tp);
    /**
     * @brief Set the expected Tp, which is used to control the leg swinging
     * 
     * @param _tp unit: Nm
     */
    void setTp(float _tp);
    /**
     * @brief Set the expected Tw, which is used to control the wheel rolling
     * 
     * @param _tw unit: Nm
     */
    void setTw(float _tw);

    /**
     * @brief support fn calculation, check if the leg is floating (i.e., not in contact with ground)
     * 
     * @param _az vertical acceleration of the body (counteract with gravitational acceleration), unit: m/s^2
     * @param _q pitch angle of chassis, unit: rad
     * @return true 
     * @return false 
     */
    bool isFloating(float _az, float _q);

    /**
     * @brief calc pid, publish control commands to motors
     * @note should be called after stateUpdate() in each control loop
     */
    void ctrl();

    const LegState_s &state() const { return legState_; }

    bool floatFlag = false;
    bool needJumping = false;

private:
    LegState_s legState_{};

    bool fdLimitFlag_ = false;

    float tp_ = 0.f;
    float tpFeedforward_ = 0.f;
    float fd_ = 0.f;
    float fdFeedforward_ = 0.f;
    float tw_ = 0.f;
    float tau1_ = 0.f;
    float tau4_ = 0.f;

    float fn_ = 0.f;
    FILTER::LPF fnLPF_{ 1000.f, 10.f };
    float fnThreshold_ = FLOATING_THRESHOLD;
    float rawFn_ = 0.f;

    VMC vmc_ = VMC();
    // VMCTest vmcTest_;

    GasSpring spring_;
    float fdFromSpring_ = 0;

    float setL_ = 0.f;

    PositionalPid lengthAngPid_{
        50.f, LENGTH_ANG_PID_KI, 0.5f, 0.001f, LENGTH_ANG_PID_IMAX, LENGTH_ANG_PID_OUTMAX, 0.f
    };
    PositionalPid lengthVelPid_{ 30.f, 0.f, 0.f, 0.001f, 0.f, LENGTH_VEL_PID_OUTMAX, 0.f };

    PINYMOTOR::IMotor *hip1_;
    PINYMOTOR::IMotor *hip4_;
    PINYMOTOR::IMotor *wheel_;

    float minL(float _phi) const;
};

} // namespace CHASSIS
