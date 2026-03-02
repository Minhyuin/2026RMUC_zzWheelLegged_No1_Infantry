#pragma once

#include "VMC.hpp"
#include "GasSpring.hpp"
#include "IMotor.hpp"
#include "PidBasic.hpp"
#include "LPF.hpp"

namespace CHASSIS {

class WheelLegged {
    static constexpr float MASS_WHEEL = 0.334f + 0.179f;
    static constexpr float MASS_LEG = 1.43744054f;

    static constexpr float MAX_HIP_CTRL_TORQ = 39.f;
    static constexpr float MAX_WHEEL_CTRL_TORQ = 3.94f;

    // 太硬会导致电机输出超过电池可承受荷载，影响tp分量
    static constexpr float FLOAT_LENGTH_ANG_PID_PARAM[7] = { 40.f, 0.f, 1.5f, 0.001f, 0.f, 3.f, 0.f };
    static constexpr float FLOAT_LENGTH_VEL_PID_PARAM[7] = { 30.f, 0.f, 0.f, 0.001f, 0.f, 250.f, 0.f };

public:
    static constexpr float MAX_L = 0.36f; // 0.36998937
    static constexpr float MIN_L = 0.17f; // 0.16051326

    static constexpr float GRAVITY = 9.7833f;
    static constexpr float MASS = MASS_WHEEL + MASS_LEG;

    static constexpr float FLOATING_THRESHOLD = MASS_WHEEL * GRAVITY;

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
     * @brief run the vmc solver, update leg state
     * 
     * @param _dt unit: s
     */
    void stateUpdate(float _dt);

    /**
     * @brief override the length PIDs with custom parameters
     * 
     * @param _angPID 
     * @param _velPID 
     */
    void overrideLengthPid(const PositionalPid &_angPID, const PositionalPid &_velPID);
    /**
     * @brief override the length PIDs with default parameters
     */
    void overrideLengthPid();
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

    float tp_ = 0.f;
    float fd_ = 0.f;
    float fdFeedforward_ = 0.f;
    float tw_ = 0.f;

    float fn_ = 0.f;
    FILTER::LPF fnLPF_{ 1000.f, 5.f };
    float fnThreshold_ = FLOATING_THRESHOLD;
    float rawFn_ = 0.f;
    float fp_ = 0.f;
    float nWheel_ = 0.f;

    VMC vmc_ = VMC();
    // VMCTest vmcTest_;

    GasSpring spring_{ 100.f, 0.f };
    float fdFromSpring_ = 0;

    float setL_ = 0.f;

    PositionalPid lengthAngPid_{ FLOAT_LENGTH_ANG_PID_PARAM[0], FLOAT_LENGTH_ANG_PID_PARAM[1],
                                 FLOAT_LENGTH_ANG_PID_PARAM[2], FLOAT_LENGTH_ANG_PID_PARAM[3],
                                 FLOAT_LENGTH_ANG_PID_PARAM[4], FLOAT_LENGTH_ANG_PID_PARAM[5],
                                 FLOAT_LENGTH_ANG_PID_PARAM[6] };
    PositionalPid lengthVelPid_{ FLOAT_LENGTH_VEL_PID_PARAM[0], FLOAT_LENGTH_VEL_PID_PARAM[1],
                                 FLOAT_LENGTH_VEL_PID_PARAM[2], FLOAT_LENGTH_VEL_PID_PARAM[3],
                                 FLOAT_LENGTH_VEL_PID_PARAM[4], FLOAT_LENGTH_VEL_PID_PARAM[5],
                                 FLOAT_LENGTH_VEL_PID_PARAM[6] };

    PINYMOTOR::IMotor *hip1_;
    PINYMOTOR::IMotor *hip4_;
    PINYMOTOR::IMotor *wheel_;

    float minL(float _phi) const;
};

} // namespace CHASSIS
