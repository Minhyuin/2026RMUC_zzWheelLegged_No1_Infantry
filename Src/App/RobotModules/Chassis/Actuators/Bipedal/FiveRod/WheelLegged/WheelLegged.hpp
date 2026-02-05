#pragma once

#include "VMC.hpp"
#include "IMotor.hpp"
#include "PidBasic.hpp"
#include "LPF.hpp"

namespace CHASSIS {

class WheelLegged {
    static constexpr float MASS_WHEEL = 0.334f + 0.179f;
    static constexpr float MASS_LEG = 1.43744054f;

    static constexpr float MAX_HIP_CTRL_TORQ = 40.f;
    static constexpr float MAX_WHEEL_CTRL_TORQ = 3.94f;

    static constexpr float MAX_FD = 120.f;
    static constexpr float DEFAULT_LENGTH_ANG_PID_PARAM[7] = { 30.f, 0.f, 1.f, 0.001f, 0.f, 3.f, 0.f };
    static constexpr float DEFAULT_LENGTH_VEL_PID_PARAM[7] = { 30.f, 0.f, 0.f, 0.001f, 0.f, MAX_FD, 0.f };

public:
    static constexpr float MAX_L = 0.36f; // 0.36998937
    static constexpr float MIN_L = 0.17f; // 0.16051326

    static constexpr float GRAVITY = 9.7833f;
    static constexpr float MASS = MASS_WHEEL + MASS_LEG;

    static constexpr float FN_THRESHOLD = MASS_WHEEL * GRAVITY;

    void init(PINYMOTOR::IMotor *_hip1, PINYMOTOR::IMotor *_hip4, PINYMOTOR::IMotor *_wheel);
    void stop();
    void enter();

    void stateUpdate(float _dt);

    void overrideLengthPid(const PositionalPid &_angPID, const PositionalPid &_velPID);
    void overrideLengthPid();
    void setL(float _l);
    void setFdForward(float _fd);
    void setTp(float _tp);
    void setTw(float _tw);

    bool isFloating(float _az, float _q);

    void ctrl();

    const LegState_s &state() const { return legState_; }

    bool floatFlag = false;
    bool needJumping = false;

private:
    LegState_s legState_{};

    float tp_ = 0.f;
    float fd_ = 0.f;
    float fdForward_ = 0.f;
    float tw_ = 0.f;

    FILTER::LPF fdLPF_{ 1000.f, 100.f };

    float fn_ = 0.f;
    FILTER::LPF fnLPF_{ 1000.f, 5.f };
    float fnThreshold_ = FN_THRESHOLD;
    float rawFn_ = 0.f;
    float fp_ = 0.f;
    float nWheel_ = 0.f;

    VMC vmc_ = VMC();
    // VMCTest vmcTest_;

    float setL_ = 0.f;

    // TODO: 位置环积分饱和会影响离地检测
    PositionalPid lengthAngPid_{ DEFAULT_LENGTH_ANG_PID_PARAM[0], DEFAULT_LENGTH_ANG_PID_PARAM[1],
                                 DEFAULT_LENGTH_ANG_PID_PARAM[2], DEFAULT_LENGTH_ANG_PID_PARAM[3],
                                 DEFAULT_LENGTH_ANG_PID_PARAM[4], DEFAULT_LENGTH_ANG_PID_PARAM[5],
                                 DEFAULT_LENGTH_ANG_PID_PARAM[6] };
    PositionalPid lengthVelPid_{ DEFAULT_LENGTH_VEL_PID_PARAM[0], DEFAULT_LENGTH_VEL_PID_PARAM[1],
                                 DEFAULT_LENGTH_VEL_PID_PARAM[2], DEFAULT_LENGTH_VEL_PID_PARAM[3],
                                 DEFAULT_LENGTH_VEL_PID_PARAM[4], DEFAULT_LENGTH_VEL_PID_PARAM[5],
                                 DEFAULT_LENGTH_VEL_PID_PARAM[6] };

    PINYMOTOR::IMotor *hip1_;
    PINYMOTOR::IMotor *hip4_;
    PINYMOTOR::IMotor *wheel_;

    float minL(float _phi) const;
};

} // namespace CHASSIS
