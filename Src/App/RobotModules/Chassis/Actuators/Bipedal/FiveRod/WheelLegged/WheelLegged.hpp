#pragma once

#include "VMC.hpp"
#include "IMotor.hpp"
#include "PidBasic.hpp"
#include "LPF.hpp"

namespace CHASSIS {

class WheelLegged {
public:
    static constexpr float GRAVITY = 9.7833f;
    static constexpr float MASS_WHEEL = 0.521f;
    static constexpr float MAX_L = 0.33f;
    static constexpr float MIN_L = 0.2f;

    static constexpr float MAX_HIP_CTRL_TORQ = 40.f;
    static constexpr float MAX_WHEEL_CTRL_TORQ = 3.94f;

    static constexpr VMCConfig_s VMC_CONFIG = { .L1 = 0.215f,
                                                .L2 = 0.258f,
                                                .L3 = 0.258f,
                                                .L4 = 0.215f,
                                                .L5 = 0.f };
    void init(PINYMOTOR::IMotor *_hip1, PINYMOTOR::IMotor *_hip4,
              PINYMOTOR::IMotor *_wheel);
    void stop();
    void enter();

    void stateUpdate(float _dt);

    void setL(float _l);

    void setFdForward(float _fd);
    void setTp(float _tp);
    void setTw(float _tw);

    float calcFn(float _az, float _q);

    void ctrl();

    const LegState_s &state() const { return legState_; }

    bool isFloating = false;

private:
    LegState_s legState_;

    float tp_ = 0.f;
    float fd_ = 0.f;
    float fdForward_ = 0.f;
    float tw_ = 0.f;

    FILTER::LPF fdLPF_{ 1000.f, 100.f };

    float fn_ = 0.f;
    FILTER::LPF fnLPF_{ 1000.f, 5.f };
    float rawFn_ = 0.f;
    float fp_;
    float nWheel_;

    VMC vmc_ = VMC(VMC_CONFIG);
    // VMCTest vmcTest_;

    float setL_;

    // TODO: 位置环积分饱和会影响离地检测
    PositionalPid lengthAngPid_{ 30.f, 30.f, 1.f, 0.001f, 0.3f, 3.f, 0.f };
    PositionalPid lengthVelPid_{ 50.f, 0.f, 0.f, 0.001f, 0.f, 120.f, 0.f };

    PINYMOTOR::IMotor *hip1_;
    PINYMOTOR::IMotor *hip4_;
    PINYMOTOR::IMotor *wheel_;

    float minL(float _phi) const;
};

} // namespace CHASSIS
