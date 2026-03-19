#pragma once

#include "ArmorBooster.hpp"

#include "IMotor.hpp"

#include "PidBasic.hpp"

#include "IIR.hpp"

namespace ARMORBOOSTER {

class DualFric : public ArmorBooster {
    struct Motors_s {
        PINYMOTOR::IMotor *fricL;
        PINYMOTOR::IMotor *fricR;
    };

public:
    static constexpr float FAST_FRIC_RATE = 650.f;
    static constexpr float SLOW_FRIC_RATE = 314.f;

    static constexpr float FRIC_STOP_THRESHOLD = 1.f;

    static constexpr float FRIC_MOTOR_FREQ = 10.f;

    static constexpr bool FRIC_REVERSE = false;

    DualFric();

    void ctrl(float _vel);

    void on();
    void off();

    void stop();
    void enter();
    void update(void *_param);

private:
    Motors_s motors_;

    FILTER::LPFIIR3 velFltr_[2] = {
        { 500.f, 20.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE },
        { 500.f, 20.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE },
    };

    PositionalPid fricPid_[2]{ { 0.0005f, 0.0001f, 2e-06f, 0.001f, 0.005f, 0.157f, 0.f }, { fricPid_[0] } };
    PositionalPid antiDiffPid_{ 1e-5f, 0.f, 2e-07f, 0.001f, 0.f, 0.157f, 0.1f };

    float targetVel_ = SLOW_FRIC_RATE;
};

}; // namespace ARMORBOOSTER