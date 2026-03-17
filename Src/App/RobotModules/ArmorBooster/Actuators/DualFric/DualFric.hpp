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
    static constexpr float FAST_FRIC_RATE = 628.f;
    static constexpr float SLOW_FRIC_RATE = 314.f;

    static constexpr float FRIC_STOP_THRESHOLD = 1.f;

    static constexpr float FRIC_MOTOR_FREQ = 20.f; // 1000.f

    static constexpr bool FRIC_REVERSE = true;

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
        { 1000.f, 20.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE },
        { 1000.f, 20.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE },
    };

    PositionalPid fricPid_[2]{ { 0.00025f, 4e-6f, 3e-7f, 0.001f, 0.003f, 0.182f, 0.f }, { fricPid_[0] } };

    float targetVel_ = SLOW_FRIC_RATE;
};

}; // namespace ARMORBOOSTER
