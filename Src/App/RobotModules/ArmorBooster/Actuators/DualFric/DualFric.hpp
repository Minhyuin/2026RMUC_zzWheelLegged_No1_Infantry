#pragma once

#include "ArmorBooster.hpp"

#include "IMotor.hpp"

#include "IIR.hpp"

#include "PidBasic.hpp"

namespace ARMORBOOSTER {

class DualFric : public ArmorBooster {
    union Motors_u {
        struct {
            PINYMOTOR::IMotor *fricL;
            PINYMOTOR::IMotor *fricR;
        };
        PINYMOTOR::IMotor *_[2];
    };

public:
    static constexpr float FAST_FRIC_RATE = 628.f;
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
    Motors_u motors_;

    FILTER::IIR3 velFltr_[2];

    IncrementalPid fricPid_[2]{ { 0.0003f, 1e-6f, 0.f, 0.157f }, { 0.0003f, 1e-6f, 0.f, 0.157f } };

    float targetVel_ = SLOW_FRIC_RATE;
};

}; // namespace ARMORBOOSTER
