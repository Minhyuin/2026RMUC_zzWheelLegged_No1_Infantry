#pragma once

#include "ArmorBooster.hpp"

#include "IMotor.hpp"

#include "PidBasic.hpp"

#include "IIR.hpp"

#include "TxPacket.hpp"
#include "PacketImpl.hpp"

namespace ARMORBOOSTER {

class ArmorBoosterStatePacketType : public COMM::TxPacket<COMM::ArmorBoosterStatePacketType> {
    friend class TripFric;
    ArmorBoosterStatePacketType() : TxPacket(10.f) {}
    void transmit(uint8_t *_buf, uint16_t _len) final;
};

class TripFric : public ArmorBooster {
    union Motors_u {
        struct {
            PINYMOTOR::IMotor *fricL;
            PINYMOTOR::IMotor *fricM;
            PINYMOTOR::IMotor *fricR;
        };
        PINYMOTOR::IMotor *_[3];
    };

public:
    static constexpr float FAST_FRIC_RATE = 1020.f;
    static constexpr float SLOW_FRIC_RATE = 314.f;

    static constexpr float FRIC_STOP_THRESHOLD = 5.f;

    static constexpr float FRIC_MOTOR_FREQ = 1000.f;

    TripFric();

    void ctrl(float _vel);

    void on();
    void off();

    void stop();
    void enter();
    void update(void *_param);

private:
    Motors_u motors_;

    ArmorBoosterStatePacketType armorBoosterStateTxPacket_;

    FILTER::LPFIIR3 velFltr_[3] = {
        { 1000.f, 20.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE },
        { 1000.f, 20.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE },
        { 1000.f, 20.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE },
    };

    PositionalPid fricPid_[3] = {
        { 0.00026f, 0.001f, 6e-6f, 0.001f, 0.0038f, 5.000000e-02f },
        { 0.00025f, 0.0007f, 5e-6f, 0.001f, 0.0035f, 5.000000e-02f },
        { 0.00028f, 0.0007f, 6e-6f, 0.001f, 0.003f, 5.000000e-02f },
    };

    float targetVel_ = 0;
};

}; // namespace ARMORBOOSTER