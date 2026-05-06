#pragma once

#include "ArmorBooster.hpp"

#include "IMotor.hpp"

#include "PidBasic.hpp"

#include "IIR.hpp"

#include "TxPacket.hpp"
#include "PacketImpl.hpp"

namespace ARMORBOOSTER {

class ArmorBoosterStatePacketType : public COMM::TxPacket<COMM::ArmorBoosterStatePacketType> {
    friend class DualFric;
    ArmorBoosterStatePacketType() : TxPacket(10.f) {}
    void transmit(uint8_t *_buf, uint16_t _len) final;
};

class DualFric : public ArmorBooster {
    struct Motors_s {
        PINYMOTOR::IMotor *fricL;
        PINYMOTOR::IMotor *fricR;
    };

public:
    static constexpr float FAST_FRIC_RATE = 628.f;
    static constexpr float SLOW_FRIC_RATE = 314.f;

    static constexpr float FRIC_STOP_THRESHOLD = 1.f;

    static constexpr float FRIC_MOTOR_FREQ = 20.f;

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

    ArmorBoosterStatePacketType armorBoosterStateTxPacket_;

    float targetVel_ = SLOW_FRIC_RATE;
};

}; // namespace ARMORBOOSTER