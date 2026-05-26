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
    union Motors_u {
        struct {
            PINYMOTOR::IMotor *fricL;
            PINYMOTOR::IMotor *fricR;
        };
        PINYMOTOR::IMotor *dat[2];
    };

public:
    static constexpr float FAST_FRIC_RATE = 628.f;
    static constexpr float SLOW_FRIC_RATE = 314.f;

    static constexpr float FRIC_RUNNING_THRESHILD = FAST_FRIC_RATE - 200.f;

    static constexpr float FRIC_STOP_THRESHOLD = 1.f;

    static constexpr float FRIC_MOTOR_FREQ = 1000.f;

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

    ArmorBoosterStatePacketType armorBoosterStateTxPacket_;

    FILTER::LPFIIR3 velFltr_[2] = {
        { 1000.f, 20.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE },
        { 1000.f, 20.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE },
    };

    float curVel_[2]{ 0.f, 0.f };

    PositionalPid fricPid_[2]{ { 0.0013f, 0.0001f, 2.5e-05f, 0.001f, 0.005f, 0.157f, 0.f }, { fricPid_[0] } };
    PositionalPid antiDiffPid_{ 1e-5f, 0.f, 2e-07f, 0.001f, 0.f, 0.157f, 0.1f };

    float targetVel_ = SLOW_FRIC_RATE;
};

}; // namespace ARMORBOOSTER