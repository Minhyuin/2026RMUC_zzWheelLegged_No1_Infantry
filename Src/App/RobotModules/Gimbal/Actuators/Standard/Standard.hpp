#pragma once

#include "Gimbal.hpp"

#include "sdkconfig.h"

#include "IMotor.hpp"

#include "LPF.hpp"

#include "TxPacket.hpp"
#include "RxPacket.hpp"
#include "PacketImpl.hpp"

namespace GIMBAL {

union Motors_u {
    struct {
        PINYMOTOR::IMotor *pitch;
        PINYMOTOR::IMotor *yaw;
    };
    PINYMOTOR::IMotor *_[2];
};

union States_u {
    struct {
        float pos;
        float vel;
        float torq;
    };
    float _[2];
};

class ChassisCtrlTxPacket : public COMM::TxPacket<COMM::ChassisCtrlPacketType> {
    friend class Standard;
    // ChassisCtrlTxPacket() : TxPacket(500.f) {}
    void transmit(uint8_t *_buf, uint16_t _len) final;
};

class Standard : public Gimbal {
public:
    static constexpr float PITCH_LAUNCH_ANG = 4.68467999f;
    static constexpr float YAW_LAUNCH_ANG = 3.04590487f;

    static constexpr float PITCH_MOTOR_FREQ = 1000.f;
    static constexpr float YAW_MOTOR_FREQ = 1000.f;

    // "REVERSE flag" is true when imu and motor have different direction
    static constexpr bool PITCH_REVERSE = true;
    static constexpr bool YAW_REVERSE = false;

    Standard();

    PINYMOTOR::IMotor *pitch() const;
    PINYMOTOR::IMotor *yaw() const;

    void ctrlPitchSmooth(PID *_angPID, PID *_velPID, float _pos, float _step);
    void ctrlPitch(PID *_angPID, PID *_velPID, float _pos);

    void ctrlYawSmooth(PID *_angPID, PID *_velPID, float _pos, float _step);
    void ctrlYaw(PID *_angPID, PID *_velPID, float _pos);

    void stop();
    void enter();
    void update(void *_param);

    void loadResetState(bool _state);

private:
    Motors_u motors_;

    ChassisCtrlTxPacket chassisCtrlTxPacket_;

    FILTER::LPF outFilrPitch_{ PITCH_MOTOR_FREQ, 1000.f };
    FILTER::LPF outFilrYaw_{ YAW_MOTOR_FREQ, 1000.f };
};

} // namespace GIMBAL
