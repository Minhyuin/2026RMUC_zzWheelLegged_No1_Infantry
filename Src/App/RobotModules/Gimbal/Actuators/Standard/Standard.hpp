#pragma once

#include "Gimbal.hpp"

#include "sdkconfig.h"

#include "IMotor.hpp"

#include "LPF.hpp"

#include "Vision.hpp"

#include "TxPacket.hpp"
#include "RxPacket.hpp"
#include "PacketImpl.hpp"

namespace GIMBAL {

class ChassisCtrlTxPacket : public COMM::TxPacket<COMM::ChassisCtrlPacketType> {
    friend class Standard;
    ChassisCtrlTxPacket() : TxPacket(500.f) {}
    void transmit(uint8_t *_buf, uint16_t _len) final;
};

class ShooterCtrlTxPacket : public COMM::TxPacket<COMM::ArmorLoaderPacketType> {
    friend class Standard;
    ShooterCtrlTxPacket() : TxPacket(500.f) {}
    void transmit(uint8_t *_buf, uint16_t _len) final;
};

class RefereeDataRxPacket : public COMM::RxPacket<COMM::RefereePacketType, 1> {
public:
    RefereeDataRxPacket() { registerCallback(); }
    void registerCallback() final;
};

class GimbalLaunchStrategyRxPacket : public COMM::RxPacket<COMM::GimbalLaunchStrategyPacketType, 1> {
public:
    GimbalLaunchStrategyRxPacket() { registerCallback(); }
    void registerCallback() final;
};

class Standard : public Gimbal {
    struct Motors_s {
        PINYMOTOR::IMotor *pitch;
        PINYMOTOR::IMotor *yaw;
    };

public:
    static constexpr float PITCH_LAUNCH_ANG = 5.36427498f;
    static constexpr float YAW_LAUNCH_ANG = 3.90537f;

    static constexpr float PITCH_MOTOR_FREQ = 500.f;
    static constexpr float YAW_MOTOR_FREQ = 500.f;

    static constexpr float YAW_INERTIA = 0.0397887f;
    static constexpr float YAW_FRICTION = 0.0932377f;
    static constexpr float YAW_COULOMB = 0.253459f;

    // "REVERSE flag" is true when imu and motor have different direction
    static constexpr bool PITCH_REVERSE = false;
    static constexpr bool YAW_REVERSE = false;

    Standard();

    Vision vision;

    PINYMOTOR::IMotor *pitch() const;
    PINYMOTOR::IMotor *yaw() const;

    void ctrlPitchSmooth(PID *_angPID, PID *_velPID, float _pos, float _step);
    void ctrlPitchAuto(PID *_angPID, PID *_velPID, float _pos);
    void ctrlPitch(PID *_angPID, PID *_velPID, float _pos);
    void ctrlMotorPitch(PID *_angPID, PID *_velPID, float _pos);

    void ctrlYawSmooth(PID *_angPID, PID *_velPID, float _pos, float _step);
    void ctrlYawAuto(PID *_angPID, PID *_velPID, PID *_velFeedbackPID, float _pos);
    void ctrlYaw(PID *_angPID, PID *_velPID, float _pos);
    void ctrlMotorYaw(PID *_angPID, PID *_velPID, float _pos);

    LaunchStrategy_e generateLaunchStrategy();

    void stop();
    void enter();
    void update(void *_param);

    void loadResetState(bool _state);

    bool isNeedLaunch();
    bool isAllowExitLaunch();

private:
    Motors_s motors_;

    ChassisCtrlTxPacket chassisCtrlTxPacket_;
    ShooterCtrlTxPacket shooterCtrlTxPacket_;

    RefereeDataRxPacket refereeDataRxPacket_;
    GimbalLaunchStrategyRxPacket gimbalLaunchStrategyRxPacket_;

    ChassisMsg_s cmsg_{};      // master keep it temporary
    ArmorLoaderMsg_s almsg_{}; // master keep it temporary

    FILTER::LPF outFilrPitch_{ PITCH_MOTOR_FREQ, 1000.f };
    FILTER::LPF outFilrYaw_{ YAW_MOTOR_FREQ, 1000.f };

    float calcPitchTorq(float _ang, float _vel, float _acc);
    float calcYawTorq(float _vel, float _acc);
};

} // namespace GIMBAL
