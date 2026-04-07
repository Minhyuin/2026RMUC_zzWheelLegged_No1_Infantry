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

class ShooterCtrlTxPacket : public COMM::TxPacket<COMM::ArmorLoaderCtrlPacketType> {
    friend class Standard;
    ShooterCtrlTxPacket() : TxPacket(500.f) {}
    void transmit(uint8_t *_buf, uint16_t _len) final;
};

class GimbalStateTxPacket : public COMM::TxPacket<COMM::GimbalStatePacketType> {
    friend class Standard;
    GimbalStateTxPacket() : TxPacket(30.f) {}
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

class ChassisStateRxPacket : public COMM::RxPacket<COMM::ChassisStatePacketType, 1> {
    friend class Standard;
    ChassisStateRxPacket() { registerCallback(); }
    void registerCallback() final;
};

class Standard : public Gimbal {
    struct Motors_s {
        PINYMOTOR::IMotor *pitch;
        PINYMOTOR::IMotor *yaw;
    };

public:
    static constexpr float PITCH_LAUNCH_ANG = 4.71229315f;
    static constexpr float YAW_LAUNCH_ANG = 2.26506090f;

    static constexpr float PITCH_MOTOR_FREQ = 1000.f;
    static constexpr float YAW_MOTOR_FREQ = 1000.f;

    static constexpr float YAW_INERTIA = 0.0201694f;
    static constexpr float YAW_FRICTION = 0.f; // 0.109081
    static constexpr float YAW_COULOMB = 0.f;  // 0.105144

    // "REVERSE flag" is true when imu and motor have different direction
    static constexpr bool PITCH_REVERSE = true;
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

    CHASSIS::FSMState_e chassisState() const
    {
        return static_cast<CHASSIS::FSMState_e>(chassisStateRxPacket_.data().chassisRealState);
    }

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
    GimbalStateTxPacket gimbalStateTxPacket_;

    RefereeDataRxPacket refereeDataRxPacket_;
    GimbalLaunchStrategyRxPacket gimbalLaunchStrategyRxPacket_;
    ChassisStateRxPacket chassisStateRxPacket_;

    ChassisMsg_s cmsg_{};      // master keep it temporary
    ArmorLoaderMsg_s almsg_{}; // master keep it temporary

    FILTER::LPF outFilrPitch_{ PITCH_MOTOR_FREQ, 1000.f };
    FILTER::LPF outFilrYaw_{ YAW_MOTOR_FREQ, 1000.f };

    float calcPitchTorq(float _ang, float _vel, float _acc);
    float calcYawTorq(float _vel, float _acc);
};

} // namespace GIMBAL
