#pragma once
#include "Chassis.hpp"

#include "WheelLegged.hpp"

#include "WLController.hpp"
#include "WBRController.hpp"
#include "STController.hpp"

#include "Detector.hpp"

#include "Axis1Odom.hpp"

#include "LPF.hpp"
#include "IIR.hpp"

#include "RxPacket.hpp"
#include "PacketImpl.hpp"

namespace CHASSIS {
union Motors_u {
    struct {
        PINYMOTOR::IMotor *WheelR;
        PINYMOTOR::IMotor *HipRF;
        PINYMOTOR::IMotor *HipRB;
        PINYMOTOR::IMotor *WheelL;
        PINYMOTOR::IMotor *HipLF;
        PINYMOTOR::IMotor *HipLB;
    };
    PINYMOTOR::IMotor *_[6];
};

class ChassisCtrlRxPacket : public COMM::RxPacket<COMM::ChassisCtrlPacketType, 1> {
public:
    // TODO: slightly cumbersome like this or CRTP
    ChassisCtrlRxPacket() { registerCallback(); }
    void registerCallback() final;
};

class Bipedal : public Chassis {
    static constexpr float LF_HIP_ZERO_ANG = 0.531173289f;
    static constexpr float LB_HIP_ZERO_ANG = 2.17838573f;
    static constexpr float RF_HIP_ZERO_ANG = 4.08025455f;
    static constexpr float RB_HIP_ZERO_ANG = 3.65915108f;

    static constexpr float MASS_BODY = 16.245f;

    static constexpr float MAX_ABS_ROLL = 0.3f; // rad

    static constexpr float W_RADIUS = 0.11704f / 2.f; // m
    static constexpr float W_DIAMETER = W_RADIUS * 2.f;
    static constexpr float W_CIRCUMFERENCE = (PI * W_DIAMETER);

    static constexpr float W_DISTANCE = 0.4611f / 2.f; // half of the left-right wheelbase

    static constexpr float MAX_S = 2.f;

    static constexpr float CTRL_FREQ = 1000.f;

public:
    Bipedal();

    void stop();
    void enter();
    void update(void *_param);

    PINYMOTOR::IMotor *getHipLB() { return motors_.HipLB; }
    PINYMOTOR::IMotor *getHipRB() { return motors_.HipRB; }
    PINYMOTOR::IMotor *getHipLF() { return motors_.HipLF; }
    PINYMOTOR::IMotor *getHipRF() { return motors_.HipRF; }
    PINYMOTOR::IMotor *getWheelL() { return motors_.WheelL; }
    PINYMOTOR::IMotor *getWheelR() { return motors_.WheelR; }

    void enableJump();
    void disbaleJump();

    bool isGimbalReset() const;

    void setYawZero(float _yawZero);

    void setRoll(float _roll);

    void setLengthSmooth(float _l, float _step);
    void setLengthSmooth(float _lL, float _lR, float _step);
    void setLengthSmoothL(float _l, float _step);
    void setLengthSmoothR(float _l, float _step);

    void setLength(float _l);
    void setLength(float _lL, float _lR);
    void setLengthL(float _l);
    void setLengthR(float _l);

    void setFdForward();
    void setFdForward(float _fd);
    void setFdForward(float _fdL, float _fdR);

    void setHipPhiRot(float _dphi);
    void setHipPhiRot(float _dphiL, float _dphiR);
    void setHipPhiRotL(float _dphi);
    void setHipPhiRotR(float _dphi);

    void setHipPhiSmooth(float _phi, float _step);
    void setHipPhiSmooth(float _phiL, float _phiR, float _step);
    void setHipPhiSmoothL(float _phi, float _step);
    void setHipPhiSmoothR(float _phi, float _step);

    void setHipPhi(float _phi);
    void setHipPhi(float _phiL, float _phiR);
    void setHipPhiL(float _phi);
    void setHipPhiR(float _phi);

    void setHipTp(float _tp);
    void setHipTp(float _tpL, float _tpR);

    void bal(const WLController::States_u &_ref, float _yaw);
    void bal(const WLController::States_u &_refL, const WLController::States_u &_refR, float _yaw);

    void bal(const WBRController::States_u &_ref);

    void bal(const STController::States_u &_ref, float _yaw);

    void noBal(float _vx, float _wz);

    void ctrl();

    bool isTumble();
    bool isFreeze();
    bool isOverturn();
    bool detectPitch(float _ang1, float _ang2);
    bool detectPhiL(float _ang1, float _ang2);
    bool detectPhiR(float _ang1, float _ang2);
    bool detectLength(float _l, float _threthold = 0.05f);
    bool detectLengthL(float _l, float _threthold = 0.05f);
    bool detectLengthR(float _l, float _threthold = 0.05f);

    float yaw() const;
    bool isFloating() const;

    float phiL() const;
    float phiR() const;
    float lengthL() const;
    float lengthR() const;
    float vel() const; // m/s

    float calcFdForwardL();
    float calcFdForwardR();

    float deltaYaw = 0.f;

private:
    Motors_u motors_;

    WheelLegged legL_;
    WheelLegged legR_;

    Subscriber<INS_SYS::INSData_s> *insSub_;
    INS_SYS::INSData_s insData_ = {};

#if !APP_USE_COMM
    GimbalYawMsg_s gimbalYawMsg_;
    Subscriber<GimbalYawMsg_s> *gimbalYawSub_;
#endif
#ifdef TARGET_SLAVE
    ChassisCtrlRxPacket chassisCtrlRxPacket_;
#endif

    Axis1Odom odom_;

    Detector detector_;

    PositionalPid rollAngPid_{ 0.1f, 0.f, 0.f, 0.001f, 0.f, 1.f, 0.f };
    PositionalPid rollVelPid_{ 0.001f, 0.f, 0.f, 0.001f, 0.1f, 0.05f, 0.f };

    PositionalPid hipAngPidL_{ 15.f, 1.f, 1.f, 0.001f, 1.f, 2.5f, 0.f };
    PositionalPid hipVelPidL_{ 10.f, 0.f, 0.f, 0.001f, 0, 40.f, 0.f };

    PositionalPid hipAngPidR_{ hipAngPidL_ };
    PositionalPid hipVelPidR_{ hipVelPidL_ };

    PositionalPid yawPid_[2]{ { 0.f, 0.f, 0.f, 0.001f, 0.f, 0.f, 0.f }, { 0.f, 0.f, 0.f, 0.001f, 0.f, 0.f, 0.f } };
    IncrementalPid dsPid_[2]{ { 0.6f, 0.001f, 0.f, 3.f, 0.f }, { dsPid_[0] } };

    FILTER::IIR3 wheelFltr[2];

    float FdForward_ = MASS_BODY * WheelLegged::GRAVITY;

    float setRoll_ = 0.f;
    float lengthOffset_ = 0.f;

    float yawZero_ = 0.f; // used to sync with gimbal

    float yaw_ = 0.f;

    float ds_ = 0.f;
    float s_ = 0.f;
    float sForward_ = -0.2009f;

    float wz_ = 0.f;
    float wx_ = 0.f;
    float az_ = 0.f;
    float as_ = 0.f;

    float dt_ = 0.001f;
    uint32_t dtCnt_ = 0;

    void loadDeltaYaw();
    void updateVel();

    float cosYaw_ = 0.f;
    float sinYaw_ = 0.f;
};

} // namespace CHASSIS
