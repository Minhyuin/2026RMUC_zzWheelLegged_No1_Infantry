#pragma once
#include "Chassis.hpp"

#include "WheelLegged.hpp"

#include "WLController.hpp"
#include "WBRController.hpp"
#include "STController.hpp"

#include "Detector.hpp"

#include "Axis1Odom.hpp"

#include "LPF.hpp"

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

class ChassisCtrlRxPacket : public COMM::RxPacket<COMM::SamplePacketType, 1> {
public:
    // TODO: slightly cumbersome like this or CRTP
    ChassisCtrlRxPacket() { registerCallback(); }
    void registerCallback() final;
};

class Bipedal : public Chassis {
    static constexpr float LF_HIP_ZERO_ANG = 1.93523502f;
    static constexpr float LB_HIP_ZERO_ANG = 4.08178854f;
    static constexpr float RF_HIP_ZERO_ANG = 0.617081642f;
    static constexpr float RB_HIP_ZERO_ANG = 4.168846371f;

    static constexpr float MAX_ABS_ROLL = 0.3f; // rad

    static constexpr float W_RADIUS = 0.1153f / 2.f; // m
    static constexpr float W_DIAMETER = W_RADIUS * 2.f;
    static constexpr float W_CIRCUMFERENCE = (PI * W_DIAMETER);

    static constexpr float B_RADIUS = 0.5695f / 2.f;

    static constexpr float MAX_S = 0.5f;

public:
    Bipedal();

    void stop();
    void enter();
    void update(void *_param);

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

    void setHipFd(float _fd);
    void setHipFd(float _fdL, float _fdR);

    void bal(const WLController::States_u &_ref, float _wz);
    void bal(const WLController::States_u &_refL,
             const WLController::States_u &_refR, float _wz);

    void bal(const WBRController::States_u &_ref);

    void bal(const STController::States_u &_ref, float _wz);

    void noBal(float _vx, float _wz);


    void ctrl();

    bool isTumble();
    bool isFreeze();
    bool isOverturn();
    bool isLaunched(float _setPhi, float _phiTh, float _setL, float _lTh);
    bool detectPhiL(float _ang1, float _ang2);
    bool detectPhiR(float _ang1, float _ang2);

    float yaw() const;
    bool isFloating() const;

    float phiL() const;
    float phiR() const;
    float lengthL() const;
    float lengthR() const;

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
#ifdef APP_CHASSIS_SLAVE
    ChassisCtrlRxPacket chassisCtrlRxPacket_;
#endif

    Axis1Odom odom_;

    Detector detector_;

    PositionalPid rollAngPid_{ 0, 0, 0, 0, 0, 0, 0 };
    PositionalPid rollVelPid_{ 0, 0, 0, 0, 0, 0, 0 };

    PositionalPid hipAngPidL_{ 250.f, 1400.f, 0, 0.001f, 10.f, 40, 0 };
    PositionalPid hipVelPidL_{ 1.5f, 0, 0, 0.001f, 0, 40.f, 0 };

    PositionalPid hipAngPidR_{ hipAngPidL_ };
    PositionalPid hipVelPidR_{ hipVelPidL_ };

    IncrementalPid wzVelPid_{ 1, 0, 0, 3.f, 0 };
    IncrementalPid dsPid_[2]{ { 0.6f, 0.001f, 0.f, 3.f, 0.f }, { dsPid_[0] } };

    FILTER::LPF tpLPF_[2]{ { 1000.f, 10.f }, { tpLPF_[0] } };
    FILTER::LPF twLPF_[2]{ { 1000.f, 10.f }, { twLPF_[0] } };

    float setRoll_ = 0.f;

    float yawZero_ = 0.f; // used to sync with gimbal

    float yaw_ = 0.f;
    float ds_ = 0.f;
    float s_ = 0.f;
    float wz_ = 0.f;
    float wx_ = 0.f;
    float az_ = 0.f;
    float as_ = 0.f;

    float dt_ = 0.001f;
    uint32_t dtCnt_ = 0;
};

} // namespace CHASSIS
