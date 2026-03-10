#pragma once
#include "Chassis.hpp"

#include "WheelLegged.hpp"

#include "WLController.hpp"
#include "WBRController.hpp"
#include "STController.hpp"

#include "Axis1Odom.hpp"

#include "LPF.hpp"
#include "IIR.hpp"

#include "RxPacket.hpp"
#include "TxPacket.hpp"
#include "PacketImpl.hpp"

namespace CHASSIS {
struct Motors_s {
    PINYMOTOR::IMotor *WheelR;
    PINYMOTOR::IMotor *HipRF;
    PINYMOTOR::IMotor *HipRB;
    PINYMOTOR::IMotor *WheelL;
    PINYMOTOR::IMotor *HipLF;
    PINYMOTOR::IMotor *HipLB;
};

class ChassisCtrlRxPacket : public COMM::RxPacket<COMM::ChassisCtrlPacketType, 1> {
public:
    // TODO: slightly cumbersome like this or CRTP
    ChassisCtrlRxPacket() { registerCallback(); }
    void registerCallback() final;
};

class RefereeDataTxPacket : public COMM::TxPacket<COMM::RefereePacketType> {
    friend class Bipedal;
    RefereeDataTxPacket() : TxPacket(10.f) {}
    void transmit(uint8_t *_buf, uint16_t _len) final;
};

class GimbalLaunchStrategyTxPacket : public COMM::TxPacket<COMM::GimbalLaunchStrategyPacketType> {
    friend class Bipedal;
    GimbalLaunchStrategyTxPacket() : TxPacket(10.f) {}
    void transmit(uint8_t *_buf, uint16_t _len) final;
};

class Bipedal : public Chassis {
    static constexpr float LF_HIP_CALIBRATED_ANG = 2.860530f;
    static constexpr float LB_HIP_CALIBRATED_ANG = 4.403488f;
    static constexpr float RF_HIP_CALIBRATED_ANG = 5.665203f;
    static constexpr float RB_HIP_CALIBRATED_ANG = 3.005563f;

    static constexpr float MASS_BODY = 22.6110f;

    static constexpr float MAX_ABS_ROLL = 0.3f; // maximum offset of roll angle, unit: rad

    static constexpr float W_DIAMETER = 0.11704f; // wheel diameter, unit: m
    static constexpr float W_RADIUS = W_DIAMETER / 2.f;
    static constexpr float W_CIRCUMFERENCE = (PI * W_DIAMETER);

    static constexpr float W_DISTANCE = 0.4611f / 2.f; // half of the left-right wheelbase

    static constexpr float MAX_S = 2.5f; // maximum displacement allowed for credit accumulation

    static constexpr float CTRL_FREQ = 1000.f;

    static constexpr float S_FEEDFORWARD_BACKWARD =
            0.0f; // feedfoward gain for s when the direction of movement is towards the knee joint
    static constexpr float S_FEEDFORWARD_FORWARD =
            -0.0f; // feedfoward gain for s when the direction of movement is opposite to the knee joint

public:
    static constexpr float DEFAULT_L = 0.2f; // the length for all feedforward param

    Bipedal();

    /**
     * @brief close all outputs
     */
    void stop();
    /**
     * @brief enable all outputs
     */
    void enter();
    /**
     * @brief update Bipedal chassis state
     * 
     * @param _param from context, default nullptr
     */
    void update(void *_param);

    /**
     * @brief turn on the jump flag
     */
    void enableJump();
    /**
     * @brief turn off the jump flag
     */
    void disableJump();

    /**
     * @brief turn on roll control
     */
    void enableRoll();

    /**
     * @brief turn off roll control
     */
    void disableRoll();

    /**
     * @brief check if the gimbal is resetting (i.e., the pitch or yaw angle is in a certain range)
     * @note the message is sent by the master board, which detects the state with its RobotModules/Gimbal.
     * @return true 
     * @return false 
     */
    bool isGimbalReset() const;

    /**
     * @brief force approaching overwrite current state "s", used to accelerate model convergence
     * 
     * @param _s overwrite value for "s", unit: m
     * @param _acc approaching acceleration, unit: {_s unit}/ms
     */
    void feedS(float _s, float _acc = 10.f);
    /**
     * @brief force keep the state "s", not affected by the floating detection 
     * 
     * @param _durationMs 
     */
    void lockS(uint16_t _durationMs);

    /**
     * @brief set the expected roll angle 
     * 
     * @param _roll unit: rad
     */
    void setRoll(float _roll);

    /**
     * @brief set the expected length smoothly
     * 
     * @param _l unit: m
     * @param _step unit: m/ms
     */
    void setLengthSmooth(float _l, float _step);
    void setLengthSmooth(float _lL, float _lR, float _step);
    void setLengthSmoothL(float _l, float _step);
    void setLengthSmoothR(float _l, float _step);

    /**
     * @brief set the expected length
     * 
     * @param _l unit: m
     */
    void setLength(float _l);
    void setLength(float _lL, float _lR);
    void setLengthL(float _l);
    void setLengthR(float _l);
    /**
     * @brief override the length PIDs with custom parameters
     */
    void setLengthPID(const PositionalPid &_angPID, const PositionalPid &_velPID);

    /**
     * @brief Set the Fd feedforward
     * @note default feedforward is calculated with the expected length and the body weight
     * @todo maybe this is a redundant function, not sure yet
     */
    void setFdFeedforward();
    void setFdFeedforward(float _fd);
    void setFdFeedforward(float _fdL, float _fdR);

    /**
     * @brief set the expected hip angular velocity by keeping adding an offset to the current angle
     * 
     * @param _dphi unit: rad/s
     */
    void setHipPhiRot(float _dphi);
    void setHipPhiRot(float _dphiL, float _dphiR);
    void setHipPhiRotL(float _dphi);
    void setHipPhiRotR(float _dphi);

    /**
     * @brief set the expected hip angle
     * 
     * @param _phi unit: rad
     */
    void setHipPhi(float _phi);
    void setHipPhi(float _phiL, float _phiR);
    void setHipPhiL(float _phi);
    void setHipPhiR(float _phi);

    /**
     * @brief set the expected hip angle in the global frame
     * 
     * @param _phi unit: rad
     */
    void setHipPhiGlobal(float _phi);
    void setHipPhiGlobal(float _phiL, float _phiR);
    void setHipPhiGlobalL(float _phi);
    void setHipPhiGlobalR(float _phi);

    /**
     * @brief set the expected Tp
     * 
     * @param _tp unit: Nm
     */
    void setHipTp(float _tp);
    void setHipTp(float _tpL, float _tpR);

    /**
     * @brief base on WL Model, balance the chassis with the given reference state and yaw
     * 
     * @param _ref 6D reference state in WL model
     * @param _yaw should be clamped in [-PI, PI]
     */
    void bal(const WLController::States_s &_ref, float _yaw);
    void bal(const WLController::States_s &_refL, const WLController::States_s &_refR, float _yaw);

    /**
     * @brief base on WBR Model, balance the chassis with the given reference state
     * 
     * @param _ref 10D reference state in WBR model
     */
    void bal(const WBRController::States_s &_ref);

    /**
     * @brief base on ST Model, balance the chassis with the given reference state and yaw
     * 
     * @param _ref 4D reference state in ST model
     * @param _yaw should be clamped in [-PI, PI]
     */
    void bal(const STController::States_s &_ref, float _yaw);

    /**
     * @brief disable balancing and set the expected velocity and angular velocity
     * 
     * @param _vx unit: m/s
     * @param _wz unit: rad/s
     */
    void noBal(float _vx, float _wz);

    void ctrl();

    // Observe functions
    /**
     * @brief check if the chassis is in a tumble state
     */
    bool isTumble() const;
    /**
     * @brief check if the chassis is no longer moving
     */
    bool isFreeze() const;
    /**
     * @brief check if the chassis is in an overturn state
     */
    bool isOverturn() const;
    /**
     * @brief check if the chassis is in a floating state
     */
    bool isFloating() const;
    /**
     * @brief calculate and update the floating flag of each leg
     */
    bool detectFloating();
    /**
     * @brief judge whether to stop based on the expected speed
     */
    bool detectNeedStop(float _expVel);
    /**
     * @brief base on the function detectAng to judge whether the pitch angle is in the target range
     * 
     * @param _endAng end angle (counter-clockwise relative to start angle), unit: rad
     * @param _startAng start angle, unit: rad
     */
    bool detectPitchRange(float _endAng, float _startAng) const;
    /**
     * @brief base on the function detectAng to judge whether the left leg angle is in the target range
     * 
     * @param _endAng end angle (counter-clockwise relative to start angle), unit: rad
     * @param _startAng start angle, unit: rad
     */
    bool detectPhiLRange(float _endAng, float _startAng) const;
    /**
     * @brief base on the function detectAng to judge whether the right leg angle is in the target range
     * 
     * @param _endAng end angle (counter-clockwise relative to start angle), unit: rad
     * @param _startAng start angle, unit: rad
     */
    bool detectPhiRRange(float _endAng, float _startAng) const;
    /**
     * @brief base on the function detectAng to judge whether the global left leg angle is in the target range
     * 
     * @param _endAng end angle (counter-clockwise relative to start angle), unit: rad
     * @param _startAng start angle, unit: rad
     */
    bool detectPhiGlobalLRange(float _endAng, float _startAng) const;
    /**
     * @brief base on the function detectAng to judge whether the global right leg angle is in the target range
     * 
     * @param _endAng end angle (counter-clockwise relative to start angle), unit: rad
     * @param _startAng start angle, unit: rad
     */
    bool detectPhiGlobalRRange(float _endAng, float _startAng) const;
    /**
     * @brief judge whether the leg length is in the target range
     * 
     * @param _l  target length, unit: m
     * @param _threthold deadband for length, unit: m
     */
    bool detectLengthRange(float _l, float _threthold = 0.05f) const;
    bool detectLengthLRange(float _l, float _threthold = 0.05f) const;
    bool detectLengthRRange(float _l, float _threthold = 0.05f) const;

    float yaw() const;
    float pitch() const;
    float roll() const;

    float phiL() const;
    float phiR() const;
    float phiGlobalL() const;
    float phiGlobalR() const;
    float lengthL() const;
    float lengthR() const;
    float length() const;
    float vel() const; // m/s

    float deltaYaw = 0.f;
    float cosYaw = 0.f;
    float sinYaw = 0.f;

private:
    Motors_s motors_;

    WheelLegged legL_;
    WheelLegged legR_;

    Subscriber<INS_SYS::INSData_s> *insSub_;
    INS_SYS::INSData_s insData_ = {};

#if !APP_USE_COMM
    GimbalYawMsg_s gimbalYawMsg_;
    Subscriber<GimbalYawMsg_s> *gimbalYawSub_;
#endif

    ChassisCtrlRxPacket chassisCtrlRxPacket_;

    RefereeDataTxPacket refereeDataTxPacket_;
    GimbalLaunchStrategyTxPacket gimbalLaunchStrategyTxPacket_;

    Axis1Odom odom_;

    PositionalPid rollAngPid_{ 90.f, 120.f, 0.1f, 0.001f, 3.f, 15.f, 0.f };
    PositionalPid rollVelPid_{ 0.01f, 0.f, 0.f, 0.001f, 0.f, 0.2f, 0.f };

    PositionalPid hipAngPidL_{ 60.f, 0.f, 1.5f, 0.001f, 0.f, 4.3f, 0.f };
    PositionalPid hipVelPidL_{ 15.f, 0.f, 0.f, 0.001f, 0, 120.f, 0.f };

    PositionalPid hipAngPidR_{ hipAngPidL_ };
    PositionalPid hipVelPidR_{ hipVelPidL_ };

    PositionalPid antiSplitPid_{ 100.f, 0.f, 5.f, 0.001f, 0.f, 50.f, 0.f };
    float antiSplitTp_ = 0;

    PositionalPid yawPid_[2]{ { 0.f, 0.f, 0.f, 0.001f, 0.f, 0.f, 0.f }, { 0.f, 0.f, 0.f, 0.001f, 0.f, 0.f, 0.f } };
    PositionalPid dsPid_[2]{ { 0.6f, 0.f, 0.f, 0.001f, 0.f, 3.f, 0.f }, { dsPid_[0] } };

    float setRoll_ = 0.f;
    float lengthOffset_ = 0.f;
    bool rollControlFlag = false;

    float yawZero_ = 0.f; // used to sync with gimbal

    float yaw_ = 0.f;

    float ds_ = 0.f;
    float s_ = 0.f;
    uint16_t lockSCount_ = 0;
    float dsCmdBeforeStop_ = 0.f;

    float wz_ = 0.f;
    float wx_ = 0.f;
    float az_ = 0.f;
    float as_ = 0.f;

    float azBody_ = 0.f;
    FILTER::LPF azBodyFilter_{ 1000.f, 1.f };

    float dt_ = 0.001f;
    uint32_t dtCnt_ = 0;

    void loadDeltaYaw();

    // Observe functions
    void updateBody();

    void updateGimbalLaunchStrategy();

    /**
     * @brief judge body is in the low displacement state
     */
    void detectStop();

    /**
     * @brief detect if an angle is in the target range
     * 
     * @param _input input angle
     * @param _startAng start angle
     * @param _endAng end angle (counter-clockwise relative to start angle)
     */
    bool detectAng(float _input, float _startAng, float _endAng) const;

    float phiGlobalL_ = 0.f;
    float phiGlobalR_ = 0.f;
};

} // namespace CHASSIS
