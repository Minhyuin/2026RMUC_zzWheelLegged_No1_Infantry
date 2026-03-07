#include "Bipedal.hpp"
#include "M3508.hpp"
#include "DM8009.hpp"
#include "TopicRouter.hpp"
#include "dsp/fast_math_functions.h"
#include "sdkconfig.h"
#include "Bsp_can.hpp"
#include "Dwt.hpp"
#include "Smooth.hpp"

#include "MotorCommonMacros.hpp"

#include "./State/StopState.hpp"
#include "./State/RescueState.hpp"
#include "./State/LaunchState.hpp"
#include "./State/StandState.hpp"
#include "./State/IdleState.hpp"
#include "./State/SpinState.hpp"
#include "./State/ClimbState.hpp"
#include "./State/ClimbPrepareState.hpp"
#include "./State/JumpState.hpp"
#include "./State/TestState.hpp"

#if APP_USE_REFEREE
#include "Referee.hpp"
#endif

#define HIP_HANDLE   HCAN2
#define WHEEL_HANDLE HCAN1
#define COMM_HANDLE  HCAN3

using namespace CHASSIS;
using namespace PINYMOTOR;

void ChassisCtrlRxPacket::registerCallback()
{
    Can::instance().registerCallback(&COMM_HANDLE, uid(), [this](const uint8_t *_rxBuf) {
        xQueueSendFromISR(this->queue_, _rxBuf, nullptr);
    });
    LOG::info("ChassisCtrlRxPacket", " Receive cb registered, uId:0x%hx \n", uid());
}

void RefereeDataTxPacket::transmit(uint8_t *_buf, uint16_t _len)
{
    Can::instance().transmitData(&COMM_HANDLE, uid(), _buf, _len);
}

Bipedal::Bipedal() : insSub_(new Subscriber<INS_SYS::INSData_s>(2, &TopicRouter::instance().insTopic, &insData_))
{
    InitConfig_s rfDM8009Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HIP_HANDLE),
        .comType = ComType_e::FDCAN,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 1,
        .txFreq = CTRL_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = true,
    };
    motors_.HipRF = new DMMOTOR::DM8009("HipRF", rfDM8009Config); // 右HIP4控制电机
    motors_.HipRF->setZeroAng(RF_HIP_CALIBRATED_ANG);

    InitConfig_s lfDM8009Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HIP_HANDLE),
        .comType = ComType_e::FDCAN,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 2,
        .txFreq = CTRL_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = false,
    };
    motors_.HipLF = new DMMOTOR::DM8009("HipLF", lfDM8009Config); // 左HIP4控制电机
    motors_.HipLF->setZeroAng(LF_HIP_CALIBRATED_ANG);

    InitConfig_s lbDM8009Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HIP_HANDLE),
        .comType = ComType_e::FDCAN,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 3,
        .txFreq = CTRL_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = false,
    };
    motors_.HipLB = new DMMOTOR::DM8009("HipLB", lbDM8009Config); // 左HIP1控制电机
    motors_.HipLB->setZeroAng(rangeMap(LB_HIP_CALIBRATED_ANG - PI));

    InitConfig_s rbDM8009Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HIP_HANDLE),
        .comType = ComType_e::FDCAN,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 4,
        .txFreq = CTRL_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = true,
    };
    motors_.HipRB = new DMMOTOR::DM8009("HipRB", rbDM8009Config); // 右HIP1控制电机
    motors_.HipRB->setZeroAng(rangeMap(RB_HIP_CALIBRATED_ANG - PI));

    InitConfig_s lM3508Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&WHEEL_HANDLE),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 5,
        .txFreq = CTRL_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = true,
    };
    motors_.WheelL = new DJIMOTOR::M3508("WheelL", lM3508Config);
    motors_.WheelL->overrideReductionRatio(DJIMOTOR::M3508::XROLL_GEARBOX_RR);

    InitConfig_s rM3508Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&WHEEL_HANDLE),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 6,
        .txFreq = CTRL_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = false,
    };
    motors_.WheelR = new DJIMOTOR::M3508("WheelR", rM3508Config);
    motors_.WheelR->overrideReductionRatio(DJIMOTOR::M3508::XROLL_GEARBOX_RR);

    legL_.init(motors_.HipLB, motors_.HipLF, motors_.WheelL);
    legR_.init(motors_.HipRB, motors_.HipRF, motors_.WheelR);

    this->stateFactory_.addState(FSMState_e::STOP, std::make_unique<StopState>(this));
    this->stateFactory_.addState(FSMState_e::LAUNCH, std::make_unique<LaunchState>(this));
    this->stateFactory_.addState(FSMState_e::STAND, std::make_unique<StandState>(this));
    this->stateFactory_.addState(FSMState_e::RESCUE, std::make_unique<RescueState>(this));
    this->stateFactory_.addState(FSMState_e::IDLE, std::make_unique<IdleState>(this));
    this->stateFactory_.addState(FSMState_e::SPIN, std::make_unique<SpinState>(this));
    this->stateFactory_.addState(FSMState_e::CLIMB, std::make_unique<ClimbState>(this));
    this->stateFactory_.addState(FSMState_e::CLIMB_PREPARE, std::make_unique<ClimbPrepareState>(this));
    this->stateFactory_.addState(FSMState_e::JUMP, std::make_unique<JumpState>(this));
    this->stateFactory_.addState(FSMState_e::TEST, std::make_unique<TestState>(this));
    this->stateFactory_.init(this->stateFactory_.getNextState(FSMState_e::STOP));
}

void Bipedal::stop()
{
    legL_.stop();
    legR_.stop();

    rollAngPid_.reset();
    rollVelPid_.reset();

    hipAngPidL_.reset();
    hipVelPidL_.reset();
    hipAngPidR_.reset();
    hipVelPidR_.reset();

    yawPid_[0].reset();
    yawPid_[1].reset();

    dsPid_[0].reset();
    dsPid_[1].reset();

    s_ = 0;
    setRoll_ = 0;
    lengthOffset_ = 0;
}

void Bipedal::enter()
{
    legL_.enter();
    legR_.enter();
}

void Bipedal::update(void *_param)
{
    // if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &msg, 0) == pdTRUE) {
    // };
    UNUSED(_param);
    msg.state = static_cast<FSMState_e>(chassisCtrlRxPacket_.data().chassisExptState);
    msg.vx = chassisCtrlRxPacket_.data().vx;
    msg.vy = chassisCtrlRxPacket_.data().vy;

    // update dt
    dt_ = Dwt::instance().getDt(&dtCnt_);

    // update insData
    loadDeltaYaw();

    // update leg
    legL_.stateUpdate(dt_);
    legR_.stateUpdate(dt_);
    phiGlobalL_ = rangeMap(legL_.state().phi0 + insData_.pitch, -PI, PI);
    phiGlobalR_ = rangeMap(legR_.state().phi0 + insData_.pitch, -PI, PI);
    detectFloating();

    // update vel
    updateBody();

    // update roll
    if (rollControlFlag)
        lengthOffset_ = rollVelPid_.calc(rollAngPid_.calc(getMinorArc(setRoll_, insData_.roll), 0), wx_);
    else
        lengthOffset_ = 0.f;

    // stop detect
    detectStop();

// send referee data to gimbal
#if APP_USE_REFEREE
    const auto &refereeData = referee->receiver.getRefereeData();
    refereeDataTxPacket_.setData().color = refereeData.gameRobotStatus.robotId >= 101 ? 1 : 0;
    refereeDataTxPacket_.setData().isPlay = refereeData.gameStatus.gameProgress == 4;
    refereeDataTxPacket_.setData().isAlive = refereeData.gameRobotStatus.currentHP > 0;
    refereeDataTxPacket_.setData().stageRemainTime = refereeData.gameStatus.stageRemainTime;
    refereeDataTxPacket_.setData().rateOfFire = refereeData.shootData.bulletFreq;
    refereeDataTxPacket_.setData().muzzleVel = refereeData.shootData.bulletSpeed;
#endif
    // update state
    this->stateFactory_.update();
}

void Bipedal::loadDeltaYaw()
{
    // Master Board method
    insSub_->receive();
    deltaYaw = getMinorArc(chassisCtrlRxPacket_.data().deltaYaw, 0.f);

    cosYaw = arm_cos_f32(deltaYaw);
    sinYaw = arm_sin_f32(deltaYaw);
};

void Bipedal::ctrl()
{
    legL_.ctrl();
    legR_.ctrl();
}

void Bipedal::enableJump()
{
    legL_.needJumping = true;
    legR_.needJumping = true;
}

void Bipedal::disbaleJump()
{
    legL_.needJumping = false;
    legR_.needJumping = false;
}

void Bipedal::enableRoll() { rollControlFlag = true; }

void Bipedal::disableRoll() { rollControlFlag = false; }

void Bipedal::feedS(float _s, float _acc) { SMOOTH::slopeFollowing(&_s, &s_, _acc); }

void Bipedal::lockS(uint16_t _durationMs) { lockSCount_ = _durationMs; }

void Bipedal::setRoll(float _roll) { setRoll_ = std::clamp(_roll, -MAX_ABS_ROLL, MAX_ABS_ROLL); }

void Bipedal::setLengthSmooth(float _l, float _step) { setLengthSmooth(_l, _l, _step); }
void Bipedal::setLengthSmooth(float _lL, float _lR, float _step)
{
    float deltaL = _lL - legL_.state().L0;
    deltaL = clamp(deltaL, -_step, _step);
    float deltaR = _lR - legR_.state().L0;
    deltaR = clamp(deltaR, -_step, _step);
    setLength(legL_.state().L0 + deltaL, legR_.state().L0 + deltaR);
}
void Bipedal::setLengthSmoothL(float _l, float _step)
{
    float delta = _l - legL_.state().L0;
    delta = clamp(delta, -_step, _step);
    setLengthL(legL_.state().L0 + delta);
}
void Bipedal::setLengthSmoothR(float _l, float _step)
{
    float delta = _l - legR_.state().L0;
    delta = clamp(delta, -_step, _step);
    setLengthR(legR_.state().L0 + delta);
}

void Bipedal::setLength(float _l) { setLength(_l, _l); }

void Bipedal::setLength(float _lL, float _lR)
{
    setLengthL(_lL + lengthOffset_);
    setLengthR(_lR - lengthOffset_);
}

void Bipedal::setLengthL(float _l) { legL_.setL(_l); }

void Bipedal::setLengthR(float _l) { legR_.setL(_l); }

void Bipedal::setLengthPID(const PositionalPid &_angPID, const PositionalPid &_velPID)
{
    legL_.overrideLengthPid(_angPID, _velPID);
    legR_.overrideLengthPid(_angPID, _velPID);
}

void Bipedal::setFdFeedforward()
{
    float angL = phiGlobalL();
    bool flagL = getMinorArc(PI / 2.5f, angL) > 0 && getMinorArc(-PI / 2.5f, angL) < 0;
    float fdL = flagL ? MASS_BODY * WheelLegged::GRAVITY / 2.f * arm_cos_f32(angL) : 0.f;
    float angR = phiGlobalR();
    bool flagR = getMinorArc(PI / 2.5f, angR) > 0 && getMinorArc(-PI / 2.5f, angR) < 0;
    float fdR = flagR ? MASS_BODY * WheelLegged::GRAVITY / 2.f * arm_cos_f32(angR) : 0.f;
    setFdFeedforward(fdL, fdR);
}

void Bipedal::setFdFeedforward(float _fd) { setFdFeedforward(_fd, _fd); }

void Bipedal::setFdFeedforward(float _fdL, float _fdR)
{
    legL_.setFdFeedforward(_fdL);
    legR_.setFdFeedforward(_fdR);
}

void Bipedal::setHipPhiRot(float _dphi) { setHipPhiRot(_dphi, _dphi); }
void Bipedal::setHipPhiRot(float _dphiL, float _dphiR)
{
    setHipPhiRotL(_dphiL);
    setHipPhiRotR(_dphiR);
}

void Bipedal::setHipPhiRotL(float _dphi)
{
    if (_dphi == 0)
        hipAngPidL_.reset();
    setHipPhiL(legL_.state().phi0 + (_dphi * 0.001f));
}
void Bipedal::setHipPhiRotR(float _dphi)
{
    if (_dphi == 0)
        hipAngPidR_.reset();
    setHipPhiR(legR_.state().phi0 + (_dphi * 0.001f));
}

void Bipedal::setHipPhi(float _phi) { setHipPhi(_phi, _phi); }

void Bipedal::setHipPhi(float _phiL, float _phiR)
{
    setHipPhiL(_phiL);
    setHipPhiR(_phiR);
}

void Bipedal::setHipPhiL(float _phi)
{
    float hipVelL = hipAngPidL_.calc(getMinorArc(_phi, phiL()), 0);
    float tpL = hipVelPidL_.calc(hipVelL, legL_.state().dphi0);
    legL_.setTp(tpL);
}

void Bipedal::setHipPhiR(float _phi)
{
    float hipVelR = hipAngPidR_.calc(getMinorArc(_phi, phiR()), 0);
    float tpR = hipVelPidR_.calc(hipVelR, legR_.state().dphi0);
    legR_.setTp(tpR);
}

void Bipedal::setHipPhiGlobal(float _phi) { setHipPhiGlobal(_phi, _phi); }

void Bipedal::setHipPhiGlobal(float _phiL, float _phiR)
{
    setHipPhiGlobalL(_phiL);
    setHipPhiGlobalR(_phiR);
}

void Bipedal::setHipPhiGlobalL(float _phi)
{
    float hipVelL = hipAngPidL_.calc(getMinorArc(_phi, phiGlobalL()), 0);
    float tpL = hipVelPidL_.calc(hipVelL, legL_.state().dphi0 + insData_.body.gy);
    legL_.setTp(tpL);
}

void Bipedal::setHipPhiGlobalR(float _phi)
{
    float hipVelR = hipAngPidR_.calc(getMinorArc(_phi, phiGlobalR()), 0);
    float tpR = hipVelPidR_.calc(hipVelR, legR_.state().dphi0 + insData_.body.gy);
    legR_.setTp(tpR);
}

void Bipedal::setHipTp(float _tp) { setHipTp(_tp, _tp); }

void Bipedal::setHipTp(float _tpL, float _tpR)
{
    legL_.setTp(_tpL);
    legR_.setTp(_tpR);
}

float Bipedal::yaw() const { return yaw_; }

float Bipedal::pitch() const { return insData_.pitch; }

float Bipedal::roll() const { return insData_.roll; }

float Bipedal::phiL() const { return legL_.state().phi0; }

float Bipedal::phiR() const { return legR_.state().phi0; }

float Bipedal::phiGlobalL() const { return phiGlobalL_; }

float Bipedal::phiGlobalR() const { return phiGlobalR_; }

float Bipedal::lengthL() const { return legL_.state().L0; }

float Bipedal::lengthR() const { return legR_.state().L0; }

float Bipedal::length() const { return (legL_.state().L0 + legR_.state().L0) / 2.f; }

float Bipedal::vel() const { return ds_; }
