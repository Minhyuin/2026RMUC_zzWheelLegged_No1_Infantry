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
#include "./State/TankState.hpp"
#include "./State/SpinState.hpp"
#include "./State/SpinExitingState.hpp"
#include "./State/ClimbState.hpp"
#include "./State/ClimbPrepareState.hpp"
#include "./State/JumpState.hpp"
#include "./State/TestState.hpp"

#if EXTENSION_REFEREE
#include "Referee.hpp"
#if APP_USE_UI
#include "UIApp.hpp"
#endif
#endif

#if EXTENSION_BUZZER
#include "Buzzer.hpp"
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

void GimbalStateRxPacket::registerCallback()
{
    Can::instance().registerCallback(&COMM_HANDLE, uid(), [this](const uint8_t *_rxBuf) {
        xQueueSendFromISR(this->queue_, _rxBuf, nullptr);
    });
    LOG::info("GimbalStateRxPacket", " Receive cb registered, uId:0x%hx \n", uid());
}

void RefereeDataTxPacket::transmit(uint8_t *_buf, uint16_t _len)
{
    Can::instance().transmitData(&COMM_HANDLE, uid(), _buf, _len);
}

void GimbalLaunchStrategyTxPacket::transmit(uint8_t *_buf, uint16_t _len)
{
    Can::instance().transmitData(&COMM_HANDLE, uid(), _buf, _len);
}

void ChassisStateTxPacket::transmit(uint8_t *_buf, uint16_t _len)
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
    this->stateFactory_.addState(FSMState_e::TANK, std::make_unique<TankState>(this));
    this->stateFactory_.addState(FSMState_e::SPIN, std::make_unique<SpinState>(this));
    this->stateFactory_.addState(FSMState_e::SPIN_EXITING, std::make_unique<SpinExitingState>(this));
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
#if EXTENSION_BUZZER
    BUZZER::buzz->playNote(BUZZER::Tone_e::G5, 100);
#endif

    legL_.enter();
    legR_.enter();
}

void Bipedal::update(void *_param)
{
    // if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &msg, 0) == pdTRUE) {
    // };
    UNUSED(_param);
    msg.noChassis = chassisCtrlRxPacket_.data().noChassis;
    msg.state = msg.noChassis ? FSMState_e::STOP :
                                static_cast<FSMState_e>(chassisCtrlRxPacket_.data().chassisExptState);
    msg.vx = chassisCtrlRxPacket_.data().vx;
    msg.vy = chassisCtrlRxPacket_.data().vy;
    msg.dl = chassisCtrlRxPacket_.data().dl;
    msg.droll = chassisCtrlRxPacket_.data().droll;
    msg.needJump = chassisCtrlRxPacket_.data().needJump;
    msg.needClimb = chassisCtrlRxPacket_.data().needClimb;
    msg.needStretch = chassisCtrlRxPacket_.data().needStretch;
    msg.needRush = chassisCtrlRxPacket_.data().needRush;
    msg.uiReset = chassisCtrlRxPacket_.data().uiReset;

    if (motors_.HipLF->globalState != GlobalState_e::ONLINE || motors_.HipLB->globalState != GlobalState_e::ONLINE ||
        motors_.HipRB->globalState != GlobalState_e::ONLINE || motors_.HipRF->globalState != GlobalState_e::ONLINE ||
        motors_.WheelL->globalState != GlobalState_e::ONLINE || motors_.WheelR->globalState != GlobalState_e::ONLINE) {
        msg.state = FSMState_e::STOP;
    }

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

    // set tp feedforward
    antiSplitTp_ = antiSplitPid_.calc(phiGlobalL() - phiGlobalR(), 0); // TpL - offset, TpR + offset
    // Approximate leg gravity torque around hip: tau = m * g * (L/2) * sin(phi_global).
    float tpGravityCompL =
            WheelLegged::MASS * WheelLegged::GRAVITY * (0.5f * legL_.state().L0) * arm_sin_f32(phiGlobalL());
    float tpGravityCompR =
            WheelLegged::MASS * WheelLegged::GRAVITY * (0.5f * legR_.state().L0) * arm_sin_f32(phiGlobalR());
    legL_.setTpFeedforward(tpGravityCompL);
    legR_.setTpFeedforward(tpGravityCompR);

    // hip cap
#if EXTENSION_SUPERCAP
    hipCap_.enable();
    hipCap_.setChargePowerLimit(10);
    hipCap_.setChargeEnergySlack(60);
    hipCap_.setChargelimitRatio(0.75f);
    hipCap_.task();
#endif

    // wheel cap
#if EXTENSION_SUPERCAP
#if EXTENSION_REFEREE
    const auto &refereeData = referee->receiver.getRefereeData();
    auto isAlive = refereeData.gameRobotStatus.currentHP > 0;
    powerLimit_ = std::clamp(static_cast<float>(refereeData.gameRobotStatus.chassisPowerLimit), 50.f, 120.f);
    wheelCap_.setChargeEnergySlack(refereeData.powerHeatData.chassisPowerBuffer);
    if (isAlive)
        wheelCap_.enable();
    else
        wheelCap_.disable();
#else
    wheelCap_.enable(); // default enable
    wheelCap_.setChargeEnergySlack(60);
#endif
    wheelCap_.setChargePowerLimit(static_cast<uint16_t>(powerLimit_));
    wheelCap_.task();
#endif

    // limit spd and acc
    updatePowerLimit();

    // send referee data to gimbal
#if EXTENSION_REFEREE
    refereeDataTxPacket_.setData().color = refereeData.gameRobotStatus.robotId >= 101 ? 1 : 0;
    refereeDataTxPacket_.setData().isPlay = refereeData.gameStatus.gameProgress == 4;
    refereeDataTxPacket_.setData().isAlive = isAlive;
    refereeDataTxPacket_.setData().stageRemainTime = refereeData.gameStatus.stageRemainTime;
    refereeDataTxPacket_.setData().rateOfFire = refereeData.shootData.bulletFreq;
    refereeDataTxPacket_.setData().muzzleVel = refereeData.shootData.bulletSpeed;
    // UI
#if APP_USE_UI
    if (msg.uiReset) {
        UI::APP::instance().uiReInit();
    }
    ChassisUIMsg_s chassisUIMsgData{
        .state = stateFactory_.getCurrentState()->getName(),
        .deltaYaw = deltaYaw,
        .lengthL = lengthL(),
        .lengthR = lengthR(),
        .roll = insData_.roll,
        .needRush = msg.needRush,
        .noChassis = msg.noChassis,
#if EXTENSION_SUPERCAP
        .capEnergyRatio = wheelCap_.getCapData().capEnergyRatio,
#else
        .capEnergyRatio = 0.99f,
#endif
    };
    UI::APP::instance().updateChassis(chassisUIMsgData);

    GimbalUIMsg_s gimbalUIMsgData{
        .state = static_cast<GIMBAL::FSMState_e>(gimbalStateRxPacket_.data().gimbalRealState),
        .aiming = gimbalStateRxPacket_.data().aiming,
        .visionOnline = gimbalStateRxPacket_.data().visionOnline,
        .targetMode = gimbalStateRxPacket_.data().targetMode,
        .pitchAng = getMinorArc(gimbalStateRxPacket_.data().pitch, pitch()),
        .pitchTargetOffsetAng = gimbalStateRxPacket_.data().targetPitchOffset,
        .needClimb = msg.needClimb,
    };
    UI::APP::instance().updateGimbal(gimbalUIMsgData);
#endif
#endif

    // send state to gimbal
    chassisStateTxPacket_.setData().chassisRealState = static_cast<uint8_t>(msg.state);
    chassisStateTxPacket_.setData().weightlessness = isFloating();
    chassisStateTxPacket_.setData().wz = wz();

    // stop detect
    detectStop();

    // update state
    this->stateFactory_.update();
}

void Bipedal::loadDeltaYaw()
{
    // Master Board method
    if (insSub_->receive())
        insData_.pitch = insData_.pitch - REAL_Q_OFFSET;
    deltaYaw = getMinorArc(chassisCtrlRxPacket_.data().deltaYaw, 0.f);

    cosYaw = arm_cos_f32(deltaYaw);
    sinYaw = arm_sin_f32(deltaYaw);
};

void Bipedal::ctrl()
{
    legL_.ctrl();
    legR_.ctrl();
}
