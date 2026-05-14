#include "Standard.hpp"

#include "Pid.hpp"

#include "DM4310.hpp"

#include "Bsp_can.hpp"

#include "./State/StopState.hpp"
#include "./State/LaunchState.hpp"
#include "./State/RunState.hpp"
#include "./State/AutoState.hpp"
#include "./State/UTurnState.hpp"

#if EXTENSION_BUZZER
#include "Buzzer.hpp"
#endif

volatile float PITCH_FEEDFORWARD = 2.0f;
volatile float YAW_SPIN_GAIN = -0.01f;

using namespace GIMBAL;
using namespace PINYMOTOR;

void ChassisCtrlTxPacket::transmit(uint8_t *_buf, uint16_t _len)
{
    Can::instance().transmitData(&HCAN3, uid(), _buf, _len);
}

void ShooterCtrlTxPacket::transmit(uint8_t *_buf, uint16_t _len)
{
    Can::instance().transmitData(&HCAN3, uid(), _buf, _len);
}

void GimbalStateTxPacket::transmit(uint8_t *_buf, uint16_t _len)
{
    Can::instance().transmitData(&HCAN3, uid(), _buf, _len);
}

void RefereeDataRxPacket::registerCallback()
{
    Can::instance().registerCallback(&HCAN3, uid(), [this](const uint8_t *_rxBuf) {
        xQueueSendFromISR(this->queue_, _rxBuf, nullptr);
    });
    LOG::info("RefereeDataRxPacket", " Receive cb registered, uId:0x%hx \n", uid());
}

void GimbalLaunchStrategyRxPacket ::registerCallback()
{
    Can::instance().registerCallback(&HCAN3, uid(), [this](const uint8_t *_rxBuf) {
        xQueueSendFromISR(this->queue_, _rxBuf, nullptr);
    });
    LOG::info("GimbalLaunchStrategyRxPacket", " Receive cb registered, uId:0x%hx \n", uid());
}

void ChassisStateRxPacket::registerCallback()
{
    Can::instance().registerCallback(&HCAN3, uid(), [this](const uint8_t *_rxBuf) {
        xQueueSendFromISR(this->queue_, _rxBuf, nullptr);
    });
    LOG::info("ChassisStateRxPacket", " Receive cb registered, uId:0x%hx \n", uid());
}

Standard::Standard()
{
    InitConfig_s pitchConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 2,
        .txFreq = PITCH_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = PITCH_REVERSE,
    };
    motors_.pitch = new DMMOTOR::DM4310("Pitch", pitchConfig);
    motors_.pitch->setZeroAng(PITCH_LAUNCH_ANG);
    LOG::info("Gimbal", "Pitch zeros has set to %f", motors_.pitch->data().zeroAng);

    InitConfig_s yawConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN3),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 1,
        .txFreq = YAW_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = YAW_REVERSE,
    };
    motors_.yaw = new DMMOTOR::DM4310("Yaw", yawConfig);
    motors_.yaw->setZeroAng(YAW_LAUNCH_ANG);
    LOG::info("Gimbal", "Yaw zeros has set to %f", motors_.yaw->data().zeroAng);

    this->stateFactory_.addState(FSMState_e::RUN, std::make_unique<RunState>(this));
    this->stateFactory_.addState(FSMState_e::AUTO, std::make_unique<AutoState>(this));
    this->stateFactory_.addState(FSMState_e::U_TURN, std::make_unique<UTurnState>(this));
    this->stateFactory_.addState(FSMState_e::LAUNCH, std::make_unique<LaunchState>(this));
    this->stateFactory_.addState(FSMState_e::STOP, std::make_unique<StopState>(this));
    this->stateFactory_.init(this->stateFactory_.getNextState(FSMState_e::STOP));
}

void Standard::stop()
{
    outFilrYaw_.reset();
    outFilrPitch_.reset();

    motors_.pitch->cmd(MotorCmdType_e::OFF);
    motors_.yaw->cmd(MotorCmdType_e::OFF);
}

void Standard::enter()
{
#if EXTENSION_BUZZER
    BUZZER::buzz->playNote(BUZZER::Tone_e::FS7, 50);
    BUZZER::buzz->playNote(BUZZER::Tone_e::C7, 50);
#endif

    motors_.pitch->cmd(MotorCmdType_e::ON);
    motors_.yaw->cmd(MotorCmdType_e::ON);
}

void Standard::update(void *_param)
{
    this->insSub_->receive();

    // TODO: optimize the way of checking RC connection
    if (xQueueReceive((((MsgBus_s *)_param)->gimbalQueue), &msg, 0) == pdTRUE) {
        disconnectRcCnt_ = 0;
    } else {
        disconnectRcCnt_++;
        static constexpr uint16_t MAX_DISCONNECT_CNT = 1200;
        if (disconnectRcCnt_ > MAX_DISCONNECT_CNT) {
            chassisCtrlTxPacket_.setData().chassisExptState = static_cast<uint8_t>(CHASSIS::FSMState_e::STOP);
            shooterCtrlTxPacket_.setData().armorLoaderExptState = static_cast<uint8_t>(ARMORLOADER::FSMState_e::STOP);
            msg.state = FSMState_e::STOP;
        }
    };

    if (motors_.pitch->globalState != GlobalState_e::ONLINE || motors_.yaw->globalState != GlobalState_e::ONLINE) {
        msg.state = FSMState_e::STOP;
    }

    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &cmsg_, 0) == pdTRUE) {
        if (cmsg_.noChassis) {
            cmsg_.state = CHASSIS::FSMState_e::STOP; // debug
        }
        static bool noChassisPrev = false;
        if (noChassisPrev != cmsg_.noChassis) {
#if EXTENSION_BUZZER
            for (uint8_t i = 0; i < 4; i++) {
                BUZZER::buzz->playNote(BUZZER::Tone_e::F6, 100);
                BUZZER::buzz->playNote(BUZZER::Tone_e::REST, 100);
            }
#endif
        }
        noChassisPrev = cmsg_.noChassis;

        chassisCtrlTxPacket_.setData().chassisExptState = static_cast<uint8_t>(cmsg_.state);
        chassisCtrlTxPacket_.setData().vx = cmsg_.vx;
        chassisCtrlTxPacket_.setData().vy = cmsg_.vy;
        chassisCtrlTxPacket_.setData().dl = cmsg_.dl;
        chassisCtrlTxPacket_.setData().droll = cmsg_.droll;
        chassisCtrlTxPacket_.setData().needJump = cmsg_.needJump;
        chassisCtrlTxPacket_.setData().needClimb = cmsg_.needClimb;
        chassisCtrlTxPacket_.setData().needStretch = cmsg_.needStretch;
        chassisCtrlTxPacket_.setData().needRush = cmsg_.needRush;
        chassisCtrlTxPacket_.setData().uiReset = cmsg_.uiReset;
    };
    if (cmsg_.state != CHASSIS::FSMState_e::STOP &&
        stateFactory_.getCurrentState()->getName() == GIMBAL::FSMState_e::U_TURN)
        chassisCtrlTxPacket_.setData().chassisExptState = static_cast<uint8_t>(
                CHASSIS::FSMState_e::TANK); // when gimbal is in U_TURN, chassis can only be in TANK or STOP
    chassisCtrlTxPacket_.setData().deltaYaw = motors_.yaw->posNorm();

    if (xQueueReceive((((MsgBus_s *)_param)->armorLoaderQueue), &almsg_, 0) == pdTRUE) {
        shooterCtrlTxPacket_.setData().armorLoaderExptState = static_cast<uint8_t>(almsg_.state);
        shooterCtrlTxPacket_.setData().fire = almsg_.fire;
    }
    if (!msg.forceFire)
        shooterCtrlTxPacket_.setData().fire &=
                (vision.data().control) ? vision.data().fire | (msg.state != FSMState_e::AUTO) : true;

    gimbalStateTxPacket_.setData().gimbalRealState = static_cast<uint8_t>(stateFactory_.getCurrentState()->getName());
    gimbalStateTxPacket_.setData().aiming = vision.data().control;
    gimbalStateTxPacket_.setData().visionOnline = vision.pcState == Vision::PCState_e::ONLINE;
    gimbalStateTxPacket_.setData().pitch = ins().pitch;

    // vision send
#if EXTENSION_REFEREE
    targetMode_ = 0;
    if (almsg_.state == ARMORLOADER::FSMState_e::SINGLE) {
        if (refereeDataRxPacket_.data().stageRemainTime < 240)
            targetMode_ = 2;
        else
            targetMode_ = 1;
    }
    vision.setData().detectColor = !refereeDataRxPacket_.data().color;
    vision.setData().taskMode = targetMode_;
    vision.setData().isPlay = refereeDataRxPacket_.data().isPlay;
    vision.setData().bulletSpeed = refereeDataRxPacket_.data().muzzleVel;
#else
    vision.setData().taskMode = almsg_.state == ARMORLOADER::FSMState_e::SINGLE ? 1 : 0;
    vision.setData().detectColor = 1; // enemy is blue
    vision.setData().isPlay = false;
    vision.setData().bulletSpeed = -1;
#endif

    vision.setData().resetTracker = msg.resetTracker;
    vision.setData().changeTarget = msg.changeTarget;
    vision.setData().roll = insData_.roll;
    vision.setData().pitch = insData_.pitch;
    vision.setData().yaw = insData_.yaw;

    // update state
    this->stateFactory_.update();
}

PINYMOTOR::IMotor *Standard::pitch() const { return motors_.pitch; }
PINYMOTOR::IMotor *Standard::yaw() const { return motors_.yaw; }

void Standard::ctrlPitchSmooth(PID *_angPID, PID *_velPID, float _insPos, float _step)
{
    float delta = getMinorArc(_insPos, ins().pitch);
    delta = clamp(delta, -_step, _step);
    ctrlPitch(_angPID, _velPID, ins().pitch + delta);
}

void Standard::ctrlPitchAuto(PID *_angPID, PID *_velPID, float _insPos)
{
    float insPitchDelArc = getMinorArc(_insPos, ins().pitch);
    float pos = rangeMap(pitch()->posNorm() + insPitchDelArc);
    float del = getMinorArc(pos, pitch()->posNorm());
    float vel = _angPID->calc(del, 0);
    vel = clamp(vel, -vision.data().pitchVel, vision.data().pitchVel);
    float acc = _velPID->calc(vel, insData_.body.gy);
    acc += vision.data().pitchAcc; // forward feedback
    float torq = calcPitchTorq(_insPos, vel, acc);
    pitch()->cmdTorq(outFilrPitch_.process(torq) - (PITCH_FEEDFORWARD * fabsf(arm_cos_f32(ins().pitch))));
}

void Standard::ctrlPitch(PID *_angPID, PID *_velPID, float _insPos)
{
    float insPitchDelArc = getMinorArc(_insPos, ins().pitch);
    float pos = rangeMap(pitch()->posNorm() + insPitchDelArc);
    ctrlMotorPitch(_angPID, _velPID, pos);
}

void Standard::ctrlMotorPitch(PID *_angPID, PID *_velPID, float _pos)
{
    float del = getMinorArc(_pos, pitch()->posNorm());
    float vel = _angPID->calc(del, 0);
    float torq = _velPID->calc(vel, insData_.body.gy);
    pitch()->cmdTorq(outFilrPitch_.process(torq) - (PITCH_FEEDFORWARD * fabsf(arm_cos_f32(ins().pitch))));
}

void Standard::ctrlYawSmooth(PID *_angPID, PID *_velPID, float _insPos, float _step)
{
    float delta = getMinorArc(_insPos, ins().yaw);
    delta = clamp(delta, -_step, _step);
    ctrlYaw(_angPID, _velPID, ins().yaw + delta);
}

void Standard::ctrlYawAuto(PID *_angPID, PID *_velPID, PID *_velFeedbackPID, float _insPos)
{
    float insYawDelArc = getMinorArc(_insPos, ins().yaw);
    float pos = rangeMap(yaw()->posNorm() + insYawDelArc);
    float del = getMinorArc(pos, yaw()->posNorm());
    float vel = _angPID->calc(del, 0);
    float acc = _velPID->calc(vel, insData_.earth.gz) + _velFeedbackPID->calc(vision.data().yawVel, insData_.earth.gz) +
                vision.data().yawAcc;
    float torq = calcYawTorq(insData_.earth.gz, acc);
    torq = std::clamp(torq, -DMMOTOR::DM4310::TORQ_MAX, DMMOTOR::DM4310::TORQ_MAX);
    yaw()->cmdTorq(outFilrYaw_.process(torq));
}

void Standard::ctrlYaw(PID *_angPID, PID *_velPID, float _insPos)
{
    float insYawDelArc = getMinorArc(_insPos, ins().yaw);
    float pos = rangeMap(yaw()->posNorm() + insYawDelArc);
    ctrlMotorYaw(_angPID, _velPID, pos);
}

void Standard::ctrlMotorYaw(PID *_angPID, PID *_velPID, float _pos)
{
    float del = getMinorArc(_pos, yaw()->posNorm());
    float vel = _angPID->calc(del, 0);
    float torq = _velPID->calc(vel, insData_.body.gz) + (YAW_SPIN_GAIN * chassisStateRxPacket_.data().wz);
    yaw()->cmdTorq(outFilrYaw_.process(torq));
}

void Standard::loadResetState(bool _state) { chassisCtrlTxPacket_.setData().gimbalReset = _state; }

float Standard::calcPitchTorq(float _ang, float _vel, float _acc)
{
    (void)_ang;
    (void)_vel;
    (void)_acc;
    // return 0.f; // TODO:
}

float Standard::calcYawTorq(float _vel, float _acc)
{
    static constexpr auto SIGN = [](float _v) {
        if (_v > 0.f)
            return 1.f;
        else if (_v < 0.f)
            return -1.f;
        else
            return 0.f;
    };

    float torq = (YAW_INERTIA * _acc) + (YAW_FRICTION * _vel) + (YAW_COULOMB * SIGN(_vel));
    return torq;
}

Gimbal::LaunchStrategy_e Standard::generateLaunchStrategy()
{
    // TODO: Currently all launch strategies are set to normal
    float yawAng = yaw()->posNorm();
    Gimbal::LaunchStrategy_e strategy = Gimbal::LaunchStrategy_e::NORMAL;
    bool legLFront = gimbalLaunchStrategyRxPacket_.data().isLegLFront;
    bool legRFront = gimbalLaunchStrategyRxPacket_.data().isLegRFront;
    if (yawAng > PI / 2.f && yawAng < 3.f * PI / 2.f) {
        // the head is in the back
        if (legLFront && legRFront) {
            strategy = Gimbal::LaunchStrategy_e::NORMAL;
        } else if (legLFront && !legRFront) {
            strategy = Gimbal::LaunchStrategy_e::NORMAL;
        } else if (!legLFront && legRFront) {
            strategy = Gimbal::LaunchStrategy_e::NORMAL;
        } else {
            strategy = Gimbal::LaunchStrategy_e::NORMAL;
        }
    } else {
        // the head is in the front
        if (legLFront && legRFront) {
            strategy = Gimbal::LaunchStrategy_e::NORMAL;
        } else if (legLFront && !legRFront) {
            strategy = Gimbal::LaunchStrategy_e::NORMAL;
        } else if (!legLFront && legRFront) {
            strategy = Gimbal::LaunchStrategy_e::NORMAL;
        } else {
            strategy = Gimbal::LaunchStrategy_e::NORMAL;
        }
    }
    return strategy;
}

static bool isNeedGimbalLaunchPrev = false;
bool Standard::isNeedLaunch()
{
    bool rslt = !isNeedGimbalLaunchPrev && gimbalLaunchStrategyRxPacket_.data().isNeedGimbalLaunch;
    if (isNeedGimbalLaunchPrev != gimbalLaunchStrategyRxPacket_.data().isNeedGimbalLaunch) {
        LOG::info("Gimbal", "Receive launch command %d", gimbalLaunchStrategyRxPacket_.data().isNeedGimbalLaunch);
        isNeedGimbalLaunchPrev = gimbalLaunchStrategyRxPacket_.data().isNeedGimbalLaunch;
    }
    return rslt;
}

bool Standard::isAllowExitLaunch()
{
    return gimbalLaunchStrategyRxPacket_.data().isAllowExitLaunch || cmsg_.state == CHASSIS::FSMState_e::STOP;
}

bool Standard::targetInBuff() const { return targetMode_ > 0; }
