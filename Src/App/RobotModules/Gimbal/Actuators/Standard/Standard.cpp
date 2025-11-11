#include "Standard.hpp"

#include "Pid.hpp"

#include "DM4310.hpp"

#include "Bsp_can.hpp"

#include "./State/StopState.hpp"
#include "./State/LaunchState.hpp"
#include "./State/RunState.hpp"
#include "./State/AutoState.hpp"

using namespace GIMBAL;
using namespace PINYMOTOR;

void ChassisCtrlTxPacket::transmit(uint8_t *_buf, uint16_t _len)
{
    Can::instance().transmitData(&HCAN3, uid(), _buf, _len);
}

Standard::Standard()
{
    InitConfig_s pitchConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN3),
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
    LOG::info("Gimbal", "Pitch zeros has set to %f",
              motors_.pitch->data().zeroAng);

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

    this->stateFactory_.addState(FSMState_e::RUN,
                                 std::make_unique<RunState>(this));
    this->stateFactory_.addState(FSMState_e::AUTO,
                                 std::make_unique<AutoState>(this));
    this->stateFactory_.addState(FSMState_e::LAUNCH,
                                 std::make_unique<LaunchState>(this));
    this->stateFactory_.addState(FSMState_e::STOP,
                                 std::make_unique<StopState>(this));
    this->stateFactory_.init(
            this->stateFactory_.getNextState(FSMState_e::STOP));
}

void Standard::stop()
{
    outFilrYaw_.reset();
    outFilrPitch_.reset();

    for (auto &i : motors_._) {
        i->cmd(PINYMOTOR::MotorCmdType_e::OFF);
    }
}

void Standard::enter()
{
    for (auto &i : motors_._) {
        i->cmd(PINYMOTOR::MotorCmdType_e::ON);
    }
}

void Standard::update(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->gimbalQueue), &msg, 0) == pdTRUE) {
    };
    this->insSub_->receive();

#ifdef APP_GIMBAL_MASTER
    ChassisMsg_s cmsg{};
    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &cmsg, 0) ==
        pdTRUE) {
        chassisCtrlTxPacket_.setData().state = static_cast<uint8_t>(cmsg.state);
        chassisCtrlTxPacket_.setData().vx = cmsg.vx;
        chassisCtrlTxPacket_.setData().vy = cmsg.vy;
    };
    chassisCtrlTxPacket_.setData().deltaYaw = motors_.yaw->posNorm();

    // vision send
    vision.setData().roll = insData_.roll;
    vision.setData().pitch = insData_.pitch;
    vision.setData().yaw = insData_.yaw;
    vision.setData().timestamp = xTaskGetTickCount();

#endif

    this->stateFactory_.update();
}

PINYMOTOR::IMotor *Standard::pitch() const { return motors_.pitch; }
PINYMOTOR::IMotor *Standard::yaw() const { return motors_.yaw; }

void Standard::ctrlPitchSmooth(PID *_angPID, PID *_velPID, float _insPos,
                               float _step)
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
    pitch()->cmdTorq(outFilrPitch_.process(torq));
}

void Standard::ctrlPitch(PID *_angPID, PID *_velPID, float _insPos)
{
    float insPitchDelArc = getMinorArc(_insPos, ins().pitch);
    float pos = rangeMap(pitch()->posNorm() + insPitchDelArc);
    float del = getMinorArc(pos, pitch()->posNorm());
    float vel = _angPID->calc(del, 0);
    float torq = _velPID->calc(vel, insData_.body.gy); // TODO: calc acc first
    pitch()->cmdTorq(outFilrPitch_.process(torq));
}

void Standard::ctrlYawSmooth(PID *_angPID, PID *_velPID, float _insPos,
                             float _step)
{
    float delta = getMinorArc(_insPos, ins().yaw);
    delta = clamp(delta, -_step, _step);
    ctrlYaw(_angPID, _velPID, ins().yaw + delta);
}

void Standard::ctrlYawAuto(PID *_angPID, PID *_velPID, float _insPos)
{
    float insYawDelArc = getMinorArc(_insPos, ins().yaw);
    float pos = rangeMap(yaw()->posNorm() + insYawDelArc);
    float del = getMinorArc(pos, yaw()->posNorm());
    float vel = _angPID->calc(del, 0);
    vel = clamp(vel, -vision.data().yawVel, vision.data().yawVel);
    float acc = _velPID->calc(vel, insData_.body.gz);
    acc += vision.data().yawAcc; // forward feedback
    float torq = calcYawTorq(vel, acc);
    yaw()->cmdTorq(outFilrYaw_.process(torq));
}

void Standard::ctrlYaw(PID *_angPID, PID *_velPID, float _insPos)
{
    float insYawDelArc = getMinorArc(_insPos, ins().yaw);
    float pos = rangeMap(yaw()->posNorm() + insYawDelArc);
    float del = getMinorArc(pos, yaw()->posNorm());
    float vel = _angPID->calc(del, 0);
    float torq = _velPID->calc(vel, insData_.body.gz); // TODO: calc acc first
    yaw()->cmdTorq(outFilrYaw_.process(torq));
}

void Standard::loadResetState(bool _state)
{
    chassisCtrlTxPacket_.setData().gimbalReset = _state;
}

float Standard::calcPitchTorq(float _ang, float _vel, float _acc)
{
    return 0.f; // TODO:
}

float Standard::calcYawTorq(float _vel, float _acc)
{
    return 0.f; // TODO:
}
