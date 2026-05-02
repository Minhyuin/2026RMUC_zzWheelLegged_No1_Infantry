#include "TripFric.hpp"

#include "Bsp_can.hpp"

#include "M2006.hpp"

#include "Smooth.hpp"

#include <cmath>

#include "./State/OffState.hpp"
#include "./State/OnState.hpp"

using namespace ARMORBOOSTER;
using namespace PINYMOTOR;

void ArmorBoosterStatePacketType::transmit(uint8_t *_buf, uint16_t _len)
{
    Can::instance().transmitData(&HCAN3, uid(), _buf, _len);
}

TripFric::TripFric()
{
    InitConfig_s fricLConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 1,
        .txFreq = FRIC_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = true,
    };
    motors_.fricL = new DJIMOTOR::M2006("FricL", fricLConfig);

    InitConfig_s fricMConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 2,
        .txFreq = FRIC_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = false,
    };
    motors_.fricM = new DJIMOTOR::M2006("FricM", fricMConfig);

    InitConfig_s fricRConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 3,
        .txFreq = FRIC_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = false,
    };
    motors_.fricR = new DJIMOTOR::M2006("FricR", fricRConfig);

    this->stateFactory_.addState(FSMState_e::ON, std::make_unique<OnState>(this));
    this->stateFactory_.addState(FSMState_e::OFF, std::make_unique<OffState>(this));
    this->stateFactory_.init(this->stateFactory_.getNextState(FSMState_e::OFF));
}

void TripFric::stop()
{
    motors_.fricL->cmd(MotorCmdType_e::OFF);
    motors_.fricM->cmd(MotorCmdType_e::OFF);
    motors_.fricR->cmd(MotorCmdType_e::OFF);
}

void TripFric::enter()
{
    motors_.fricL->cmd(MotorCmdType_e::ON);
    motors_.fricM->cmd(MotorCmdType_e::ON);
    motors_.fricR->cmd(MotorCmdType_e::ON);
}

void TripFric::update(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->armorBoosterQueue), &msg, 0) == pdTRUE) {
    };

    armorBoosterStateTxPacket_.setData().armorBoosterRealState =
            static_cast<uint8_t>(stateFactory_.getCurrentState()->getName());

    if (motors_.fricL->globalState != GlobalState_e::ONLINE || motors_.fricM->globalState != GlobalState_e::ONLINE ||
        motors_.fricR->globalState != GlobalState_e::ONLINE) {
        msg.state = FSMState_e::OFF;
    }

    // update state
    this->stateFactory_.update();
}

void TripFric::ctrl(float _vel)
{
    float velErr = _vel - velFltr_[0].process(motors_.fricL->vel());
    float pidOut = fricPid_[0].calc(velErr, 0);
    motors_.fricL->cmdTorq(pidOut);
    velErr = _vel - velFltr_[1].process(motors_.fricM->vel());
    pidOut = fricPid_[1].calc(velErr, 0);
    motors_.fricM->cmdTorq(pidOut);
    velErr = _vel - velFltr_[2].process(motors_.fricR->vel());
    pidOut = fricPid_[2].calc(velErr, 0);
    motors_.fricR->cmdTorq(pidOut);
}

void TripFric::on()
{
    // float constTarget = FAST_FRIC_RATE;
    // SMOOTH::slopeFollowing(&constTarget, &targetVel_, 0.5f);
    ctrl(FAST_FRIC_RATE);
}

void TripFric::off()
{
    if (std::fabs(motors_.fricL->vel() + motors_.fricM->vel() + motors_.fricR->vel()) < FRIC_STOP_THRESHOLD) {
        stop();
    } else {
        float constZero = 0;
        SMOOTH::slopeFollowing(&constZero, &targetVel_, 0.5f);
        ctrl(targetVel_);
    }
}