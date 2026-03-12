#include "DualFric.hpp"

#include "Bsp_can.hpp"

#include "DM3519.hpp"

#include "Smooth.hpp"

#include <cmath>

#include "./State/OffState.hpp"
#include "./State/OnState.hpp"

using namespace ARMORBOOSTER;
using namespace PINYMOTOR;

DualFric::DualFric()
{
    InitConfig_s fricLConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::VDES,
        .offsetId = 1,
        .txFreq = FRIC_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = FRIC_REVERSE,
    };
    motors_.fricL = new DMMOTOR::DM3519("FricL", fricLConfig);

    InitConfig_s fricRConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::VDES,
        .offsetId = 2,
        .txFreq = FRIC_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = !FRIC_REVERSE,
    };
    motors_.fricR = new DMMOTOR::DM3519("FricR", fricRConfig);

    this->stateFactory_.addState(FSMState_e::ON, std::make_unique<OnState>(this));
    this->stateFactory_.addState(FSMState_e::OFF, std::make_unique<OffState>(this));
    this->stateFactory_.init(this->stateFactory_.getNextState(FSMState_e::OFF));
}

void DualFric::stop()
{
    motors_.fricL->cmd(MotorCmdType_e::OFF);
    motors_.fricR->cmd(MotorCmdType_e::OFF);
}

void DualFric::enter()
{
    motors_.fricL->cmd(MotorCmdType_e::ON);
    motors_.fricR->cmd(MotorCmdType_e::ON);
}

void DualFric::update(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->armorBoosterQueue), &msg, 0) == pdTRUE) {
    };

    if (motors_.fricL->globalState != GlobalState_e::ONLINE || motors_.fricR->globalState != GlobalState_e::ONLINE) {
        msg.state = FSMState_e::OFF;
    }

    // update state
    this->stateFactory_.update();
}

void DualFric::ctrl(float _vel)
{
    // float velErr = _vel - velFltr_[0].process(motors_.fricL->vel());
    // float pidOut = fricPid_[0].calc(velErr, 0);
    // motors_.fricL->cmdTorq(pidOut);
    // velErr = _vel - velFltr_[1].process(motors_.fricR->vel());
    // pidOut = fricPid_[1].calc(velErr, 0);
    // motors_.fricR->cmdTorq(pidOut);
    motors_.fricR->cmdVel(_vel);
    motors_.fricL->cmdVel(_vel);
}

void DualFric::on()
{
    float constTarget = FAST_FRIC_RATE;
    SMOOTH::slopeFollowing(&constTarget, &targetVel_, 0.2f);
    ctrl(targetVel_);
}

void DualFric::off()
{
    if (std::fabs(motors_.fricL->vel() + motors_.fricR->vel()) < FRIC_STOP_THRESHOLD) {
        stop();
    } else {
        float constZero = 0;
        SMOOTH::slopeFollowing(&constZero, &targetVel_, 0.2f);
        ctrl(targetVel_);
    }
}
