#include "DualFric.hpp"

#include "Bsp_can.hpp"

#include "M3508.hpp"

#include "Smooth.hpp"

#include <cmath>

#include "./State/OffState.hpp"
#include "./State/OnState.hpp"

#if EXTENSION_BUZZER
#include "Buzzer.hpp"
#endif

using namespace ARMORBOOSTER;
using namespace PINYMOTOR;

void ArmorBoosterStatePacketType::transmit(uint8_t *_buf, uint16_t _len)
{
    Can::instance().transmitData(&HCAN3, uid(), _buf, _len);
}

DualFric::DualFric()
{
    InitConfig_s fricLConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 2,
        .txFreq = FRIC_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = FRIC_REVERSE,
    };
    motors_.fricL = new DJIMOTOR::M3508("FricL", fricLConfig);

    InitConfig_s fricRConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 3,
        .txFreq = FRIC_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = !FRIC_REVERSE,
    };
    motors_.fricR = new DJIMOTOR::M3508("FricR", fricRConfig);

    this->stateFactory_.addState(FSMState_e::ON, std::make_unique<OnState>(this));
    this->stateFactory_.addState(FSMState_e::OFF, std::make_unique<OffState>(this));
    this->stateFactory_.init(this->stateFactory_.getNextState(FSMState_e::OFF));
}

void DualFric::stop()
{
    fricPid_[0].reset();
    fricPid_[1].reset();
    antiDiffPid_.reset();

    motors_.fricL->cmd(MotorCmdType_e::OFF);
    motors_.fricR->cmd(MotorCmdType_e::OFF);
}

void DualFric::enter()
{
    // detect motor disconnect
    bool motorOffline = false;
    for (auto &i : motors_.dat) {
        if (i->globalState != GlobalState_e::ONLINE) {
            motorOffline = true;
            break;
        }
    }

    if (motorOffline) {
#if EXTENSION_BUZZER
        BUZZER::buzz->playNote(BUZZER::Tone_e::REST, 50);
        BUZZER::buzz->playNote(BUZZER::Tone_e::C8, 100);
        BUZZER::buzz->playNote(BUZZER::Tone_e::REST, 50);
        BUZZER::buzz->playNote(BUZZER::Tone_e::C8, 100);
        BUZZER::buzz->playNote(BUZZER::Tone_e::REST, 50);
        BUZZER::buzz->playNote(BUZZER::Tone_e::C8, 100);
        BUZZER::buzz->playNote(BUZZER::Tone_e::REST, 50);
        BUZZER::buzz->playNote(BUZZER::Tone_e::C8, 100);
#endif
        stop();
    } else {
        for (auto &i : motors_.dat) {
            i->cmd(PINYMOTOR::MotorCmdType_e::ON);
        }
    }
}

void DualFric::update(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->armorBoosterQueue), &msg, 0) == pdTRUE) {
    };

    armorBoosterStateTxPacket_.setData().armorBoosterRealState =
            static_cast<uint8_t>(stateFactory_.getCurrentState()->getName());

    if (motors_.fricL->globalState != GlobalState_e::ONLINE || motors_.fricR->globalState != GlobalState_e::ONLINE) {
        msg.state = FSMState_e::OFF;
    }

    for (uint8_t i = 0; i < 2; ++i)
        curVel_[i] = velFltr_[i].process(motors_.dat[i]->vel());
    float avgVel = (curVel_[0] + curVel_[1]) / 2.f;
    bool running = avgVel > FRIC_RUNNING_THRESHILD;
    armorBoosterStateTxPacket_.setData().running = running;

    // update state
    this->stateFactory_.update();
}

void DualFric::ctrl(float _vel)
{
    float diffVel = curVel_[0] - curVel_[1];
    float antiDiffPidOut = antiDiffPid_.calc(diffVel, 0);
    float velErr = _vel - velFltr_[0].process(curVel_[0]);
    float pidOut = fricPid_[0].calc(velErr, 0);
    motors_.fricL->cmdTorq(
            std::clamp(pidOut - antiDiffPidOut, -DJIMOTOR::M3508::TORQ_RATED, DJIMOTOR::M3508::TORQ_RATED));
    velErr = _vel - velFltr_[1].process(curVel_[1]);
    pidOut = fricPid_[1].calc(velErr, 0);
    motors_.fricR->cmdTorq(
            std::clamp(pidOut + antiDiffPidOut, -DJIMOTOR::M3508::TORQ_RATED, DJIMOTOR::M3508::TORQ_RATED));
}

void DualFric::on()
{
    // float constTarget = FAST_FRIC_RATE;
    // SMOOTH::slopeFollowing(&constTarget, &targetVel_, 0.5f);
    ctrl(FAST_FRIC_RATE);
}

void DualFric::off()
{
    if (std::fabs(motors_.fricL->vel() + motors_.fricR->vel()) < FRIC_STOP_THRESHOLD) {
        stop();
    } else {
        float constZero = 0;
        SMOOTH::slopeFollowing(&constZero, &targetVel_, 0.5f);
        ctrl(targetVel_);
    }
}