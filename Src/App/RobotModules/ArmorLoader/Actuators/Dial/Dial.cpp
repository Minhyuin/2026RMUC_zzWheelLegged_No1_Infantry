#include "Dial.hpp"

#include <cmath>

#include "Bsp_can.hpp"

#include "MG4005.hpp"

#include "./State/AutoState.hpp"
#include "./State/BlockState.hpp"
#include "./State/SingleState.hpp"
#include "./State/StopState.hpp"

#if EXTENSION_REFEREE
#include "Referee.hpp"
#endif

using namespace ARMORLOADER;
using namespace PINYMOTOR;

void ShooterCtrlRxPacket::registerCallback()
{
    Can::instance().registerCallback(&HCAN3, uid(), [this](const uint8_t *_rxBuf) {
        xQueueSendFromISR(this->queue_, _rxBuf, nullptr);
    });
    LOG::info("ShooterCtrlRxPacket", " Receive cb registered, uId:0x%hx", uid());
}

Dial::Dial()
{
    InitConfig_s dialConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 1,
        .txFreq = DIAL_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = DIAL_REVERSE,
    };
    motors_.dial = new LKMOTOR::MG4005("MG4005", dialConfig);
    motors_.dial->overrideReductionRatio(LKMOTOR::MG4005::REDUCTION_RATIO * OUTER_RR);

    this->stateFactory_.addState(FSMState_e::AUTO, std::make_unique<AutoState>(this));
    this->stateFactory_.addState(FSMState_e::BLOCK, std::make_unique<BlockState>(this));
    this->stateFactory_.addState(FSMState_e::SINGLE, std::make_unique<SingleState>(this));
    this->stateFactory_.addState(FSMState_e::STOP, std::make_unique<StopState>(this));
    this->stateFactory_.init(this->stateFactory_.getNextState(FSMState_e::STOP));
}

void Dial::stop() { motors_.dial->cmd(MotorCmdType_e::OFF); }

void Dial::enter()
{
    motors_.dial->setZeroAng(); // reset multipCirAng and cirNum
    motors_.dial->cmd(MotorCmdType_e::ON);
}

void Dial::update(void *_param)
{
    // if (xQueueReceive((((MsgBus_s *)_param)->armorLoaderQueue), &msg, 0) == pdTRUE) {
    // };
    UNUSED(_param);
    msg.state = static_cast<FSMState_e>(shooterCtrlRxPacket_.data().armorLoaderExptState);
    msg.fire = shooterCtrlRxPacket_.data().fire;
    if (motors_.dial->globalState != GlobalState_e::ONLINE) {
        msg.state = FSMState_e::STOP;
    }

    // uint16_t nextRoundsFiredNum = static_cast<int>(motors_.dial->pos() / (2.f * PI)) * BULLET_NUM_ROUND;
    // uint16_t deltaRoundsFiredNum = nextRoundsFiredNum - roundsFiredNum_;
    // if (deltaRoundsFiredNum > 0)
    //     barrelHeat_ += static_cast<float>(deltaRoundsFiredNum * 10); // 每颗17mm弹丸10热量
    // roundsFiredNum_ = nextRoundsFiredNum;

#if EXTENSION_REFEREE
    const auto &refereeData = referee->receiver.getRefereeData();
    if (lastBarrelHeatFromReferee_ != refereeData.powerHeatData.shooter_17mmBarrelHeat) {
        barrelHeat_ = refereeData.powerHeatData.shooter_17mmBarrelHeat;
        lastBarrelHeatFromReferee_ = refereeData.powerHeatData.shooter_17mmBarrelHeat;
    }
    if (barrelHeat_ > 0)
        barrelHeat_ -= static_cast<float>(refereeData.gameRobotStatus.shooterCoolingValue) * 0.001f;
    msg.fire &= barrelHeat_ <=
                static_cast<float>(refereeData.gameRobotStatus.shooterHeatLimit) - BARREL_HEAT_THRESHOLD_OFFSET;
#endif

    vel_ = velFltr_.process(dial()->vel());
    curr_ = dial()->data().curr;

    if (std::fabs(curr_) > BLOCK_CURRENT && std::fabs(vel_) < BLOCK_SPEED_MIN) {
        blockCnt = blockCnt < BLOCK_CNT_MAX ? blockCnt + 1 : BLOCK_CNT_MAX;
    } else {
        blockCnt = blockCnt > 0 ? blockCnt - 1 : 0;
    }

    // update state
    this->stateFactory_.update();
}

IMotor *Dial::dial() const { return motors_.dial; }

void Dial::ctrl(float _torq) { dial()->cmdTorq(_torq); }

void Dial::ctrl(PID *_velPID, float _vel) { dial()->cmdTorq(_velPID->calc(_vel, vel_)); }

void Dial::ctrl(PID *_posPID, PID *_velPID, float _ang)
{
    float v = _posPID->calc(_ang, dial()->pos());
    dial()->cmdTorq(_velPID->calc(v, vel_));
}

bool Dial::isBlock() const { return blockCnt >= BLOCK_CNT_MAX; }
