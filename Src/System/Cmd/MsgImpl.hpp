#pragma once
#include "MsgBase.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include <array>


namespace CHASSIS {
enum class FSMState_e : uint8_t;
}

namespace GIMBAL {
enum class FSMState_e : uint8_t;
}

namespace ARMORBOOSTER {
enum class FSMState_e : uint8_t;
}

namespace ARMORLOADER {
enum class FSMState_e : uint8_t;
}

struct MsgBus_s {
    QueueHandle_t chassisQueue;
    QueueHandle_t gimbalQueue;
    QueueHandle_t armorBoosterQueue;
    QueueHandle_t armorLoaderQueue;
    QueueHandle_t refereeQueue;
    QueueHandle_t tofLefDistQueue;
    QueueHandle_t tofRigDistQueue;
};

//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ ctrl msg ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
struct ChassisMsg_s : public Msg {
    CHASSIS::FSMState_e state;
    float vx, vy, wz;
    float dl;
    float dphi;
};

struct GimbalMsg_s : public Msg {
    GIMBAL::FSMState_e state;
    float wRoll, wPitch, wYaw;
};

struct ArmorBoosterMsg_s : public Msg {
    ARMORBOOSTER::FSMState_e state;
};


struct ArmorLoaderMsg_s : public Msg {
    ARMORLOADER::FSMState_e state;
    bool fire;
};

struct RefereeMsg_s : public Msg {
    uint16_t currentHP, shooterHeatLimit, chassisPowerLimit, chassisPowerBuffer;
    float bulletSpeed, rxFreq;
};

struct TOFMsg_s : public Msg {
    float dist;
};

//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ other msg ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
struct INSEularMsg_s : public Msg {
    float roll, pitch, yaw;
};

struct GimbalYawMsg_s : public Msg {
    bool isReset;
    float followYaw;
    float chassisYaw; // only single core system need it
};

//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ temp cache ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
struct ChassisTxMsg_s {
    CHASSIS::FSMState_e state;
};

struct GimbalTxMsg_s {};

struct ArmTxMsg_s {};

struct RefereeTxMsg_s {};
