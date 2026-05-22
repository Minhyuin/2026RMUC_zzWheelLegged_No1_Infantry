#pragma once

#include "ArmorLoader.hpp"

#include "IMotor.hpp"

#include "LPF.hpp"

#include "RxPacket.hpp"
#include "PacketImpl.hpp"

namespace ARMORLOADER {

class ShooterCtrlRxPacket : public COMM::RxPacket<COMM::ArmorLoaderCtrlPacketType, 1> {
    friend class Dial;
    ShooterCtrlRxPacket() { registerCallback(); }
    void registerCallback() final;
};

class ArmorBoosterStateRxPacket : public COMM::RxPacket<COMM::ArmorBoosterStatePacketType, 1> {
    friend class Dial;
    ArmorBoosterStateRxPacket() { registerCallback(); }
    void registerCallback() final;
};

class Dial : public ArmorLoader {
    struct Motors_s {
        PINYMOTOR::IMotor *dial;
    };

public:
    static constexpr float OUTER_RR = 1.f;

    static constexpr uint8_t BULLET_NUM_ROUND = 10;

    static constexpr float DIAL_MOTOR_FREQ = 500.f;

    static constexpr bool DIAL_REVERSE = false;

    static constexpr float BLOCK_CURRENT = 2.3f;   // A
    static constexpr float BLOCK_SPEED_MIN = 0.3f; // rad/s
    static constexpr uint16_t BLOCK_CNT_MAX = 500.f;

    static constexpr float BARREL_HEAT_THRESHOLD_OFFSET = 40.f;

    static constexpr float MAX_BULLETS_RATE = 15.f; // bullets per second

    FSMState_e lastState = FSMState_e::STOP;

    Dial();

    PINYMOTOR::IMotor *dial() const;

    /**
     * @brief ctrl dial rotate in _torq
     * 
     * @param _torq 
     */
    void ctrl(float _torq);

    /**
     * @brief ctrl dial rotate in _vel
     * 
     * @param _velPID 
     * @param _vel rad/s
     */
    void ctrl(PID *_velPID, float _vel);

    /**
     * @brief ctrl dial rotate to _ang
     * 
     * @param _posPID 
     * @param _velPID 
     * @param _ang multiple circle ang in rad
     */
    void ctrl(PID *_posPID, PID *_velPID, float _ang);

    void stop();
    void enter();
    void update(void *_param);

    bool isBlock() const;

    float targetRate() const;

private:
    Motors_s motors_;

    FILTER::LPF velFltr_{ DIAL_MOTOR_FREQ, 50.f };

    ShooterCtrlRxPacket shooterCtrlRxPacket_;
    ArmorBoosterStateRxPacket armorBoosterStateRxPacket_;

    float vel_;  // rad/s
    float curr_; // A

    float barrelHeat_ = 0.f;
    uint16_t lastBarrelHeatFromReferee_ = 0.f;
    float lastMuzzleVel = 0.f;

    uint16_t roundsFiredNum_ = 0; // the number of rounds fired

    uint16_t blockCnt = 0.f;
};

}; // namespace ARMORLOADER
