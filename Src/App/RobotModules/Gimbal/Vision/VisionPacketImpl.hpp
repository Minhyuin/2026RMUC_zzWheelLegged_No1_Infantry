#pragma once

#include <cstdint>

#include "Crc.hpp"
#include "StmLog.hpp"

class VisionTxPacket {
public:
    static constexpr uint8_t LEN = (160 / 8); // bits to bytes
    static constexpr uint16_t ID = 0x5A;
    struct ProtoData_s {
        uint8_t detectColor; // 0: red, 1: blue
        uint8_t taskMode;    // 0: armor, 1:small buff, 2: large buff
        bool resetTracker;
        bool isPlay;
        bool changeTarget;
        float roll;
        float pitch;
        float yaw;
        float bulletSpeed;
    };
#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            uint8_t header;
            uint8_t detectColor : 1;
            uint8_t taskMode : 2;
            bool resetTracker : 1;
            bool isPlay : 1;
            bool changeTarget : 1;
            uint8_t reserved : 2;
            float roll;
            float pitch;
            float yaw;
            float bulletSpeed;
            uint16_t checksum;
        } content;
        uint8_t bytes[LEN];
    } data;
#pragma pack(pop)
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.header = ID;
        d.detectColor = _protoData.detectColor;
        d.taskMode = _protoData.taskMode;
        d.resetTracker = _protoData.resetTracker;
        d.isPlay = _protoData.isPlay;
        d.changeTarget = _protoData.changeTarget;
        d.reserved = 0;
        d.roll = _protoData.roll;
        d.pitch = _protoData.pitch;
        d.yaw = _protoData.yaw;
        d.bulletSpeed = _protoData.bulletSpeed;
        d.checksum = Get_CRC16_Check_Sum(_data.bytes, LEN - 2, CRC16_INIT);
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        (void)_data;
        (void)_protoData;
    }
};

class VisionRxPacket {
public:
    static constexpr uint8_t LEN = (232 / 8); // bits to bytes
    static constexpr uint16_t ID = 0xA5;
    struct ProtoData_s {
        uint8_t control;
        uint8_t fire;
        float yaw;
        float yawVel;
        float yawAcc;
        float pitch;
        float pitchVel;
        float pitchAcc;
    };
#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            uint8_t header;
            uint8_t control;
            uint8_t fire;
            float yaw;
            float yawVel;
            float yawAcc;
            float pitch;
            float pitchVel;
            float pitchAcc;
            uint16_t checksum = 0;
        } content;
        uint8_t bytes[LEN];
    } data;
#pragma pack(pop)
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        (void)_protoData;
        (void)_data;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        if (_data.content.header == ID && Verify_CRC16_Check_Sum(_data.bytes, LEN)) {
            _protoData.fire = _data.content.fire;
            _protoData.control = _data.content.control;
            _protoData.yaw = _data.content.yaw;
            _protoData.yawVel = _data.content.yawVel;
            _protoData.yawAcc = _data.content.yawAcc;
            _protoData.pitch = _data.content.pitch;
            _protoData.pitchVel = _data.content.pitchVel;
            _protoData.pitchAcc = _data.content.pitchAcc;
        } else {
            LOG::warn("Vision", "wrong SOF or CRC!");
        }
    }
};