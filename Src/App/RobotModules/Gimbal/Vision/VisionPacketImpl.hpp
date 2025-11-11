#pragma once

#include <cstdint>
#include "Crc.hpp"

class VisionTxPacket {
public:
    static constexpr uint8_t LEN = (272 / 8); // bits to bytes
    static constexpr uint16_t ID = 0x5A;
    struct ProtoData_s {
        uint8_t detectColor;
        uint8_t taskMode;
        bool resetTracker;
        bool isPlay;
        bool changeTarget;
        float roll;
        float pitch;
        float yaw;
        uint16_t gameTime; // s [0,450]
        uint32_t timestamp;
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
            float aimX;        // unuse
            float aimY;        // unuse
            float aimZ;        // unuse
            uint16_t gameTime; // s [0,450]
            uint32_t timestamp;
            uint16_t checksum;
        } content;
        uint8_t bytes[LEN];
    } data;
#pragma pack(pop)
    static Data_u compress(const ProtoData_s &_protoData)
    {
        Data_u data{};
        Data_u::Data_s &d = data.content;
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
        d.aimX = 0;
        d.aimY = 0;
        d.aimZ = 0;
        d.gameTime = _protoData.gameTime;
        d.timestamp = _protoData.timestamp;
        d.checksum = Get_CRC16_Check_Sum(data.bytes, LEN - 2, CRC16_INIT);
        return data;
    }
    static ProtoData_s decompress(const Data_u &_data)
    {
        ProtoData_s protoData{}; // unsupported
        return protoData;
    }
};

class VisionRxPacket {
public:
    static constexpr uint8_t LEN = (208 / 8); // bits to bytes
    static constexpr uint16_t ID = 0xA5;
    struct ProtoData_s {
        uint8_t fire : 1;
        float targetYaw;
        float targetPitch;
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
            uint8_t fire; // 0-false 1-true
            float targetYaw;
            float targetPitch;
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
    static Data_u compress(const ProtoData_s &_protoData)
    {
        Data_u data{}; // unsupported
        return data;
    }
    static ProtoData_s decompress(const Data_u &_data)
    {
        ProtoData_s protoData{};
        // if (_data.content.header == ID &&
        //     Verify_CRC16_Check_Sum(const_cast<uint8_t *>(_data.bytes),
        //                            LEN)) { // Don't safe but temporary
        //     protoData.fire = _data.content.fire;
        //     protoData.targetYaw = _data.content.targetYaw;
        //     protoData.targetPitch = _data.content.targetPitch;
        //     protoData.yaw = _data.content.yaw;
        //     protoData.yawVel = _data.content.yawVel;
        //     protoData.yawAcc = _data.content.yawAcc;
        //     protoData.pitch = _data.content.pitch;
        //     protoData.pitchVel = _data.content.pitchVel;
        //     protoData.pitchAcc = _data.content.pitchAcc;
        // }
        protoData.fire = _data.content.fire;
        protoData.targetYaw = _data.content.targetYaw;
        protoData.targetPitch = _data.content.targetPitch;
        protoData.yaw = _data.content.yaw;
        protoData.yawVel = _data.content.yawVel;
        protoData.yawAcc = _data.content.yawAcc;
        protoData.pitch = _data.content.pitch;
        protoData.pitchVel = _data.content.pitchVel;
        protoData.pitchAcc = _data.content.pitchAcc;
        return protoData;
    }
};