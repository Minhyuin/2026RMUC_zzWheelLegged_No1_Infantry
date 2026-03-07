#pragma once

#include "./utils.hpp"
#include <cstring>

namespace COMM {

/**
 * @brief Sample Packet (Gimbal -> Chassis)
 * 
 */

class SamplePacketType {
public:
    static constexpr uint8_t LEN = 8;
    static constexpr uint16_t ID = 0x21;
    struct ProtoData_s {
        bool gimbalReset;
        float vx;
        float vy;
        float gimbalYaw;
    };
#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            bool gimbalReset : 1;
            int16_t vx : 16;
            int16_t vy : 16;
            int16_t gimbalYaw : 16;
            uint16_t reversed : 15;
        } content;
        uint8_t bytes[LEN];
    } data;
#pragma pack(pop)
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.gimbalReset = _protoData.gimbalReset;
        d.vx = static_cast<int16_t>(_protoData.vx * 1000.f);
        d.vy = static_cast<int16_t>(_protoData.vy * 1000.f);
        d.gimbalYaw = convertToInt16(_protoData.gimbalYaw);
        d.reversed = 0;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.gimbalReset = _data.content.gimbalReset;
        _protoData.vx = static_cast<float>(_data.content.vx) / 1000.f;
        _protoData.vy = static_cast<float>(_data.content.vy) / 1000.f;
        _protoData.gimbalYaw = convertToFloat(_data.content.gimbalYaw);
    }
};

class ChassisCtrlPacketType {
public:
    static constexpr uint8_t LEN = 8;
    static constexpr uint16_t ID = 0x21;
    struct ProtoData_s {
        bool gimbalReset;
        uint8_t chassisExptState;
        uint8_t gimbalRealState;
        float vx;
        float vy;
        float deltaYaw;
    };
#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            bool gimbalReset : 1;
            int16_t vx : 16;
            int16_t vy : 16;
            int16_t deltaYaw : 16;
            uint8_t chassisExptState : 4;
            uint8_t gimbalRealState : 4;
            uint16_t reversed : 7;
        } content;
        uint8_t bytes[LEN];
    } data;
#pragma pack(pop)
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.gimbalReset = _protoData.gimbalReset;
        d.vx = static_cast<int16_t>(_protoData.vx * 1000.f);
        d.vy = static_cast<int16_t>(_protoData.vy * 1000.f);
        d.chassisExptState = _protoData.chassisExptState;
        d.gimbalRealState = _protoData.gimbalRealState;
        d.deltaYaw = convertToInt16(_protoData.deltaYaw);
        d.reversed = 0;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.gimbalReset = _data.content.gimbalReset;
        _protoData.vx = static_cast<float>(_data.content.vx) / 1000.f;
        _protoData.vy = static_cast<float>(_data.content.vy) / 1000.f;
        _protoData.chassisExptState = _data.content.chassisExptState;
        _protoData.gimbalRealState = _data.content.gimbalRealState;
        _protoData.deltaYaw = convertToFloat(_data.content.deltaYaw);
    }
};

class ArmorLoaderPacketType {
public:
    static constexpr uint8_t LEN = 4;
    static constexpr uint16_t ID = 0x22;
    struct ProtoData_s {
        uint8_t armorLoaderExptState;
        bool fire;
    };
    union Data_u {
        struct Data_s {
            uint8_t armorLoaderExptState : 8;
            bool fire : 1;
            uint32_t reversed : 23;
        } content;
        uint8_t bytes[LEN];
    } data;
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.armorLoaderExptState = _protoData.armorLoaderExptState;
        d.fire = _protoData.fire;
        d.reversed = 0;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.armorLoaderExptState = _data.content.armorLoaderExptState;
        _protoData.fire = _data.content.fire;
    }
};

class RefereePacketType {
public:
    static constexpr uint8_t LEN = 8;
    static constexpr uint16_t ID = 0x23;
    struct ProtoData_s {
        uint8_t color;
        bool isPlay;
        bool isAlive;
        uint16_t stageRemainTime; // [0 - 1023]
    };
    union Data_u {
        struct Data_s {
            uint8_t color : 1; // 0 - red, 1 - blue
            uint8_t isPlay : 1;
            uint8_t isAlive : 1;
            uint16_t stageRemainTime : 10; // [0 - 1023]
            uint32_t reversed1 : 32;
            uint32_t reversed2 : 19;
        } content;
        uint8_t bytes[LEN];
    } data;
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.color = _protoData.color;
        d.isPlay = _protoData.isPlay;
        d.isAlive = _protoData.isAlive;
        d.stageRemainTime = _protoData.stageRemainTime;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.color = _data.content.color;
        _protoData.isPlay = _data.content.isPlay;
        _protoData.isAlive = _data.content.isAlive;
        _protoData.stageRemainTime = _data.content.stageRemainTime;
    }
};

} // namespace COMM
