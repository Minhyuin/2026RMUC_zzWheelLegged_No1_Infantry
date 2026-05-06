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
        float vx;    // must be in [-3.2, 3.1]
        float vy;    // must be in [-3.2, 3.1]
        float dl;    // must be in [-0.16, 0.15]
        float droll; // must be in [-0.8, 0.7]
        bool needJump;
        bool needClimb;
        bool needStretch;
        bool needRush;
        bool bodyReset;
        float deltaYaw;
        bool uiReset;
    };
#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            bool gimbalReset : 1;
            int8_t vx : 6;    // [-32, 31]
            int8_t vy : 6;    // [-32, 31]
            int8_t dl : 5;    // [-16, 15]
            int8_t droll : 4; // [-8, 7]
            uint8_t needJump : 1;
            uint8_t needClimb : 1;
            uint8_t needStretch : 1;
            uint8_t needRush : 1;
            int16_t deltaYaw : 16;
            uint8_t chassisExptState : 4;
            uint8_t gimbalRealState : 4;
            uint8_t uiReset : 1;
            uint16_t reversed : 14;
        } content;
        uint8_t bytes[LEN];
    } data;
#pragma pack(pop)
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.gimbalReset = _protoData.gimbalReset;
        d.vx = static_cast<int8_t>(_protoData.vx * 10.f);
        d.vy = static_cast<int8_t>(_protoData.vy * 10.f);
        d.dl = static_cast<int8_t>(_protoData.dl * 100.f);
        d.droll = static_cast<int8_t>(_protoData.droll * 10.f);
        d.needJump = _protoData.needJump;
        d.needClimb = _protoData.needClimb;
        d.needStretch = _protoData.needStretch;
        d.needRush = _protoData.needRush;
        d.chassisExptState = _protoData.chassisExptState;
        d.gimbalRealState = _protoData.gimbalRealState;
        d.deltaYaw = convertToInt16(_protoData.deltaYaw);
        d.uiReset = _protoData.uiReset;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.gimbalReset = _data.content.gimbalReset;
        _protoData.vx = static_cast<float>(_data.content.vx) / 10.f;
        _protoData.vy = static_cast<float>(_data.content.vy) / 10.f;
        _protoData.dl = static_cast<float>(_data.content.dl) / 100.f;
        _protoData.droll = static_cast<float>(_data.content.droll) / 10.f;
        _protoData.needJump = _data.content.needJump;
        _protoData.needClimb = _data.content.needClimb;
        _protoData.needStretch = _data.content.needStretch;
        _protoData.needRush = _data.content.needRush;
        _protoData.chassisExptState = _data.content.chassisExptState;
        _protoData.gimbalRealState = _data.content.gimbalRealState;
        _protoData.deltaYaw = convertToFloat(_data.content.deltaYaw);
        _protoData.uiReset = _data.content.uiReset;
    }
};

class ArmorLoaderCtrlPacketType {
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

class GimbalStatePacketType {
public:
    static constexpr uint8_t LEN = 8;
    static constexpr uint16_t ID = 0x23;
    struct ProtoData_s {
        uint8_t gimbalRealState;
        bool visionOnline;
        bool aiming;
        float pitch;
    };
    union Data_u {
        struct Data_s {
            uint8_t gimbalRealState : 4;
            uint8_t visionOnline : 1;
            uint8_t aiming : 1;
            int16_t pitch : 16;
            uint32_t reversed1 : 10;
            uint32_t reversed2 : 32;
        } content;
        uint8_t bytes[LEN];
    } data;
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.gimbalRealState = _protoData.gimbalRealState;
        d.visionOnline = _protoData.visionOnline;
        d.aiming = _protoData.aiming;
        d.pitch = convertToInt16(_protoData.pitch);
        d.reversed1 = 0;
        d.reversed2 = 0;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.gimbalRealState = _data.content.gimbalRealState;
        _protoData.visionOnline = _data.content.visionOnline;
        _protoData.aiming = _data.content.aiming;
        _protoData.pitch = convertToFloat(_data.content.pitch);
    }
};

class ArmorBoosterStatePacketType {
public:
    static constexpr uint8_t LEN = 4;
    static constexpr uint16_t ID = 0x24;
    struct ProtoData_s {
        uint8_t armorBoosterRealState;
    };
    union Data_u {
        struct Data_s {
            uint8_t armorBoosterRealState : 4;
            uint32_t reversed : 28;
        } content;
        uint8_t bytes[LEN];
    } data;
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.armorBoosterRealState = _protoData.armorBoosterRealState;
        d.reversed = 0;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.armorBoosterRealState = _data.content.armorBoosterRealState;
    }
};

class RefereePacketType {
public:
    static constexpr uint8_t LEN = 8;
    static constexpr uint16_t ID = 0x25;
    struct ProtoData_s {
        uint8_t color;
        bool isPlay;
        bool isAlive;
        uint16_t stageRemainTime; // [0 - 1023]
        uint8_t rateOfFire;       // [0 - 63], unit:Hz
        float muzzleVel = 20.f;   // [0 - 32.767] unit:m/s
    };
    union Data_u {
        struct Data_s {
            uint8_t color : 1; // 0 - red, 1 - blue
            uint8_t isPlay : 1;
            uint8_t isAlive : 1;
            uint16_t stageRemainTime : 10; // [0 - 1023]
            uint8_t rateOfFire : 6;        // [0 - 63]
            uint16_t muzzleVel : 15;       // [0 - 32767]
            uint32_t reversed1 : 11;
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
        d.rateOfFire = _protoData.rateOfFire;
        d.muzzleVel = static_cast<uint16_t>(_protoData.muzzleVel * 1000.f);
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.color = _data.content.color;
        _protoData.isPlay = _data.content.isPlay;
        _protoData.isAlive = _data.content.isAlive;
        _protoData.stageRemainTime = _data.content.stageRemainTime;
        _protoData.rateOfFire = _data.content.rateOfFire;
        _protoData.muzzleVel = static_cast<float>(_data.content.muzzleVel) / 1000.f;
    }
};

class GimbalLaunchStrategyPacketType {
public:
    static constexpr uint8_t LEN = 8;
    static constexpr uint16_t ID = 0x26;
    struct ProtoData_s {
        bool isLegLFront;
        bool isLegRFront;
        bool isNeedGimbalLaunch;
        bool isAllowExitLaunch;
    };
    union Data_u {
        struct Data_s {
            uint8_t isLegLFront : 1;
            uint8_t isLegRFront : 1;
            uint8_t isNeedGimbalLaunch : 1;
            uint8_t isAllowExitLaunch : 1;
            uint32_t reversed1 : 28;
            uint32_t reversed2 : 32;
        } content;
        uint8_t bytes[LEN];
    } data;
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.isLegLFront = _protoData.isLegLFront;
        d.isLegRFront = _protoData.isLegRFront;
        d.isNeedGimbalLaunch = _protoData.isNeedGimbalLaunch;
        d.isAllowExitLaunch = _protoData.isAllowExitLaunch;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.isLegLFront = _data.content.isLegLFront;
        _protoData.isLegRFront = _data.content.isLegRFront;
        _protoData.isNeedGimbalLaunch = _data.content.isNeedGimbalLaunch;
        _protoData.isAllowExitLaunch = _data.content.isAllowExitLaunch;
    }
};

class ChassisStatePacketType {
public:
    static constexpr uint8_t LEN = 4;
    static constexpr uint16_t ID = 0x27;
    struct ProtoData_s {
        uint8_t chassisRealState;
    };
    union Data_u {
        struct Data_s {
            uint8_t chassisRealState : 4;
            uint32_t reversed : 28;
        } content;
        uint8_t bytes[LEN];
    } data;
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.chassisRealState = _protoData.chassisRealState;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.chassisRealState = _data.content.chassisRealState;
    }
};
} // namespace COMM
