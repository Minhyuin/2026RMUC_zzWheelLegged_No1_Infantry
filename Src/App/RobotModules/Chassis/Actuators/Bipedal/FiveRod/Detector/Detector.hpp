#pragma once

#include "INS.hpp"

#include "WheelLegged.hpp"

namespace CHASSIS {
class Detector {
    static constexpr float FN_THRESHOLD =
            0; // WheelLegged::MASS_WHEEL * WheelLegged::GRAVITY;

    static constexpr float STOP_THRESHOLD = 0.03f;

    static constexpr float DANG_FREEZE_THRESHOLD = 0.1f;

    static constexpr uint16_t FREEZE_MAX_TIME = 700; // 700ms

public:
    Detector() = default;

    bool isFloating(float _fn);

    bool isStop(float _expVel, float _curVel);

    bool detectTumble(const LegState_s &_legL, const LegState_s &_legR,
                      const INS_SYS::INSData_s &_ins);

    bool detectFreeze(const INS_SYS::INSData_s &_ins);

private:
    uint16_t freezeTime = 0;
};
} // namespace CHASSIS
