#pragma once

#include "./UIClient.hpp"
#include <cstdint>
/* need to config */
static constexpr uint8_t DYNAIMIC_NUM = 14;
static constexpr uint8_t STATIC_NUM = 14;

// 超电
static constexpr uint8_t CAP_ENERGY_BAR_WIDTH = 240;
static constexpr uint32_t CAP_ENERGY_BAR_START_X = 960 - (CAP_ENERGY_BAR_WIDTH / 2);
static constexpr uint32_t CAP_ENERGY_BAR_END_X = 960 + (CAP_ENERGY_BAR_WIDTH / 2);

// 真实弹道
static constexpr uint32_t REAL_AIM_X = 960;
static constexpr uint32_t REAL_AIM_Y = 540;

// 底盘方向
static constexpr uint32_t CHASSIS_DIRECTION_RADIUS = 80;

// 行车线
// 抬头到极限角度的参数
static constexpr float UI_PITCH_START_ANG = -0.6f;
static constexpr uint32_t UP_MOVE_LEFT_POS_START_X = 0 + 400;
static constexpr uint32_t UP_MOVE_LEFT_POS_START_Y = 0;
static constexpr uint32_t UP_MOVE_LEFT_POS_END_X = 0 + 640;
static constexpr uint32_t UP_MOVE_LEFT_POS_END_Y = 50;
static constexpr uint32_t UP_MOVE_RIGHT_POS_START_X = 1920 - 400;
static constexpr uint32_t UP_MOVE_RIGHT_POS_START_Y = 0;
static constexpr uint32_t UP_MOVE_RIGHT_POS_END_X = 1920 - 640;
static constexpr uint32_t UP_MOVE_RIGHT_POS_END_Y = 50;

// 低头到最极限角度的参数
static constexpr float UI_PITCH_END_ANG = 0.4f;
static constexpr uint32_t DOWN_MOVE_LEFT_POS_START_X = 0;
static constexpr uint32_t DOWN_MOVE_LEFT_POS_START_Y = 510;
static constexpr uint32_t DOWN_MOVE_LEFT_POS_END_X = 0 + 330;
static constexpr uint32_t DOWN_MOVE_LEFT_POS_END_Y = 580;
static constexpr uint32_t DOWN_MOVE_RIGHT_POS_START_X = 1920;
static constexpr uint32_t DOWN_MOVE_RIGHT_POS_START_Y = 510;
static constexpr uint32_t DOWN_MOVE_RIGHT_POS_END_X = 1920 - 330;
static constexpr uint32_t DOWN_MOVE_RIGHT_POS_END_Y = 580;

// 小陀螺指示
static constexpr uint32_t UI_SPINNING_DEG = 30;

union DynamicUIInfos_u {
    struct {
        // chassis
        UI::Info_s chassisState;
        UI::Info_s noChassisLine;
        UI::Info_s capEnergyBar;
        UI::Info_s chassisDirection;
        UI::Info_s legLengthValue;
        UI::Info_s chassisRollValue;

        // gimbal
        UI::Info_s visionState;
        UI::Info_s gimbalState;
        UI::Info_s pitchAngValue;
        UI::Info_s pitchTargetOffsetAngValue;
        UI::Info_s drivingReferenceLineLeft;  // 左行车线
        UI::Info_s drivingReferenceLineRight; // 右行车线

        // shooter
        UI::Info_s fricSwitch;
        UI::Info_s armorLoaderState;
    };
    UI::Info_s list[DYNAIMIC_NUM];
};

union StaticUIInfos_u {
    struct {
        // chassis
        UI::Info_s chassisString;                 // 底盘当前状态
        UI::Info_s capEnergyBottomLine;           // 电容能量下限指示线
        UI::Info_s capEnergyBarBackground;        // 电容能量管
        UI::Info_s legLengthAndChassisRollString; // 腿长和底盘滚转角
        UI::Info_s allowClimbLine1;               // 允许攀爬线 - 下
        UI::Info_s allowClimbLine2;               // 允许攀爬线 - 上

        // gimbal
        UI::Info_s gimbalString;        // 云台当前状态
        UI::Info_s pitchOffsetBrackets; // 俯仰偏移值括号

        // shooter
        UI::Info_s shooterString;

        // other
        UI::Info_s crosshairVertical; // 十字准星竖线
        UI::Info_s crosshairHorizontal0;
        UI::Info_s crosshairHorizontal1;
        UI::Info_s crosshairHorizontal2;
        UI::Info_s crosshairHorizontal3;
    };
    UI::Info_s list[STATIC_NUM];
};

void dynamicConfig(DynamicUIInfos_u &_info);
void staticConfig(StaticUIInfos_u &_info);
