#pragma once

#include "./UIClient.hpp"
#include <cstdint>
/* need to config */
static constexpr uint8_t DYNAIMIC_NUM = 13;
static constexpr uint8_t STATIC_NUM = 13;

// 超电
static constexpr uint32_t CAP_ENERGY_ARC_START_ENG = 60;
static constexpr uint32_t CAP_ENERGY_ARC_END_ENG = 119;

// 底盘能量
static constexpr uint32_t CHASSIS_ENERGY_ARC_START_ENG = 60;
static constexpr uint32_t CHASSIS_ENERGY_ARC_END_ENG = 118;

// 真实弹道
static constexpr uint32_t REAL_AIM_X = 960;

// 速度弧线
static constexpr float UI_CHASSIS_SPD_MAX = 4.f;
static constexpr uint32_t CHASSIS_SPD_ARC_START_X = 870;
static constexpr uint32_t CHASSIS_SPD_ARC_START_Y = 540;
static constexpr uint32_t CHASSIS_SPD_ARC_END_X = 500;
static constexpr uint32_t CHASSIS_SPD_ARC_END_Y = 500;
static constexpr uint32_t CHASSIS_SPD_ARC_START_ANG = 230;
static constexpr uint32_t CHASSIS_SPD_ARC_END_ANG = 310;

// 云台俯仰角
static constexpr uint32_t PITCH_ARC_START_X = 900;
static constexpr uint32_t PITCH_ARC_START_Y = 540;
static constexpr uint32_t PITCH_ARC_END_X = 480;
static constexpr uint32_t PITCH_ARC_END_Y = 480;
static constexpr uint32_t PITCH_ARC_START_ANG = 240;
static constexpr uint32_t PITCH_ARC_END_ANG = 300;

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

// 敌方血量
static constexpr uint32_t MAX_HP_LEFT_POS_X = 959 - 300;
static constexpr uint32_t MAX_HP_RIGHT_POS_X = 961 + 300;

union DynamicUIInfos_u {
    struct {
        // chassis
        UI::Info_s chassisSpdSlider;
        UI::Info_s chassisSpdValue;
        UI::Info_s chassisState;
        UI::Info_s capEnergyArc;
        UI::Info_s chassisEnergyArc;

        // gimbal
        UI::Info_s visionState;
        UI::Info_s pitchAngSlider;
        UI::Info_s pitchAngValue;
        UI::Info_s gimbalState;
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
        UI::Info_s chassisString;              // 底盘当前状态
        UI::Info_s chassisVelArcBackground;    // 底盘速度背景弧线
        UI::Info_s capEnergyBottomLine;        // 电容能量下限指示线
        UI::Info_s chassisEnergyBottomLine;    // 底盘能量下限指示线
        UI::Info_s capEnergyArcBackground;     // 电容能量管
        UI::Info_s chassisEnergyArcBackground; // 底盘能量管

        // gimbal
        UI::Info_s gimbalString;          // 云台当前状态
        UI::Info_s pitchAngArcBackground; // 俯仰角度背景弧线

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
