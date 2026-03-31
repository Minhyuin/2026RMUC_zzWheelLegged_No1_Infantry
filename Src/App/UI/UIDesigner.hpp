#pragma once

#include "./UIClient.hpp"
#include <cstdint>
/* need to config */
static constexpr uint8_t DYNAIMIC_NUM = 14;
static constexpr uint8_t STATIC_NUM = 14;

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

        // shooter
        UI::Info_s fricSwitch;
        UI::Info_s armorLoaderState;

        // other
        UI::Info_s enemyHP;
        UI::Info_s drivingReferenceLineLeft;  // 左行车线
        UI::Info_s drivingReferenceLineRight; // 右行车线
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
        UI::Info_s enemyHPBackground; // 敌方HP背景
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
