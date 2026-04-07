#include "UIProtocol.hpp"
#include <cstring>
#include "./UIDesigner.hpp"

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-designated-field-initializers"
#endif

using namespace UI;

// NOLINTBEGIN
// 兼容RM_UI_Designer

void dynamicConfig(DynamicUIInfos_u &_info)
{
    auto &chassisSpdSlider = _info.chassisSpdSlider.config;
    chassisSpdSlider = {};
    chassisSpdSlider.priority = UI::Priority_e::LOW;
    chassisSpdSlider.uiType = Type_e::CIRCLE;
    chassisSpdSlider.operateType = OperateType_e::MODIFY;
    chassisSpdSlider.layer = 5;
    chassisSpdSlider.color = Color_e::RED_BLUE;
    chassisSpdSlider.width = 5;
    chassisSpdSlider.startX = 369;
    chassisSpdSlider.startY = 544;
    chassisSpdSlider.radius = 19;

    auto &chassisSpdValue = _info.chassisSpdValue.config;
    chassisSpdValue = {};
    chassisSpdValue.priority = UI::Priority_e::LOW;
    chassisSpdValue.uiType = Type_e::FLOAT;
    chassisSpdValue.operateType = OperateType_e::MODIFY;
    chassisSpdValue.layer = 1;
    chassisSpdValue.color = Color_e::CYAN_BLUE;
    chassisSpdValue.width = 2;
    chassisSpdValue.startX = 465;
    chassisSpdValue.startY = 882;
    chassisSpdValue.floatNum = 0.f;
    chassisSpdValue.decimal = 1;
    chassisSpdValue.size = 15;

    auto &chassisState = _info.chassisState.config;
    chassisState = {};
    chassisState.priority = UI::Priority_e::MID;
    chassisState.uiType = Type_e::CHAR;
    chassisState.operateType = OperateType_e::MODIFY;
    chassisState.layer = 3;
    chassisState.color = Color_e::ORANGE;
    chassisState.width = 2;
    chassisState.startX = 140;
    chassisState.startY = 840;
    chassisState.size = 20;
    std::strncpy(chassisState.text, "STOP", sizeof(chassisState.text) - 1);

    auto &capEnergyArc = _info.capEnergyArc.config;
    capEnergyArc = {};
    capEnergyArc.priority = UI::Priority_e::MID;
    capEnergyArc.uiType = Type_e::ARC;
    capEnergyArc.operateType = OperateType_e::MODIFY;
    capEnergyArc.layer = 2;
    capEnergyArc.color = Color_e::WHITE;
    capEnergyArc.width = 20;
    capEnergyArc.startX = 1108;
    capEnergyArc.startY = 540;
    capEnergyArc.endX = 425;
    capEnergyArc.endY = 425;
    capEnergyArc.startAngle = CAP_ENERGY_ARC_START_ENG;
    capEnergyArc.endAngle = CAP_ENERGY_ARC_END_ENG;

    auto &chassisEnergyArc = _info.chassisEnergyArc.config;
    chassisEnergyArc = {};
    chassisEnergyArc.priority = UI::Priority_e::LOW;
    chassisEnergyArc.uiType = Type_e::ARC;
    chassisEnergyArc.operateType = OperateType_e::MODIFY;
    chassisEnergyArc.layer = 3;
    chassisEnergyArc.color = Color_e::ORANGE;
    chassisEnergyArc.width = 20;
    chassisEnergyArc.startX = 1286;
    chassisEnergyArc.startY = 540;
    chassisEnergyArc.endX = 203;
    chassisEnergyArc.endY = 203;
    chassisEnergyArc.startAngle = CHASSIS_ENERGY_ARC_START_ENG;
    chassisEnergyArc.endAngle = CHASSIS_ENERGY_ARC_END_ENG;

    auto &chassisDirection = _info.chassisDirection.config;
    chassisDirection = {};
    chassisDirection.priority = UI::Priority_e::MID;
    chassisDirection.uiType = Type_e::CIRCLE;
    chassisDirection.operateType = OperateType_e::MODIFY;
    chassisDirection.layer = 5;
    chassisDirection.color = Color_e::RED_BLUE;
    chassisDirection.width = 5;
    chassisDirection.startX = 369;
    chassisDirection.startY = 544;
    chassisDirection.radius = 8;

    auto &legLengthValue = _info.legLengthValue.config;
    legLengthValue = {};
    legLengthValue.priority = UI::Priority_e::LOW;
    legLengthValue.uiType = Type_e::FLOAT;
    legLengthValue.operateType = OperateType_e::MODIFY;
    legLengthValue.layer = 1;
    legLengthValue.color = Color_e::ORANGE;
    legLengthValue.width = 2;
    legLengthValue.startX = 940 - 200;
    legLengthValue.startY = 40;
    legLengthValue.floatNum = 0.f;
    legLengthValue.decimal = 2;
    legLengthValue.size = 15;

    auto &chassisRollValue = _info.chassisRollValue.config;
    chassisRollValue = {};
    chassisRollValue.priority = UI::Priority_e::LOW;
    chassisRollValue.uiType = Type_e::FLOAT;
    chassisRollValue.operateType = OperateType_e::MODIFY;
    chassisRollValue.layer = 1;
    chassisRollValue.color = Color_e::ORANGE;
    chassisRollValue.width = 2;
    chassisRollValue.startX = 940 + 200;
    chassisRollValue.startY = 40;
    chassisRollValue.floatNum = 0.f;
    chassisRollValue.decimal = 3;
    chassisRollValue.size = 15;

    auto &visionState = _info.visionState.config;
    visionState = {};
    visionState.priority = UI::Priority_e::MID;
    visionState.uiType = Type_e::RECTANGEL;
    visionState.operateType = OperateType_e::MODIFY;
    visionState.layer = 2;
    visionState.color = Color_e::WHITE;
    visionState.width = 1;
    visionState.startX = 960 - 300;
    visionState.startY = 540 - 200;
    visionState.endX = 960 + 300;
    visionState.endY = 540 + 200;
    visionState.startAngle = 60;
    visionState.endAngle = 120;

    auto &pitchAngSlider = _info.pitchAngSlider.config;
    pitchAngSlider = {};
    pitchAngSlider.priority = UI::Priority_e::LOW;
    pitchAngSlider.uiType = Type_e::CIRCLE;
    pitchAngSlider.operateType = OperateType_e::MODIFY;
    pitchAngSlider.layer = 5;
    pitchAngSlider.color = Color_e::RED_BLUE;
    pitchAngSlider.width = 5;
    pitchAngSlider.startX = 421;
    pitchAngSlider.startY = 545;
    pitchAngSlider.radius = 8;

    auto &pitchAngValue = _info.pitchAngValue.config;
    pitchAngValue = {};
    pitchAngValue.priority = UI::Priority_e::LOW;
    pitchAngValue.uiType = Type_e::FLOAT;
    pitchAngValue.operateType = OperateType_e::MODIFY;
    pitchAngValue.layer = 1;
    pitchAngValue.color = Color_e::GREEN;
    pitchAngValue.width = 2;
    pitchAngValue.startX = 472;
    pitchAngValue.startY = 809;
    pitchAngValue.floatNum = 0.f;
    pitchAngValue.decimal = 4;
    pitchAngValue.size = 15;

    auto &gimbalState = _info.gimbalState.config;
    gimbalState = {};
    gimbalState.priority = UI::Priority_e::MID;
    gimbalState.uiType = Type_e::CHAR;
    gimbalState.operateType = OperateType_e::MODIFY;
    gimbalState.layer = 3;
    gimbalState.color = Color_e::ORANGE;
    gimbalState.width = 2;
    gimbalState.startX = 140;
    gimbalState.startY = 800;
    gimbalState.size = 20;
    std::strncpy(gimbalState.text, "STOP", sizeof(gimbalState.text) - 1);

    auto &fricSwitch = _info.fricSwitch.config;
    fricSwitch = {};
    fricSwitch.priority = UI::Priority_e::MID;
    fricSwitch.uiType = Type_e::LINE;
    fricSwitch.operateType = OperateType_e::MODIFY;
    fricSwitch.layer = 3;
    fricSwitch.color = Color_e::RED_BLUE;
    fricSwitch.width = 8;
    fricSwitch.startX = 906;
    fricSwitch.startY = 485;
    fricSwitch.endX = 1014;
    fricSwitch.endY = 594;

    auto &armorLoaderState = _info.armorLoaderState.config;
    armorLoaderState = {};
    armorLoaderState.priority = UI::Priority_e::MID;
    armorLoaderState.uiType = Type_e::CHAR;
    armorLoaderState.operateType = OperateType_e::MODIFY;
    armorLoaderState.layer = 3;
    armorLoaderState.color = Color_e::ORANGE;
    armorLoaderState.width = 2;
    armorLoaderState.startX = 140;
    armorLoaderState.startY = 760;
    armorLoaderState.size = 20;
    std::strncpy(armorLoaderState.text, "STOP", sizeof(armorLoaderState.text) - 1);

    auto &drivingReferenceLineLeft = _info.drivingReferenceLineLeft.config;
    drivingReferenceLineLeft = {};
    drivingReferenceLineLeft.priority = UI::Priority_e::LOW;
    drivingReferenceLineLeft.uiType = Type_e::LINE;
    drivingReferenceLineLeft.operateType = OperateType_e::MODIFY;
    drivingReferenceLineLeft.layer = 4;
    drivingReferenceLineLeft.color = Color_e::GREEN;
    drivingReferenceLineLeft.width = 3;
    drivingReferenceLineLeft.startX = DOWN_MOVE_LEFT_POS_START_X;
    drivingReferenceLineLeft.startY = DOWN_MOVE_LEFT_POS_START_Y;
    drivingReferenceLineLeft.endX = DOWN_MOVE_LEFT_POS_END_X;
    drivingReferenceLineLeft.endY = DOWN_MOVE_LEFT_POS_END_Y;

    auto &drivingReferenceLineRight = _info.drivingReferenceLineRight.config;
    drivingReferenceLineRight = {};
    drivingReferenceLineRight.priority = UI::Priority_e::LOW;
    drivingReferenceLineRight.uiType = Type_e::LINE;
    drivingReferenceLineRight.operateType = OperateType_e::MODIFY;
    drivingReferenceLineRight.layer = 4;
    drivingReferenceLineRight.color = Color_e::GREEN;
    drivingReferenceLineRight.width = 3;
    drivingReferenceLineRight.startX = DOWN_MOVE_RIGHT_POS_START_X;
    drivingReferenceLineRight.startY = DOWN_MOVE_RIGHT_POS_START_Y;
    drivingReferenceLineRight.endX = DOWN_MOVE_RIGHT_POS_END_X;
    drivingReferenceLineRight.endY = DOWN_MOVE_RIGHT_POS_END_Y;
}

void staticConfig(StaticUIInfos_u &_info)
{
    auto &chassisString = _info.chassisString.config;
    chassisString = {};
    chassisString.uiType = Type_e::CHAR;
    chassisString.layer = 4;
    chassisString.color = Color_e::GREEN;
    chassisString.width = 2;
    chassisString.startX = 76;
    chassisString.startY = 840;
    chassisString.size = 20;
    std::strncpy(chassisString.text, "C:", sizeof(chassisString.text) - 1);

    auto &chassisVelArcBackground = _info.chassisVelArcBackground.config;
    chassisVelArcBackground = {};
    chassisVelArcBackground.uiType = Type_e::ARC;
    chassisVelArcBackground.layer = 3;
    chassisVelArcBackground.color = Color_e::CYAN_BLUE;
    chassisVelArcBackground.width = 3;
    chassisVelArcBackground.startX = CHASSIS_SPD_ARC_START_X;
    chassisVelArcBackground.startY = CHASSIS_SPD_ARC_START_Y;
    chassisVelArcBackground.endX = CHASSIS_SPD_ARC_END_X;
    chassisVelArcBackground.endY = CHASSIS_SPD_ARC_END_Y;
    chassisVelArcBackground.startAngle = CHASSIS_SPD_ARC_START_ANG;
    chassisVelArcBackground.endAngle = CHASSIS_SPD_ARC_END_ANG;

    auto &capEnergyBottomLine = _info.capEnergyBottomLine.config;
    capEnergyBottomLine = {};
    capEnergyBottomLine.uiType = Type_e::LINE;
    capEnergyBottomLine.layer = 0;
    capEnergyBottomLine.color = Color_e::WHITE;
    capEnergyBottomLine.width = 8;
    capEnergyBottomLine.startX = 1487;
    capEnergyBottomLine.startY = 329;
    capEnergyBottomLine.endX = 1454;
    capEnergyBottomLine.endY = 352;

    auto &chassisEnergyBottomLine = _info.chassisEnergyBottomLine.config;
    chassisEnergyBottomLine = {};
    chassisEnergyBottomLine.uiType = Type_e::LINE;
    chassisEnergyBottomLine.layer = 0;
    chassisEnergyBottomLine.color = Color_e::WHITE;
    chassisEnergyBottomLine.width = 8;
    chassisEnergyBottomLine.startX = 1476;
    chassisEnergyBottomLine.startY = 433;
    chassisEnergyBottomLine.endX = 1440;
    chassisEnergyBottomLine.endY = 452;

    auto &capEnergyArcBackground = _info.capEnergyArcBackground.config;
    capEnergyArcBackground = {};
    capEnergyArcBackground.uiType = Type_e::ARC;
    capEnergyArcBackground.layer = 0;
    capEnergyArcBackground.color = Color_e::WHITE;
    capEnergyArcBackground.width = 5;
    capEnergyArcBackground.startX = 1100;
    capEnergyArcBackground.startY = 540;
    capEnergyArcBackground.endX = 445;
    capEnergyArcBackground.endY = 445;
    capEnergyArcBackground.startAngle = 64;
    capEnergyArcBackground.endAngle = 119;

    auto &chassisEnergyArcBackground = _info.chassisEnergyArcBackground.config;
    chassisEnergyArcBackground = {};
    chassisEnergyArcBackground.uiType = Type_e::ARC;
    chassisEnergyArcBackground.layer = 0;
    chassisEnergyArcBackground.color = Color_e::WHITE;
    chassisEnergyArcBackground.width = 5;
    chassisEnergyArcBackground.startX = 1296;
    chassisEnergyArcBackground.startY = 540;
    chassisEnergyArcBackground.endX = 206;
    chassisEnergyArcBackground.endY = 206;
    chassisEnergyArcBackground.startAngle = 62;
    chassisEnergyArcBackground.endAngle = 120;

    auto &legLengthAndChassisRollString = _info.legLengthAndChassisRollString.config;
    legLengthAndChassisRollString = {};
    legLengthAndChassisRollString.uiType = Type_e::CHAR;
    legLengthAndChassisRollString.layer = 4;
    legLengthAndChassisRollString.color = Color_e::GREEN;
    legLengthAndChassisRollString.width = 2;
    legLengthAndChassisRollString.startX = 960 - 50;
    legLengthAndChassisRollString.startY = 40;
    legLengthAndChassisRollString.size = 15;
    std::strncpy(legLengthAndChassisRollString.text, "LEG/ROLL", sizeof(legLengthAndChassisRollString.text) - 1);

    auto &gimbalString = _info.gimbalString.config;
    gimbalString = {};
    gimbalString.uiType = Type_e::CHAR;
    gimbalString.layer = 4;
    gimbalString.color = Color_e::GREEN;
    gimbalString.width = 2;
    gimbalString.startX = 76;
    gimbalString.startY = 800;
    gimbalString.size = 20;
    std::strncpy(gimbalString.text, "G:", sizeof(gimbalString.text) - 1);

    auto &pitchAngArcBackground = _info.pitchAngArcBackground.config;
    pitchAngArcBackground = {};
    pitchAngArcBackground.uiType = Type_e::ARC;
    pitchAngArcBackground.layer = 3;
    pitchAngArcBackground.color = Color_e::GREEN;
    pitchAngArcBackground.width = 3;
    pitchAngArcBackground.startX = 900;
    pitchAngArcBackground.startY = 540;
    pitchAngArcBackground.endX = 480;
    pitchAngArcBackground.endY = 480;
    pitchAngArcBackground.startAngle = 240;
    pitchAngArcBackground.endAngle = 300;

    auto &shooterString = _info.shooterString.config;
    shooterString = {};
    shooterString.uiType = Type_e::CHAR;
    shooterString.layer = 4;
    shooterString.color = Color_e::GREEN;
    shooterString.width = 2;
    shooterString.startX = 76;
    shooterString.startY = 760;
    shooterString.size = 20;
    std::strncpy(shooterString.text, "S:", sizeof(shooterString.text) - 1);

    auto &crosshairVertical = _info.crosshairVertical.config;
    crosshairVertical = {};
    crosshairVertical.uiType = Type_e::LINE;
    crosshairVertical.layer = 5;
    crosshairVertical.color = Color_e::GREEN;
    crosshairVertical.width = 1;
    crosshairVertical.startX = REAL_AIM_X;
    crosshairVertical.startY = 540;
    crosshairVertical.endX = REAL_AIM_X;
    crosshairVertical.endY = 540 - 100 - 300;

    auto &crosshairHorizontal0 = _info.crosshairHorizontal0.config;
    crosshairHorizontal0 = {};
    crosshairHorizontal0.uiType = Type_e::CIRCLE;
    crosshairHorizontal0.layer = 5;
    crosshairHorizontal0.color = Color_e::RED_BLUE;
    crosshairHorizontal0.width = 10;
    crosshairHorizontal0.startX = REAL_AIM_X;
    crosshairHorizontal0.startY = 540;
    crosshairHorizontal0.radius = 3;

    auto &crosshairHorizontal1 = _info.crosshairHorizontal1.config;
    crosshairHorizontal1 = {};
    crosshairHorizontal1.uiType = Type_e::LINE;
    crosshairHorizontal1.layer = 5;
    crosshairHorizontal1.color = Color_e::GREEN;
    crosshairHorizontal1.width = 1;
    crosshairHorizontal1.startX = REAL_AIM_X - 40;
    crosshairHorizontal1.startY = 540 - 80;
    crosshairHorizontal1.endX = REAL_AIM_X + 40;
    crosshairHorizontal1.endY = 540 - 80;

    auto &crosshairHorizontal2 = _info.crosshairHorizontal2.config;
    crosshairHorizontal2 = {};
    crosshairHorizontal2.uiType = Type_e::LINE;
    crosshairHorizontal2.layer = 5;
    crosshairHorizontal2.color = Color_e::GREEN;
    crosshairHorizontal2.width = 1;
    crosshairHorizontal2.startX = REAL_AIM_X - 30;
    crosshairHorizontal2.startY = 540 - 90;
    crosshairHorizontal2.endX = REAL_AIM_X + 30;
    crosshairHorizontal2.endY = 540 - 90;

    auto &crosshairHorizontal3 = _info.crosshairHorizontal3.config;
    crosshairHorizontal3 = {};
    crosshairHorizontal3.uiType = Type_e::LINE;
    crosshairHorizontal3.layer = 5;
    crosshairHorizontal3.color = Color_e::GREEN;
    crosshairHorizontal3.width = 1;
    crosshairHorizontal3.startX = REAL_AIM_X - 20;
    crosshairHorizontal3.startY = 540 - 100;
    crosshairHorizontal3.endX = REAL_AIM_X + 20;
    crosshairHorizontal3.endY = 540 - 100;
}