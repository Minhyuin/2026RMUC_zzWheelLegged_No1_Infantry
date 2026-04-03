#include <numbers>

#include "./UIApp.hpp"
#include "StmLog.hpp"
#include "UIDesigner.hpp"
#include "sdkconfig.h"
#include "magic_enum/magic_enum.hpp"
#include "dsp/fast_math_functions.h"

using namespace UI;

extern UART_HandleTypeDef UI_UART;

APP::APP() : client_(UI_UART) {};

void APP::init()
{
    /* need to config */
    dynamicConfig(dynamicInfo_);
    staticConfig(constInfo_);

    // 初始化UI链表
    if (client_.initList(dynamicInfo_.list, DYNAIMIC_NUM, constInfo_.list, STATIC_NUM) == Status_e::ERROR) {
        LOG::error("UI", "List init failed");
    } else {
        LOG::info("UI", "List init success");
    }

    LOG::info("UI", "task init success");
}

void APP::updateChassis(const ChassisUIMsg_s &_msg)
{
    // state
    static CHASSIS::FSMState_e lastState = CHASSIS::FSMState_e::STOP;
    if (lastState != _msg.state) {
        auto name = magic_enum::enum_name(_msg.state);
        memset(dynamicInfo_.chassisState.config.text, 0, sizeof(dynamicInfo_.chassisState.config.text));
        snprintf(dynamicInfo_.chassisState.config.text, sizeof(dynamicInfo_.chassisState.config.text), "%.*s",
                 name.size(), name.data());
        lastState = _msg.state;
        client_.ready(&dynamicInfo_.chassisState);
    }

    // chassisSpdSlider
    constexpr float CHASSIS_SPD_ARC_RANGE_HALF = (CHASSIS_SPD_ARC_END_ANG - CHASSIS_SPD_ARC_START_ANG) / 2.f;
    constexpr float CHASSIS_SPD_ARC_CENTER = CHASSIS_SPD_ARC_START_ANG + CHASSIS_SPD_ARC_RANGE_HALF;
    float chassisVel = std::max(-UI_CHASSIS_SPD_MAX, std::min(_msg.vel, UI_CHASSIS_SPD_MAX));
    float sliderAng = (chassisVel / UI_CHASSIS_SPD_MAX * CHASSIS_SPD_ARC_RANGE_HALF) + CHASSIS_SPD_ARC_CENTER;
    float sliderRad = sliderAng * PI / 180.f;
    dynamicInfo_.chassisSpdSlider.config.startX =
            static_cast<uint32_t>(CHASSIS_SPD_ARC_START_X + (arm_sin_f32(sliderRad) * CHASSIS_SPD_ARC_END_X));
    dynamicInfo_.chassisSpdSlider.config.startY =
            static_cast<uint32_t>(CHASSIS_SPD_ARC_START_Y + (arm_cos_f32(sliderRad) * CHASSIS_SPD_ARC_END_Y));
    client_.ready(&dynamicInfo_.chassisSpdSlider);

    // chassisSpdValue
    dynamicInfo_.chassisSpdValue.config.floatNum = _msg.vel;
    client_.ready(&dynamicInfo_.chassisSpdValue);

    // capEnergyArc
    constexpr float CAP_ENERGY_ARC_RANGE = CAP_ENERGY_ARC_END_ENG - CAP_ENERGY_ARC_START_ENG;
    dynamicInfo_.capEnergyArc.config.startAngle =
            CAP_ENERGY_ARC_END_ENG - static_cast<uint32_t>(_msg.capEnergyRatio * CAP_ENERGY_ARC_RANGE);
    if (_msg.capEnergyRatio < 0.3f)
        dynamicInfo_.capEnergyArc.config.color = Color_e::ORANGE;
    else {
        if (_msg.useCap)
            dynamicInfo_.capEnergyArc.config.color = Color_e::YELLOW;
        else
            dynamicInfo_.capEnergyArc.config.color = Color_e::WHITE;
    }
    client_.ready(&dynamicInfo_.capEnergyArc);

    // chassisEnergyArc
    constexpr float CHASSIS_ENERGY_ARC_RANGE = CHASSIS_ENERGY_ARC_END_ENG - CHASSIS_ENERGY_ARC_START_ENG;
    dynamicInfo_.chassisEnergyArc.config.startAngle =
            CHASSIS_ENERGY_ARC_END_ENG - static_cast<uint32_t>(_msg.chassisEnergyRatio * CHASSIS_ENERGY_ARC_RANGE);
    client_.ready(&dynamicInfo_.chassisEnergyArc);

    // chassisDirection
    dynamicInfo_.chassisDirection.config.startX =
            static_cast<int32_t>(CHASSIS_DIRECTION_RADIUS * arm_sin_f32(_msg.deltaYaw)) + REAL_AIM_X;
    dynamicInfo_.chassisDirection.config.startY =
            static_cast<int32_t>(CHASSIS_DIRECTION_RADIUS * arm_cos_f32(_msg.deltaYaw)) + REAL_AIM_Y;
    client_.ready(&dynamicInfo_.chassisDirection);

    // legLengthValue
    dynamicInfo_.legLengthValue.config.floatNum = (_msg.lengthL + _msg.lengthR) / 2.f;
    client_.ready(&dynamicInfo_.legLengthValue);

    // chassisRollValue
    dynamicInfo_.chassisRollValue.config.floatNum = _msg.roll;
    client_.ready(&dynamicInfo_.chassisRollValue);
}

void APP::updateGimbal(const GimbalUIMsg_s &_msg)
{
    // state
    static GIMBAL::FSMState_e lastState = GIMBAL::FSMState_e::STOP;
    if (lastState != _msg.state) {
        auto name = magic_enum::enum_name(_msg.state);
        memset(dynamicInfo_.gimbalState.config.text, 0, sizeof(dynamicInfo_.gimbalState.config.text));
        snprintf(dynamicInfo_.gimbalState.config.text, sizeof(dynamicInfo_.gimbalState.config.text), "%.*s",
                 name.size(), name.data());
        lastState = _msg.state;
        client_.ready(&dynamicInfo_.gimbalState);
    }

    // vision
    static bool lastAiming = false;
    static bool lastVisionOnline = false;
    if (lastAiming != _msg.aiming) {
        dynamicInfo_.visionState.config.color = _msg.aiming ? Color_e::GREEN : Color_e::WHITE;
        lastAiming = _msg.aiming;
        client_.ready(&dynamicInfo_.visionState);
    }
    if (lastVisionOnline != _msg.visionOnline) {
        if (!_msg.visionOnline)
            dynamicInfo_.visionState.config.color = Color_e::BLACK;
        lastVisionOnline = _msg.visionOnline;
        client_.ready(&dynamicInfo_.visionState);
    }

    // pitchAngSlider
    constexpr float PITCH_ANG_MAX = std::max(std::abs(UI_PITCH_START_ANG), std::abs(UI_PITCH_END_ANG));
    float pitchAng = std::max(-PITCH_ANG_MAX, std::min(_msg.pitchAng, PITCH_ANG_MAX));
    dynamicInfo_.pitchAngSlider.config.startX =
            static_cast<uint32_t>(PITCH_ARC_START_X - (arm_cos_f32(-pitchAng) * PITCH_ARC_END_X));
    dynamicInfo_.pitchAngSlider.config.startY =
            static_cast<uint32_t>(PITCH_ARC_START_Y + (arm_sin_f32(-pitchAng) * PITCH_ARC_END_Y));
    client_.ready(&dynamicInfo_.pitchAngSlider);

    // pitchAngValue
    dynamicInfo_.pitchAngValue.config.floatNum = _msg.pitchAng;
    client_.ready(&dynamicInfo_.pitchAngValue);

    // drivingReferenceLine
    constexpr float PITCH_ANG_RANGE = UI_PITCH_END_ANG - UI_PITCH_START_ANG;
    constexpr float LEFT_X_POS_START_RANGE =
            static_cast<float>(UP_MOVE_LEFT_POS_START_X) - static_cast<float>(DOWN_MOVE_LEFT_POS_START_X);
    constexpr float LEFT_Y_POS_START_RANGE =
            static_cast<float>(UP_MOVE_LEFT_POS_START_Y) - static_cast<float>(DOWN_MOVE_LEFT_POS_START_Y);
    constexpr float LEFT_X_POS_END_RANGE =
            static_cast<float>(UP_MOVE_LEFT_POS_END_X) - static_cast<float>(DOWN_MOVE_LEFT_POS_END_X);
    constexpr float LEFT_Y_POS_END_RANGE =
            static_cast<float>(UP_MOVE_LEFT_POS_END_Y) - static_cast<float>(DOWN_MOVE_LEFT_POS_END_Y);
    float pitchAngAbs = pitchAng - UI_PITCH_START_ANG;
    dynamicInfo_.drivingReferenceLineLeft.config.startX =
            UP_MOVE_LEFT_POS_START_X - static_cast<int32_t>(pitchAngAbs / PITCH_ANG_RANGE * LEFT_X_POS_START_RANGE);
    dynamicInfo_.drivingReferenceLineLeft.config.startY =
            UP_MOVE_LEFT_POS_START_Y - static_cast<int32_t>(pitchAngAbs / PITCH_ANG_RANGE * LEFT_Y_POS_START_RANGE);
    dynamicInfo_.drivingReferenceLineLeft.config.endX =
            UP_MOVE_LEFT_POS_END_X - static_cast<int32_t>(pitchAngAbs / PITCH_ANG_RANGE * LEFT_X_POS_END_RANGE);
    dynamicInfo_.drivingReferenceLineLeft.config.endY =
            UP_MOVE_LEFT_POS_END_Y - static_cast<int32_t>(pitchAngAbs / PITCH_ANG_RANGE * LEFT_Y_POS_END_RANGE);
    client_.ready(&dynamicInfo_.drivingReferenceLineLeft);
    constexpr float RIGHT_X_POS_START_RANGE =
            static_cast<float>(UP_MOVE_RIGHT_POS_START_X) - static_cast<float>(DOWN_MOVE_RIGHT_POS_START_X);
    constexpr float RIGHT_Y_POS_START_RANGE =
            static_cast<float>(UP_MOVE_RIGHT_POS_START_Y) - static_cast<float>(DOWN_MOVE_RIGHT_POS_START_Y);
    constexpr float RIGHT_X_POS_RANGE =
            static_cast<float>(UP_MOVE_RIGHT_POS_END_X) - static_cast<float>(DOWN_MOVE_RIGHT_POS_END_X);
    constexpr float RIGHT_Y_POS_RANGE =
            static_cast<float>(UP_MOVE_RIGHT_POS_END_Y) - static_cast<float>(DOWN_MOVE_RIGHT_POS_END_Y);
    dynamicInfo_.drivingReferenceLineRight.config.startX =
            UP_MOVE_RIGHT_POS_START_X - static_cast<int32_t>(pitchAngAbs / PITCH_ANG_RANGE * RIGHT_X_POS_START_RANGE);
    dynamicInfo_.drivingReferenceLineRight.config.startY =
            UP_MOVE_RIGHT_POS_START_Y - static_cast<int32_t>(pitchAngAbs / PITCH_ANG_RANGE * RIGHT_Y_POS_START_RANGE);
    dynamicInfo_.drivingReferenceLineRight.config.endX =
            UP_MOVE_RIGHT_POS_END_X - static_cast<int32_t>(pitchAngAbs / PITCH_ANG_RANGE * RIGHT_X_POS_RANGE);
    dynamicInfo_.drivingReferenceLineRight.config.endY =
            UP_MOVE_RIGHT_POS_END_Y - static_cast<int32_t>(pitchAngAbs / PITCH_ANG_RANGE * RIGHT_Y_POS_RANGE);
    client_.ready(&dynamicInfo_.drivingReferenceLineRight);
}

void APP::updateArmorLoader(const ArmorLoaderUIMsg_s &_msg)
{
    // state
    static ARMORLOADER::FSMState_e lastState = ARMORLOADER::FSMState_e::STOP;
    if (lastState != _msg.state) {
        auto name = magic_enum::enum_name(_msg.state);
        memset(dynamicInfo_.armorLoaderState.config.text, 0, sizeof(dynamicInfo_.armorLoaderState.config.text));
        snprintf(dynamicInfo_.armorLoaderState.config.text, sizeof(dynamicInfo_.armorLoaderState.config.text), "%.*s",
                 name.size(), name.data());
        lastState = _msg.state;
        client_.ready(&dynamicInfo_.armorLoaderState);
    }
}

void APP::updateArmorBooster(const ArmorBoosterUIMsg_s &_msg)
{
    // state
    static ARMORBOOSTER::FSMState_e lastState = ARMORBOOSTER::FSMState_e::OFF;
    if (lastState != _msg.state) {
        if (_msg.state == ARMORBOOSTER::FSMState_e::OFF)
            dynamicInfo_.fricSwitch.config.operateType = OperateType_e::ADD;
        else
            dynamicInfo_.fricSwitch.config.operateType = OperateType_e::DELETE;
        lastState = _msg.state;
        client_.ready(&dynamicInfo_.fricSwitch);
    }
}

void APP::task()
{
    constexpr uint32_t RESET_DELAY = 2000 / SEND_INTERVAL;
    if (xTaskGetTickCount() - updateCnt >= SEND_INTERVAL) {
        updateCnt = xTaskGetTickCount();
        if (!client_.isCharInit || !client_.isGraphicInit) {
            client_.sendInit();
            resetCnt = 0;
        } else if (resetCnt >= RESET_DELAY)
            client_.send();
        resetCnt++;
    }
}

void APP::uiReInit()
{
    client_.isCharInit = false;
    client_.isGraphicInit = false;
    client_.isIniting = true;
    client_.initTimes = 0;
    client_.nodeReset();
}
