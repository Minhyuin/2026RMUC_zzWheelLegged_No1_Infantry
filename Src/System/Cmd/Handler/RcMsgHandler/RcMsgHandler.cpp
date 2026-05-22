#include "RcMsgHandler.hpp"
#include "sdkconfig.h"

#include "RemoteControl.hpp"

#ifdef CHASSIS_TYPE
#include CHASSIS_FILE
#endif
#ifdef GIMBAL_TYPE
#include GIMBAL_FILE
#endif

#include "DT7.hpp"
#include "ET08A.hpp"
#include "VT13.hpp"

// Explicit template instantiation
#if EXTENSION_DT7
template class RcMsgHandler<RCDevType_e::DT7>;
#endif
#if EXTENSION_ET08A
template class RcMsgHandler<RCDevType_e::ET08A>;
#endif
#if EXTENSION_VT13
template class RcMsgHandler<RCDevType_e::VT13>;
#endif

#if APP_USE_DAEMONS
#include "Daemons.hpp"
#endif

#include "StmLog.hpp"

#include <cmath>
#include <cstring>

#include "magic_enum/magic_enum.hpp"

#include "Smooth.hpp"

template <RCDevType_e DEV_TYPE>
RcMsgHandler<DEV_TYPE>::RcMsgHandler(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event)
{
    createRcDev(_huart, _event);
};

template <RCDevType_e DEV_TYPE>
void RcMsgHandler<DEV_TYPE>::createRcDev(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event)
{
    if constexpr (DEV_TYPE == RCDevType_e::DT7) {
        rc_ = std::make_unique<DT7>(_huart, _event, this->bit_);
    } else if constexpr (DEV_TYPE == RCDevType_e::ET08A) {
        rc_ = std::make_unique<ET08A>(_huart, _event, this->bit_);
    } else if constexpr (DEV_TYPE == RCDevType_e::VT13) {
        rc_ = std::make_unique<VT13>(_huart, _event, this->bit_);
    }
    scheduleDaemon();
}

template <RCDevType_e DEV_TYPE> void RcMsgHandler<DEV_TYPE>::scheduleDaemon()
{
#if APP_USE_DAEMONS
    Daemons::instance().schedule([this]() {
        static uint32_t updateCnt = 0;
        if (!this->rc_->isOnline()) {
            if (xTaskGetTickCount() - updateCnt >= 2000) {
                updateCnt = xTaskGetTickCount();
                LOG::warn("RcMsgHandler", "%s offline", magic_enum::enum_name(DEV_TYPE).data());
            }
        }
    });
#endif
}

template <RCDevType_e DEV_TYPE> void RcMsgHandler<DEV_TYPE>::init(MsgBus_s *_bus, EventGroupHandle_t _event)
{
    msgBus_ = _bus;
    this->event = _event;
}

template <RCDevType_e DEV_TYPE> void RcMsgHandler<DEV_TYPE>::handle()
{
    rc_->parse();

    masterHandle();
}

/**
 * @brief Master handler for processing remote controller messages
 * 
 * @details Since remote controllers vary widely in channel configurations and button mappings
 *          a universal processing method cannot be implemented. Users should acquire data from 
 *          their specific remote controller, normalize it as needed, and implement custom
 *          interaction logic based on the application requirements.
 */
template <RCDevType_e DEV_TYPE> void RcMsgHandler<DEV_TYPE>::masterHandle()
{
    // User-defined processing logic goes here ↓
    if constexpr (DEV_TYPE == RCDevType_e::DT7) {
        DT7::RcData_s *rcData = static_cast<DT7::RcData_s *>(rc_->getData());
        static DT7::RcData_s rcDataPrev = {};
        static constexpr float CH_RANGE = DT7::CH_VALUE_MAX - DT7::CH_VALUE_OFFSET;
        static constexpr float WHEEL_THESHORLD = 0.2f;
        using SW = DT7::Sw_e;
        SW &sr = rcData->rc.switchRight;
        SW &sl = rcData->rc.switchLeft;
        if (sr == SW::DOWN) {
            gMsg_.state = GIMBAL::FSMState_e::STOP;
            gMsg_.wPitch = gMsg_.wYaw = 0.f;
            cMsg_.state = CHASSIS::FSMState_e::STOP;
            cMsg_.vx = cMsg_.vy = cMsg_.droll = cMsg_.wz = cMsg_.dphi = cMsg_.dl = 0.f;
            alMsg_.state = ARMORLOADER::FSMState_e::STOP;
            alMsg_.fire = false;
            abMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
        } else if (sr == SW::MID) {
            cMsg_.vx = static_cast<float>(rcData->rc.ch1) / CH_RANGE * INPUT_VX_MAX;
            cMsg_.vy = -static_cast<float>(rcData->rc.ch0) / CH_RANGE * INPUT_VX_MAX;
            cMsg_.droll = static_cast<float>(rcData->rc.ch0) / CH_RANGE * INPUT_DROLL_MAX;
            gMsg_.wPitch = -static_cast<float>(rcData->rc.ch3) / CH_RANGE * INPUT_WPITCH_MAX;
            gMsg_.wYaw = -static_cast<float>(rcData->rc.ch2) / CH_RANGE * INPUT_WYAW_MAX;
            cMsg_.dphi = -static_cast<float>(rcData->rc.ch0) / CH_RANGE * INPUT_DPHI_MAX;
            cMsg_.dl = static_cast<float>(rcData->rc.ch3) / CH_RANGE * INPUT_DL_MAX;
            cMsg_.wz = -static_cast<float>(rcData->rc.ch2) / CH_RANGE * INPUT_WZ_MAX;
            alMsg_.fire = std::fabs(static_cast<float>(rcData->wheel - DT7::CH_VALUE_OFFSET) / CH_RANGE) >
                          WHEEL_THESHORLD;
            if (sl == SW::DOWN) {
                gMsg_.state = GIMBAL::FSMState_e::RUN;
                cMsg_.state = CHASSIS::FSMState_e::IDLE;
                alMsg_.state = ARMORLOADER::FSMState_e::STOP;
                abMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
            } else if (sl == SW::MID) {
                gMsg_.state = GIMBAL::FSMState_e::AUTO;
                // cMsg_.state = CHASSIS::FSMState_e::TANK;
                alMsg_.state = ARMORLOADER::FSMState_e::AUTO;
                abMsg_.state = ARMORBOOSTER::FSMState_e::ON;
            } else if (sl == SW::UP) {
                alMsg_.state = ARMORLOADER::FSMState_e::SINGLE;
            }
        } else if (sr == SW::UP) {
            cMsg_.vx = static_cast<float>(rcData->rc.ch1) / CH_RANGE * INPUT_VX_MAX;
            cMsg_.vy = -static_cast<float>(rcData->rc.ch0) / CH_RANGE * INPUT_VX_MAX;
            cMsg_.droll = static_cast<float>(rcData->rc.ch0) / CH_RANGE * INPUT_DROLL_MAX;
            gMsg_.wPitch = -static_cast<float>(rcData->rc.ch3) / CH_RANGE * INPUT_WPITCH_MAX;
            gMsg_.wYaw = -static_cast<float>(rcData->rc.ch2) / CH_RANGE * INPUT_WYAW_MAX;
            cMsg_.dphi = -static_cast<float>(rcData->rc.ch0) / CH_RANGE * INPUT_DPHI_MAX;
            cMsg_.dl = static_cast<float>(rcData->rc.ch3) / CH_RANGE * INPUT_DL_MAX;
            cMsg_.wz = -static_cast<float>(rcData->rc.ch2) / CH_RANGE * INPUT_WZ_MAX;
            alMsg_.fire = false;
            if (sl == SW::DOWN) {
                cMsg_.needClimb = true;
            } else {
                cMsg_.needClimb = false;
                cMsg_.state = CHASSIS::FSMState_e::SPIN;
            }
            gMsg_.state = GIMBAL::FSMState_e::RUN;
            alMsg_.state = ARMORLOADER::FSMState_e::STOP;
            abMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
        } else {
            // TODO: Use keyboard
            return;
        }
        memcpy(&rcDataPrev, rcData, sizeof(DT7::RcData_s));
    } else if constexpr (DEV_TYPE == RCDevType_e::ET08A) {
        ET08A::RcData_s *rcData = static_cast<ET08A::RcData_s *>(rc_->getData());
        static ET08A::RcData_s rcDataPrev = {};
        static constexpr float CH_RANGE = ET08A::CH_VALUE_MAX - ET08A::CH_VALUE_OFFSET;
        static constexpr float WHEEL_THESHORLD = 0.2f;
        using SW = ET08A::Sw_e;
        SW &sr = rcData->switchRight;
        SW &sl = rcData->switchLeft;
        if (sr == SW::DOWN) {
            gMsg_.state = GIMBAL::FSMState_e::STOP;
            gMsg_.wPitch = gMsg_.wYaw = 0.f;
            cMsg_.state = CHASSIS::FSMState_e::STOP;
            cMsg_.vx = cMsg_.vy = cMsg_.droll = cMsg_.wz = cMsg_.dphi = cMsg_.dl = 0.f;
            alMsg_.state = ARMORLOADER::FSMState_e::STOP;
            alMsg_.fire = false;
            abMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
        } else if (sr == SW::MID) {
            cMsg_.vx = static_cast<float>(rcData->ch1) / CH_RANGE * INPUT_VX_MAX;
            cMsg_.vy = -static_cast<float>(rcData->ch0) / CH_RANGE * INPUT_VX_MAX;
            cMsg_.droll = static_cast<float>(rcData->ch0) / CH_RANGE * INPUT_DROLL_MAX;
            gMsg_.wPitch = -static_cast<float>(rcData->ch3) / CH_RANGE * INPUT_WPITCH_MAX;
            gMsg_.wYaw = -static_cast<float>(rcData->ch2) / CH_RANGE * INPUT_WYAW_MAX;
            cMsg_.dphi = -static_cast<float>(rcData->ch0) / CH_RANGE * INPUT_DPHI_MAX;
            cMsg_.dl = static_cast<float>(rcData->ch3) / CH_RANGE * INPUT_DL_MAX;
            cMsg_.wz = -static_cast<float>(rcData->ch2) / CH_RANGE * INPUT_WZ_MAX;
            alMsg_.fire = std::fabs(static_cast<float>(rcData->wheel - ET08A::CH_VALUE_OFFSET) / CH_RANGE) >
                          WHEEL_THESHORLD;
            if (sl == SW::DOWN) {
                gMsg_.state = GIMBAL::FSMState_e::RUN;
                cMsg_.state = CHASSIS::FSMState_e::IDLE;
                alMsg_.state = ARMORLOADER::FSMState_e::STOP;
                abMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
            } else if (sl == SW::MID) {
                gMsg_.state = GIMBAL::FSMState_e::AUTO;
                // cMsg_.state = CHASSIS::FSMState_e::TANK;
                alMsg_.state = ARMORLOADER::FSMState_e::AUTO;
                abMsg_.state = ARMORBOOSTER::FSMState_e::ON;
            } else if (sl == SW::UP) {
                alMsg_.state = ARMORLOADER::FSMState_e::SINGLE;
            }
        } else if (sr == SW::UP) {
            cMsg_.vx = static_cast<float>(rcData->ch1) / CH_RANGE * INPUT_VX_MAX;
            cMsg_.vy = -static_cast<float>(rcData->ch0) / CH_RANGE * INPUT_VX_MAX;
            cMsg_.droll = static_cast<float>(rcData->ch0) / CH_RANGE * INPUT_DROLL_MAX;
            gMsg_.wPitch = -static_cast<float>(rcData->ch3) / CH_RANGE * INPUT_WPITCH_MAX;
            gMsg_.wYaw = -static_cast<float>(rcData->ch2) / CH_RANGE * INPUT_WYAW_MAX;
            cMsg_.dphi = -static_cast<float>(rcData->ch0) / CH_RANGE * INPUT_DPHI_MAX;
            cMsg_.dl = static_cast<float>(rcData->ch3) / CH_RANGE * INPUT_DL_MAX;
            cMsg_.wz = -static_cast<float>(rcData->ch2) / CH_RANGE * INPUT_WZ_MAX;
            alMsg_.fire = false;
            if (sl == SW::DOWN) {
                cMsg_.needClimb = true;
            } else {
                cMsg_.needClimb = false;
                cMsg_.state = CHASSIS::FSMState_e::SPIN;
            }
            gMsg_.state = GIMBAL::FSMState_e::RUN;
            alMsg_.state = ARMORLOADER::FSMState_e::STOP;
            abMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
        } else {
            // TODO: Use keyboard
            return;
        }
        memcpy(&rcDataPrev, rcData, sizeof(ET08A::RcData_s));
    } else if constexpr (DEV_TYPE == RCDevType_e::VT13) {
        static constexpr float CH_RANGE = VT13::CH_VALUE_MAX - VT13::CH_VALUE_OFFSET;
        VT13::RcData_s *rcData = static_cast<VT13::RcData_s *>(rc_->getData());
        static VT13::RcData_s rcDataPrev = {};
        keyBoardCtrlFlag_ = (rcData->gamepad.sw == VT13::Sw_e::C);
        ctrlFlag_ = (keyBoardCtrlFlag_ ?
                             ((IS_KEY_PRESS(rcData->KBM.key, VT13::A) && IS_KEY_PRESS(rcData->KBM.key, VT13::S) &&
                               IS_KEY_PRESS(rcData->KBM.key, VT13::Z)) ?
                                      ([&]() {
                                          cMsg_.state = CHASSIS::FSMState_e::IDLE;
                                          gMsg_.state = GIMBAL::FSMState_e::RUN;
                                          return true;
                                      }()) :
                              (IS_KEY_PRESS(rcData->KBM.key, VT13::E) && IS_KEY_PRESS(rcData->KBM.key, VT13::C)/*&&
                               IS_KEY_PRESS(rcData->KBM.key, VT13::B)*/) ?
                                      false :
                              (lastSwitch_ != static_cast<uint8_t>(VT13::Sw_e::C)) ? false :
                                                                                     ctrlFlag_) :
                             true);
        lastSwitch_ = static_cast<uint8_t>(rcData->gamepad.sw);
        if (IS_KEY_PRESS(rcData->KBM.key, VT13::G) && !IS_KEY_PRESS(rcDataPrev.KBM.key, VT13::G))
            cMsg_.noChassis = !cMsg_.noChassis;
        if (!ctrlFlag_) {
            gMsg_.state = GIMBAL::FSMState_e::STOP;
            gMsg_.wPitch = gMsg_.wYaw = 0.f;
            if (rcData->gamepad.ch[1] < -600 && rcData->gamepad.ch[0] > 600 && rcData->gamepad.ch[2] < -600 &&
                rcData->gamepad.ch[3] < -600)
                keepTime_++;
            else
                keepTime_ = 0;
            if (keepTime_ > 100) {
                keepTime_ = 0;
                cMsg_.noChassis = !cMsg_.noChassis;
            }
            cMsg_.state = CHASSIS::FSMState_e::STOP;
            cMsg_.vx = cMsg_.vy = cMsg_.droll = cMsg_.wz = cMsg_.dphi = cMsg_.dl = 0.f;
            alMsg_.state = ARMORLOADER::FSMState_e::STOP;
            alMsg_.fire = false;
            abMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
#ifdef APP_USE_UI
            cMsg_.uiReset = rcData->gamepad.pause && !rcDataPrev.gamepad.pause;
#endif
        } else {
            if (!keyBoardCtrlFlag_) {
                if (rcData->gamepad.pause && !rcDataPrev.gamepad.pause) {
                    cMsg_.needRush = !cMsg_.needRush;
                }
                if (rcData->gamepad.sw == VT13::Sw_e::N) {
                    gMsg_.state = GIMBAL::FSMState_e::RUN;
                    cMsg_.state = CHASSIS::FSMState_e::IDLE;
                    alMsg_.state = ARMORLOADER::FSMState_e::STOP;
                    abMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
                } else if (rcData->gamepad.sw == VT13::Sw_e::S) {
                    gMsg_.state = GIMBAL::FSMState_e::AUTO;
                    cMsg_.state = CHASSIS::FSMState_e::IDLE;
                    if (rcData->gamepad.fn2 && !rcDataPrev.gamepad.fn2) {
                        if (alMsg_.state == ARMORLOADER::FSMState_e::AUTO)
                            alMsg_.state = ARMORLOADER::FSMState_e::SINGLE;
                        else
                            alMsg_.state = ARMORLOADER::FSMState_e::AUTO;
                    }
                    if (alMsg_.state == ARMORLOADER::FSMState_e::STOP)
                        alMsg_.state = ARMORLOADER::FSMState_e::AUTO;
                    abMsg_.state = ARMORBOOSTER::FSMState_e::ON;
                }
                if (rcData->gamepad.trigger)
                    cMsg_.state = CHASSIS::FSMState_e::SPIN;

                cMsg_.needStretch = rcData->gamepad.pause;
                cMsg_.needClimb = rcData->gamepad.pause && rcData->gamepad.fn2;

                cMsg_.vx = static_cast<float>(rcData->gamepad.ch[1]) / CH_RANGE * INPUT_VX_MAX;
                cMsg_.vy = -static_cast<float>(rcData->gamepad.ch[0]) / CH_RANGE * INPUT_VX_MAX;
                gMsg_.wPitch = -static_cast<float>(rcData->gamepad.ch[2]) / CH_RANGE * INPUT_WPITCH_MAX;
                gMsg_.wYaw = -static_cast<float>(rcData->gamepad.ch[3]) / CH_RANGE * INPUT_WYAW_MAX;
                if (rcData->gamepad.fn2) {
                    cMsg_.dl = static_cast<float>(rcData->gamepad.ch[2]) / CH_RANGE * INPUT_DL_MAX;
                    cMsg_.droll = static_cast<float>(rcData->gamepad.ch[3]) / CH_RANGE * INPUT_DROLL_MAX;
                    gMsg_.wPitch = gMsg_.wYaw = 0;
                } else {
                    cMsg_.dl = cMsg_.droll = 0;
                }
                cMsg_.needJump = rcData->gamepad.fn1;
                alMsg_.fire = rcData->gamepad.wheel != 0;
                alMsg_.shootFreqRatio = std::fabs(static_cast<float>(rcData->gamepad.wheel) / CH_RANGE);
            } else {
                static constexpr float KEYBOARD_ENABLE_ACC = 0.1f;
                static constexpr float KEYBOARD_RESTORE_ACC = 0.2f;
                // WASD
                if (IS_KEY_PRESS(rcData->KBM.key, VT13::W)) {
                    cMsg_.vx = SMOOTH::slopeFollowing(INPUT_VX_MAX, cMsg_.vx, KEYBOARD_ENABLE_ACC);
                }
                if (IS_KEY_PRESS(rcData->KBM.key, VT13::S)) {
                    cMsg_.vx = SMOOTH::slopeFollowing(-INPUT_VX_MAX, cMsg_.vx, KEYBOARD_ENABLE_ACC);
                }
                if (!IS_KEY_PRESS(rcData->KBM.key, VT13::S) && (!IS_KEY_PRESS(rcData->KBM.key, VT13::W))) {
                    cMsg_.vx = SMOOTH::slopeFollowing(0, cMsg_.vx, KEYBOARD_RESTORE_ACC);
                    cMsg_.dl = SMOOTH::slopeFollowing(0, cMsg_.dl, KEYBOARD_RESTORE_ACC);
                }
                if (IS_KEY_PRESS(rcData->KBM.key, VT13::A)) {
                    cMsg_.vy = SMOOTH::slopeFollowing(INPUT_VX_MAX, cMsg_.vy, KEYBOARD_ENABLE_ACC);
                }
                if (IS_KEY_PRESS(rcData->KBM.key, VT13::D)) {
                    cMsg_.vy = SMOOTH::slopeFollowing(-INPUT_VX_MAX, cMsg_.vy, KEYBOARD_ENABLE_ACC);
                }
                if (!IS_KEY_PRESS(rcData->KBM.key, VT13::A) && (!IS_KEY_PRESS(rcData->KBM.key, VT13::D))) {
                    cMsg_.vy = SMOOTH::slopeFollowing(0, cMsg_.vy, KEYBOARD_RESTORE_ACC);
                    cMsg_.droll = SMOOTH::slopeFollowing(0, cMsg_.droll, KEYBOARD_RESTORE_ACC);
                }

                // length and roll
                if (IS_KEY_PRESS(rcData->KBM.key, VT13::F)) {
                    cMsg_.vy = 0;
                    cMsg_.needStretch = true;
                    // if (IS_KEY_PRESS(rcData->KBM.key, VT13::A)) {
                    //     cMsg_.droll = SMOOTH::slopeFollowing(-INPUT_DROLL_MAX, cMsg_.droll, KEYBOARD_ENABLE_ACC);
                    // }
                    // if (IS_KEY_PRESS(rcData->KBM.key, VT13::D)) {
                    //     cMsg_.droll = SMOOTH::slopeFollowing(INPUT_DROLL_MAX, cMsg_.droll, KEYBOARD_ENABLE_ACC);
                    // }
                } else {
                    cMsg_.needStretch = false;
                }

                // gimbal
                gMsg_.wPitch = -static_cast<float>(rcData->KBM.mouseY) * GAIN_MOUSE_Y;
                gMsg_.wYaw = -static_cast<float>(rcData->KBM.mouseX) * GAIN_MOUSE_X;
                gMsg_.forceFire = IS_KEY_PRESS(rcData->KBM.key, VT13::C);
                if (IS_KEY_PRESS(rcData->KBM.key, VT13::V) && IS_KEY_PRESS(rcData->KBM.key, VT13::Z)) {
                    gMsg_.resetTracker = true;
                    gMsg_.wPitchOffset = -static_cast<float>(rcData->KBM.mouseZ) * GAIN_MOUSE_Z;
                    gMsg_.needClearPitchOffset = rcData->KBM.mouseMiddle;
                } else {
                    gMsg_.resetTracker = false;
                    gMsg_.wPitchOffset = 0;
                }
                gMsg_.changeTarget = IS_KEY_PRESS(rcData->KBM.key, VT13::R);
                if (rcData->KBM.mouseRight) {
                    gMsg_.state = GIMBAL::FSMState_e::AUTO;
                } else {
                    gMsg_.state = GIMBAL::FSMState_e::RUN;
                }

                // fire
                alMsg_.fire = rcData->KBM.mouseLeft;
                alMsg_.shootFreqRatio = 1.f;

                // other buttons
                if (IS_KEY_PRESS(rcData->KBM.key, VT13::Z) && !IS_KEY_PRESS(rcDataPrev.KBM.key, VT13::Z))
                    cMsg_.state = CHASSIS::FSMState_e::IDLE;

                if (IS_KEY_PRESS(rcData->KBM.key, VT13::X) && !IS_KEY_PRESS(rcDataPrev.KBM.key, VT13::X))
                    cMsg_.state = CHASSIS::FSMState_e::SPIN;
                if (IS_KEY_PRESS(rcData->KBM.key, VT13::Q) && !IS_KEY_PRESS(rcDataPrev.KBM.key, VT13::Q))
                    gMsg_.state = GIMBAL::FSMState_e::U_TURN;
                if (IS_KEY_PRESS(rcData->KBM.key, VT13::V) && !IS_KEY_PRESS(rcDataPrev.KBM.key, VT13::V)) {
                    alMsg_.state = ARMORLOADER::FSMState_e::STOP;
                    abMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
                }
                if (IS_KEY_PRESS(rcData->KBM.key, VT13::C) && !IS_KEY_PRESS(rcDataPrev.KBM.key, VT13::C)) {
                    alMsg_.state = ARMORLOADER::FSMState_e::AUTO;
                    abMsg_.state = ARMORBOOSTER::FSMState_e::ON;
                }
                if (IS_KEY_PRESS(rcData->KBM.key, VT13::B) && !IS_KEY_PRESS(rcDataPrev.KBM.key, VT13::B)) {
                    gMsg_.buffFlagOffSite = !gMsg_.buffFlagOffSite;
                    alMsg_.state = ARMORLOADER::FSMState_e::SINGLE;
                    abMsg_.state = ARMORBOOSTER::FSMState_e::ON;
                }
                cMsg_.needJump = IS_KEY_PRESS(rcData->KBM.key, VT13::E) && IS_KEY_PRESS(rcData->KBM.key, VT13::SHIFT);
                cMsg_.needClimb = IS_KEY_PRESS(rcData->KBM.key, VT13::SHIFT) && IS_KEY_PRESS(rcData->KBM.key, VT13::F);
                cMsg_.needRush = IS_KEY_PRESS(rcData->KBM.key, VT13::SHIFT);
            }
        }
        memcpy(&rcDataPrev, rcData, sizeof(VT13::RcData_s));
    }

    notify(&gMsg_, msgBus_->gimbalQueue);
    notify(&cMsg_, msgBus_->chassisQueue);
    notify(&alMsg_, msgBus_->armorLoaderQueue);
    notify(&abMsg_, msgBus_->armorBoosterQueue);
    // User-defined processing logic goes here ↑
}

template <RCDevType_e DEV_TYPE> void RcMsgHandler<DEV_TYPE>::notify(Msg *_msg, QueueHandle_t _queue)
{
    xQueueSend(_queue, _msg, 0);
}
