#pragma once
#include "Bsp.hpp"
#include "Handler.hpp"
#include <memory>
#include "RemoteControl.hpp"

#define IS_KEY_PRESS(CODE, KEY) (((CODE) & (KEY)) == (KEY))

enum class RCDevType_e : uint8_t {
    DT7 = 0,
    ET08A = 1,
    VT13 = 2,
};

template <RCDevType_e DEV_TYPE> class RcMsgHandler final : public Handler {
    static constexpr float INPUT_VX_MAX = 2.5f;
    static constexpr float INPUT_VY_MAX = INPUT_VX_MAX;
    static constexpr float INPUT_WZ_MAX = 3.f;

    static constexpr float INPUT_DL_MAX = 0.15f;
    static constexpr float INPUT_DROLL_MAX = 0.7f;
    static constexpr float INPUT_DPHI_MAX = 1.f;

    static constexpr float INPUT_WPITCH_MAX = 4.f;
    static constexpr float INPUT_WYAW_MAX = 4.5f;

    static constexpr float GAIN_MOUSE_X = 0.1f;
    static constexpr float GAIN_MOUSE_Y = 0.1f;

public:
    RcMsgHandler(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event);
    void init(MsgBus_s *_bus, EventGroupHandle_t _event) final;
    void handle() final;
    void notify(Msg *_msg, QueueHandle_t _queue) final;

private:
    MsgBus_s *msgBus_;

    GimbalMsg_s gMsg_ = {};
    ChassisMsg_s cMsg_ = {};
    ArmorLoaderMsg_s alMsg_ = {};
    ArmorBoosterMsg_s abMsg_ = {};
    uint16_t keepTime_ = 0;

    bool keyBoardCtrlFlag_ = false;
    bool ctrlFlag_ = false;
    uint8_t lastSwitch_ = 0;

    std::unique_ptr<RemoteControl> rc_ = nullptr;

    void createRcDev(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event);

    void scheduleDaemon();

    void masterHandle();
};
