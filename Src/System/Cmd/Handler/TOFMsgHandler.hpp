#pragma once
#include "Bsp.hpp"
#include "Handler.hpp"
#include "TFmini-S.hpp"
#include <bitset>

enum TOFDevType_e : uint8_t {
    TFMINI_S = 0,
};

template <TOFDevType_e Dev> class TOFMsgHandler : public Handler {
public:
    TOFMsgHandler(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event, const QueueHandle_t &_queue) : queue_(_queue)
    {
        switch (Dev) {
        case TFMINI_S:
            tof_ = std::make_unique<TFminiS>(_huart, _event, this->bit_);
        }
    }

    void init(MsgBus_s *_bus, EventGroupHandle_t _event) final
    {
        msgBus_ = _bus;
        this->event = _event;
    }

    void handle() final
    {
        tof_->parse();
        tofMsg_.dist = tof_->getDist();
        notify(&tofMsg_, queue_);
    }

    void notify(Msg *_msg, QueueHandle_t _queue) final { xQueueSend(_queue, _msg, 0); }

private:
    MsgBus_s *msgBus_;

    const QueueHandle_t &queue_;

    TOFMsg_s tofMsg_;

    UART_HandleTypeDef *_uart;

    std::unique_ptr<TOF> tof_ = nullptr;
};