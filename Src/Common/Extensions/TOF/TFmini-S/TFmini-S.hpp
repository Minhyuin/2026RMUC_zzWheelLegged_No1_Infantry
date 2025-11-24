/**
 * @file TFmini-S.hpp
 * @brief 北醒 TFmini-S 协议层驱动，不支持原件的IIC和IO以及字符串格式，默认使用标准数据接收协议
 *        TFmini-S 使用 UART 波特率 115200，8N1，使用 DMA 双缓冲环形接收
 * 
 */

#pragma once

#include <cstdint>
#include "Bsp.hpp"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "TOF.hpp"

#define TOF_LEFT_READY_EVENT  (1 << 4)
#define TOF_RIGHT_READY_EVENT (1 << 5)

class TFminiS : public TOF {
    static constexpr uint8_t SOF = 0x59;
    static constexpr uint8_t FRAME_LENGTH = 9;
    static constexpr uint8_t RX_LOST_MAX = 10;

public:
    TFminiS(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event);
    ~TFminiS() override;
    bool isOnline() final;
    float getDist() final;

#pragma pack(push, 1)
    struct TFminiSData_s {
        uint16_t distance; // cm [0,1200]
        uint16_t strength;
        uint16_t temp; // ℃
    };
#pragma pack(pop)

private:
    void parse() final;
    void callBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos) final;

    bool checksum();

    TFminiSData_s data_{};
};