#include "TFmini-S.hpp"
#include <cstring>

TFminiS::TFminiS(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event) : TOF(_huart, FRAME_LENGTH * 2, _event)
{
    uart_.recvDmaMultiBufInit((uint32_t *)buf_, FRAME_LENGTH * 2);
    uart_.registerCallback(
            [this](UART_HandleTypeDef *_huart, uint16_t _dataLength) { callBackFromISR(_huart, _dataLength); });

    rxLostCnt_ = RX_LOST_MAX;
}

TFminiS::~TFminiS()
{
    Dma::instance().ram_free(buf_);
    uart_.unregisterCallback();
}

bool TFminiS::checksum()
{
    int sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += buf_[i];
    }
    return (sum & 0xFF) == buf_[8];
}

void TFminiS::callBackFromISR(UART_HandleTypeDef *_huart, uint16_t _dataLength)
{
    uint16_t size = _huart->RxXferCount;
    // NOLINTBEGIN(readability-redundant-casting)
    if (((((DMA_Stream_TypeDef *)_huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET) {
        __HAL_DMA_DISABLE(_huart->hdmarx);
        ((DMA_Stream_TypeDef *)_huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
        __HAL_DMA_SET_COUNTER(_huart->hdmarx, 2 * FRAME_LENGTH);
        if (size == FRAME_LENGTH) {
            if (buf_[0] == SOF && buf_[1] == SOF && checksum()) {
                xEventGroupSetBitsFromISR(event_, TOF_READY_EVENT1 | TOF_READY_EVENT2, nullptr);
            }
        }
    } else {
        __HAL_DMA_DISABLE(_huart->hdmarx);
        ((DMA_Stream_TypeDef *)_huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
        __HAL_DMA_SET_COUNTER(_huart->hdmarx, 2 * FRAME_LENGTH);
        if (size == FRAME_LENGTH) {
            if (buf_[0] == SOF && buf_[1] == SOF && checksum()) {
                xEventGroupSetBitsFromISR(event_, TOF_READY_EVENT1 | TOF_READY_EVENT2, nullptr);
            }
        }
    }
    __HAL_DMA_ENABLE(_huart->hdmarx);
    // NOLINTEND(readability-redundant-casting)
}

void TFminiS::parse()
{
    data_.distance = buf_[3] << 8 | buf_[2];
    data_.strength = buf_[5] << 8 | buf_[4];
    data_.temp = (buf_[7] << 8 | buf_[6]) / 8 - 256;

    rxLostCnt_ = 0;
}

bool TFminiS::isOnline()
{
    if (rxLostCnt_ < RX_LOST_MAX) {
        rxLostCnt_ = rxLostCnt_ + 1;
        return true;
    } else
        return false;
}

void *TFminiS::getData() { return &data_; }
