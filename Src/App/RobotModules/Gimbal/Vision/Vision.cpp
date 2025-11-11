#include "Vision.hpp"
#include "Bsp_usb.hpp"

using namespace GIMBAL;

Vision::Vision() : TxPacket<VisionTxPacket>(500.f) { registerCallback(); }

void Vision::transmit(uint8_t *_buf, uint16_t _len)
{
    Usb::instance().send(_buf, _len);
}

void Vision::registerCallback()
{
    Usb::instance().registerCallback([this](uint8_t *_buf, uint16_t _len) {
        if (_len == sizeof(RxPacket::Data)) {
            xQueueSendFromISR(this->queue_, _buf, nullptr);
        }
    });
}
