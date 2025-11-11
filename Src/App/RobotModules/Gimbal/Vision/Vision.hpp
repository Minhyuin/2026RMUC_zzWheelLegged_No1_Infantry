#pragma once

#include "TxPacket.hpp"
#include "RxPacket.hpp"
#include "VisionPacketImpl.hpp"

namespace GIMBAL {

class Vision : public COMM::TxPacket<VisionTxPacket>,
               public COMM::RxPacket<VisionRxPacket, 3> {
public:
    Vision();

private:
    void transmit(uint8_t *_buf, uint16_t _len) final;
    void registerCallback() final;
};

} // namespace GIMBAL
