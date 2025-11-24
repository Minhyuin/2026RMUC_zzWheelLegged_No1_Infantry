#pragma once

#include "MsgImpl.hpp"
#include "Task.hpp"
#include "queue.h"
#include "event_groups.h"

#include "RttMsgHandler.hpp"
#include "RcMsgHandler.hpp"
#include "RefereeHandler.hpp"
#include "TofMsgHandler.hpp"
#include <bitset>

class Cmd : public Task<Cmd, 256> {
public:
    Cmd();

    void task();
    MsgBus_s *getMsgBus() { return &msgBus_; }

protected:
    void parseMsg();

private:
    MsgBus_s msgBus_;
    EventGroupHandle_t eventGroup_;
    std::bitset<32> masks_;

    RcMsgHandler rcHandler_;
    RTTMsgHandler rttHandler_;
#if defined APP_USE_REFEREE
    RefereeHandler refereeHandler_;
#endif
#if EXTENSION_TOF
    std::unique_ptr<TOFMsgHandler<TFMINI_S> > tofLeftHandler_;
    std::unique_ptr<TOFMsgHandler<TFMINI_S> > tofRightHandler_;
#endif
};
