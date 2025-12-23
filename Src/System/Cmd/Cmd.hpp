#pragma once

#include "MsgImpl.hpp"
#include "Task.hpp"
#include "queue.h"
#include "event_groups.h"

#include "RttMsgHandler.hpp"
#include "RcMsgHandler.hpp"
#include "RefereeHandler.hpp"
#include "TofMsgHandler.hpp"

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

    RcMsgHandler rcHandler_;

    RTTMsgHandler rttHandler_;

#if APP_USE_REFEREE
    RefereeHandler refereeHandler_;
#endif

#if EXTENSION_TOF
    TOFMsgHandler<TFMINI_S> tofLeftHandler_;
    TOFMsgHandler<TFMINI_S> tofRightHandler_;
#endif
};
