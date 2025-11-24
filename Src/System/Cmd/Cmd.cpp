#include "Cmd.hpp"
#include "MsgImpl.hpp"
#include "StmLog.hpp"

Cmd::Cmd()
        : Task<Cmd, 256>("CmdTask", TaskPriority_e::LOW2)
        , eventGroup_(xEventGroupCreate())
        , rcHandler_(&RC_UART, eventGroup_)
{
    msgBus_.chassisQueue = xQueueCreate(10, sizeof(ChassisMsg_s));
    msgBus_.gimbalQueue = xQueueCreate(10, sizeof(GimbalMsg_s));
    msgBus_.armorBoosterQueue = xQueueCreate(10, sizeof(ArmorBoosterMsg_s));
    msgBus_.armorLoaderQueue = xQueueCreate(10, sizeof(ArmorLoaderMsg_s));
#if defined APP_USE_REFEREE
    msgBus_.refereeQueue = xQueueCreate(10, sizeof(RefereeMsg_s));
#endif
#if EXTENSION_TOF
    msgBus_.tofLefDistQueue = xQueueCreate(10, sizeof(TOFMsg_s));
    msgBus_.tofRigDistQueue = xQueueCreate(10, sizeof(TOFMsg_s));
    tofLeftHandler_ =
            std::make_unique<TOFMsgHandler<TFMINI_S> >(&TFMINIS_LEFT_UART, eventGroup_, msgBus_.tofLefDistQueue);
    tofRightHandler_ =
            std::make_unique<TOFMsgHandler<TFMINI_S> >(&TFMINIS_RIGHT_UART, eventGroup_, msgBus_.tofRigDistQueue);
#endif

    masks_.reset();

    rttHandler_.init(&msgBus_, eventGroup_);
    masks_ = masks_ | std::bitset<32>(RTT_READY_EVENT);

    rcHandler_.init(&msgBus_, eventGroup_);
    masks_ = masks_ | std::bitset<32>(RC_READY_EVENT);
    masks_ = masks_ | std::bitset<32>(ET08A_READY_EVENT);

#if defined APP_USE_REFEREE
    refereeHandler_.init(&msgBus_, eventGroup_);
    masks_ = masks_ | std::bitset<32>(REFEREE_READY_EVENT);
#endif

#if EXTENSION_TOF
    tofLeftHandler_->init(&msgBus_, eventGroup_);
    masks_ = masks_ | std::bitset<32>(TOF_LEFT_READY_EVENT);
    tofRightHandler_->init(&msgBus_, eventGroup_);
    masks_ = masks_ | std::bitset<32>(TOF_RIGHT_READY_EVENT);
#endif

    LOG::info("cmd", "init success");
}


void Cmd::parseMsg()
{
    EventBits_t xBits = xEventGroupWaitBits(eventGroup_, masks_.to_ulong(), pdTRUE, pdFALSE, portMAX_DELAY);
    if (xBits & (RC_READY_EVENT | ET08A_READY_EVENT)) {
        rcHandler_.handle();
    }
    if (xBits & RTT_READY_EVENT) {
        rttHandler_.handle();
    }
#if defined APP_USE_REFEREE
    if (xBits & REFEREE_READY_EVENT) {
        refereeHandler_.handle();
    }
#endif

#if EXTENSION_TOF
    if (xBits & TOF_LEFT_READY_EVENT) {
        tofLeftHandler_->handle();
    }
    if (xBits & TOF_RIGHT_READY_EVENT) {
        tofRightHandler_->handle();
    }
#endif
}

void Cmd::task()
{
    for (;;) {
        parseMsg();
    }
}
