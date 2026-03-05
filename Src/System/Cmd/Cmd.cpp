#include "Cmd.hpp"
#include "MsgImpl.hpp"
#include "StmLog.hpp"

Cmd::Cmd()
        : Task<Cmd, 256>("CmdTask", TaskPriority_e::LOW2)
#if EXTENSION_TOF
        , tofLeftHandler_(&TFMINIS_LEFT_UART, eventGroup_, msgBus_.tofLefDistQueue)
        , tofRightHandler_(&TFMINIS_RIGHT_UART, eventGroup_, msgBus_.tofRigDistQueue)
#endif

{
    msgBus_.chassisQueue = xQueueCreate(10, sizeof(ChassisMsg_s));
    msgBus_.gimbalQueue = xQueueCreate(10, sizeof(GimbalMsg_s));
    msgBus_.armorBoosterQueue = xQueueCreate(10, sizeof(ArmorBoosterMsg_s));
    msgBus_.armorLoaderQueue = xQueueCreate(10, sizeof(ArmorLoaderMsg_s));
#if APP_USE_REFEREE
    msgBus_.refereeQueue = xQueueCreate(10, sizeof(RefereeMsg_s));
#endif
#if EXTENSION_TOF
    msgBus_.tofLefDistQueue = xQueueCreate(10, sizeof(TOFMsg_s));
    msgBus_.tofRigDistQueue = xQueueCreate(10, sizeof(TOFMsg_s));
#endif

    for (auto i : Handler::getHandlerList()) {
        i.handler->init(&msgBus_, eventGroup_);
    }

    LOG::info("cmd", "init success");
}

void Cmd::parseMsg()
{
    EventBits_t xBits =
            xEventGroupWaitBits(eventGroup_, Handler::getMasks().to_ulong(), pdTRUE, pdFALSE, portMAX_DELAY);
    for (auto i : Handler::getHandlerList()) {
        if (xBits & i.bit) {
            i.handler->handle();
        }
    }
}

void Cmd::task()
{
    for (;;) {
        parseMsg();
    }
}
