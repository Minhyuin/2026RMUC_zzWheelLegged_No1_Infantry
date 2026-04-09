#pragma once

#include "FSMState.hpp"
#include "TopicRouter.hpp"
#include "SuperCap.hpp"

class Chassis {
public:
    ChassisMsg_s msg = {};

protected:
    StateFactory<CHASSIS::FSMState_e> stateFactory_;
};
