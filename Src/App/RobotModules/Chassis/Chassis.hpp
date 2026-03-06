#pragma once

#include "FSMState.hpp"
#include "TopicRouter.hpp"

class Chassis {
public:
    ChassisMsg_s msg = {};

protected:
    StateFactory<CHASSIS::FSMState_e> stateFactory_;
};
