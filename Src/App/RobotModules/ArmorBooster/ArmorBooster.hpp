#pragma once

#include "FSMState.hpp"

#include "MsgImpl.hpp"

class ArmorBooster {
public:
    ArmorBoosterMsg_s msg = {};

protected:
    StateFactory<ARMORBOOSTER::FSMState_e> stateFactory_;
};
