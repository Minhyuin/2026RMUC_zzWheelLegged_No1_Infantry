#pragma once

#include "FSMState.hpp"
#include "MsgImpl.hpp"

class ArmorLoader {
public:
    ArmorLoaderMsg_s msg = {};

protected:
    StateFactory<ARMORLOADER::FSMState_e> stateFactory_;
};
