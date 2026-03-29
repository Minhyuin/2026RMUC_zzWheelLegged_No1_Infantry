#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "Smooth.hpp"

namespace CHASSIS {

class SpinState : public FSMState<FSMState_e> {
    static constexpr float MAX_WZ = 10.f;
    static constexpr float SPIN_SPEED_BASE = 8.f;
    static constexpr float SPIN_SPEED_VAR = 5.f;
    static constexpr float SPIN_L = 0.17f;

public:
    SpinState(Bipedal *_chassis) : FSMState<FSMState_e>(FSMState_e::SPIN), chassis_(_chassis) {}

    void enter() final
    {
        chassis_->enter();
        refYaw_ = chassis_->yaw();
        refLength_ = chassis_->length();
        targetRefLength_ = SPIN_L;
        chassis_->setRoll(0);
        LOG::info("ChassisSpin", " enter");
    }

    void run() final
    {
        targetRefLength_ += chassis_->msg.dl * 0.001f;
        refLength_ = SMOOTH::slopeFollowing(targetRefLength_, refLength_, 0.0002f);
        chassis_->setLength(refLength_);
        chassis_->setFdFeedforward();

        // WBR
        WBRController::States_s ref{};
        float wz = getSpinVel(spinCnt_++);
        wz = std::clamp(wz, -MAX_WZ, MAX_WZ) * 0.001f; // convert to rad/ms
        refYaw_ = chassis_->isFloating() ? chassis_->yaw() : (PINYMOTOR::rangeMap(refYaw_ + wz, -PI, PI));
        ref.ds = 0; // TODO: no spin ds temporary
        ref.yaw = refYaw_;
        ref.dyaw = wz;
        chassis_->bal(ref);

        chassis_->ctrl();
    }

    void exit() final
    {
        spinCnt_ = 0;
        spinDir_ = !spinDir_;
        LOG::info("ChassisSpin", " exit");
    }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP || chassis_->isTumble())
            return FSMState_e::STOP;
        else if (chassis_->msg.state != FSMState_e::SPIN)
            return FSMState_e::SPIN_EXITING;
        else
            return FSMState_e::SPIN;
    }

    bool change() final
    {
        WBRController::States_s ref{};
        ref.yaw = chassis_->yaw();
        chassis_->bal(ref);
        chassis_->ctrl();
        return std::fabs(chassis_->vel()) < 0.3f;
    }

private:
    Bipedal *chassis_;

    float refYaw_ = 0.f;
    float refLength_ = 0.f;
    float targetRefLength_ = SPIN_L;

    bool spinDir_ = false; // true : left, false: right
    uint32_t spinCnt_ = 0;

    float getSpinVel(uint32_t _cnt)
    {
        // float x = 2.1f * (static_cast<float>(_cnt) * 0.001f);
        // float rslt = (SPIN_SPEED_VAR * sinf(x) * sinf(x)) + SPIN_SPEED_BASE;
        // return spinDir_ ? rslt : -rslt;
        return spinDir_ ? SPIN_SPEED_BASE : -SPIN_SPEED_BASE;
    }
};
} // namespace CHASSIS
