#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

namespace CHASSIS {

class SpinState : public FSMState<FSMState_e> {
    static constexpr float MAX_SPIN_SPEED = 5.0f;
    static constexpr float MAX_VX_ACC = 1.4f;

public:
    SpinState(Bipedal *_chassis) : FSMState<FSMState_e>(FSMState_e::SPIN), chassis_(_chassis) {}

    void enter() final
    {
        chassis_->enter();
        refYaw_ = chassis_->yaw();
        refLength_ = (chassis_->lengthL() + chassis_->lengthR()) / 2.f;
        chassis_->setRoll(0);
        LOG::info("ChassisSpin", " enter");
    }

    void run() final
    {
        refLength_ += chassis_->msg.dl * 0.001f;
        chassis_->setLength(refLength_);
        chassis_->setFdForward();

        // WBR
        WBRController::States_u ref{};
        float wz = getSpinVel(spinCnt_++) * 0.001f;
        refYaw_ = chassis_->isFloating() ? chassis_->yaw() : (PINYMOTOR::rangeMap(refYaw_ + wz, -PI, PI));
        ref.ds = std::clamp(chassis_->msg.vx, chassis_->vel() - MAX_VX_ACC, chassis_->vel() + MAX_VX_ACC);
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
        else if (chassis_->msg.state == FSMState_e::IDLE)
            return FSMState_e::IDLE;
        else if (chassis_->msg.state == FSMState_e::SPIN)
            return FSMState_e::SPIN;
        else if (chassis_->msg.state == FSMState_e::CLIMB)
            return FSMState_e::CLIMB;
        else
            return FSMState_e::SPIN;
    }

    /// @brief speed change spinning
    float getSpinVel(uint32_t _cnt)
    {
        float x = 2.1f * (static_cast<float>(_cnt) * 0.001f);
        float rslt = (MAX_SPIN_SPEED * sinf(x) * sinf(x)) + 2.5f;
        return spinDir_ ? rslt : -rslt;
    }

private:
    Bipedal *chassis_;

    float refYaw_ = 0.f;
    float refLength_ = 0.f;

    bool spinDir_ = false; // true : left, false: right
    uint32_t spinCnt_ = 0;
};
} // namespace CHASSIS
