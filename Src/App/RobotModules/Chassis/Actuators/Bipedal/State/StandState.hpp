#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace CHASSIS {

class StandState : public FSMState<FSMState_e> {
    static constexpr float STAND_PHI = 0.f;
    static constexpr float STAND_L = 0.2f;

    static constexpr float BALANCE_DETECT_THESHOLD = 0.2f;
    static constexpr uint16_t MAX_BALANCE_MAINTAIN_TIME = 300;

    static constexpr float TARGET_PITCH_FORWARD = 0.13f;
    static constexpr float TARGET_PITCH_BACKWARD = 0.f;

    static constexpr float S_FEEDBACK_FORWARD = -1.0f;
    static constexpr float S_FEEDBACK_BACKWARD = -0.9f;
    static constexpr uint16_t S_FEEDBACK_LOCK_TIME = 500; // ms

public:
    StandState(Bipedal *_chassis) : FSMState<FSMState_e>(FSMState_e ::STAND), chassis_(_chassis) {}

    void enter() final
    {
        isBalance_ = false;
        balanceCnt_ = 0;
        chassis_->enter();
        startLength_ = STAND_L;
        startYaw_ = chassis_->yaw();

        if ((chassis_->phiL() + chassis_->phiR()) / 2.f >= 0.05f) {
            // 腿在后面
            sFeedforward_ = S_FEEDBACK_FORWARD;
            targetBalancePitch_ = TARGET_PITCH_FORWARD;
        } else {
            // 腿在前面
            sFeedforward_ = S_FEEDBACK_BACKWARD;
            targetBalancePitch_ = TARGET_PITCH_BACKWARD;
        }

        LOG::info("ChassisStand", "enter");
    }

    void run() final
    {
        chassis_->setLength(startLength_);

        chassis_->setHipPhi(0.f);
        STController::States_u ref{};
        float vx = chassis_->msg.vx;
        ref.q = targetBalancePitch_;
        ref.ds = vx;
        chassis_->bal(ref, startYaw_);

        if (chassis_->detectPitchRange(BALANCE_DETECT_THESHOLD, -BALANCE_DETECT_THESHOLD)) {
            balanceCnt_++;
        }
        if (balanceCnt_ > MAX_BALANCE_MAINTAIN_TIME) {
            isBalance_ = true;
            LOG::info("ChassisStand", "balance");
        }

        chassis_->ctrl();
    }

    void exit() final
    {
        chassis_->feedS(sFeedforward_);
        chassis_->lockS(S_FEEDBACK_LOCK_TIME);
        LOG::info("ChassisStand", "exit");
    }

    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (isBalance_)
            // return FSMState_e::STAND;
            return FSMState_e::IDLE;
        // TODO: the other state
        else
            return FSMState_e::STAND;
    }

private:
    Bipedal *chassis_;
    uint16_t balanceCnt_ = 0;
    bool isBalance_ = false;
    float startLength_ = 0.f;
    float startYaw_ = 0.f;
    float targetBalancePitch_ = 0.f;
    float sFeedforward_ = 0.f;
};

} // namespace CHASSIS
