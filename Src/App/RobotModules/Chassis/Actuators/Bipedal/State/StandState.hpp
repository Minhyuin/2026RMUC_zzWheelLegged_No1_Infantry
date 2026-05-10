#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

static float standLegLengthAngPidPD[2]{ 30.f, 0.0f };
static float standLegLengthVelPidPD[2]{ 90.f, 0.0f };
namespace CHASSIS {

class StandState : public FSMState<FSMState_e> {
    static constexpr float JUDGE_PHI = -0.f; // 用于判断站立策略

    static constexpr float STAND_PHI = 0.0f;
    static constexpr float STAND_L = WheelLegged::MIN_L;

    static constexpr float MAX_ACC_BACK = 1.f;
    static constexpr float MAX_ACC_FRONT = 1.f;

    static constexpr float BALANCE_DETECT_THESHOLD = 0.3f;
    static constexpr uint16_t MAX_BALANCE_MAINTAIN_TIME = 100;

    static constexpr float TARGET_PITCH_FORWARD = 0.1f;
    static constexpr float TARGET_PITCH_BACKWARD = 0.1f;

    static constexpr float S_FEEDFORWARD_FORWARD = -0.3f;
    static constexpr float S_FEEDFORWARD_BACKWARD = -0.6f;
    static constexpr uint16_t S_FEEDBACK_LOCK_TIME = 1000; // ms

public:
    StandState(Bipedal *_chassis) : FSMState<FSMState_e>(FSMState_e ::STAND), chassis_(_chassis) {}

    void enter() final
    {
        isBalance_ = false;
        balanceCnt_ = 0;
        chassis_->enter();
        startLength_ = STAND_L;
        startYaw_ = chassis_->yaw();
        chassis_->enableRoll();
        chassis_->setRoll(0.f);
        chassis_->setLegLengthAngPidPD(standLegLengthAngPidPD[0], standLegLengthAngPidPD[1]);
        chassis_->setLegLengthVelPidPD(standLegLengthVelPidPD[0], standLegLengthVelPidPD[1]);

        if ((chassis_->phiL() + chassis_->phiR()) / 2.f <= JUDGE_PHI) {
            // 腿在后面
            sFeedforward_ = S_FEEDFORWARD_BACKWARD;
            targetBalancePitch_ = TARGET_PITCH_BACKWARD;
            model_ = Model_e::WBR;
        } else {
            // 腿在前面
            sFeedforward_ = S_FEEDFORWARD_FORWARD;
            targetBalancePitch_ = TARGET_PITCH_FORWARD;
            model_ = Model_e::WBR;
        }

        LOG::info("ChassisStand", "enter");
    }

    void run() final
    {
        chassis_->setLength(startLength_);
        chassis_->setFdFeedforward();

        // ST
        if (model_ == Model_e::ST) {
            chassis_->setHipPhi(STAND_PHI);
            STController::States_s ref{};
            ref.q = targetBalancePitch_;
            ref.ds = std::clamp(chassis_->msg.vx, chassis_->vel() - MAX_ACC_BACK, chassis_->vel() + MAX_ACC_FRONT);
            chassis_->bal(ref, startYaw_);
        }
        // WBR
        else {
            WBRController::States_s ref{};
            float wz = chassis_->msg.wz * 0.001f;
            ref.ds = std::clamp(chassis_->msg.vx, chassis_->vel() - MAX_ACC_BACK, chassis_->vel() + MAX_ACC_FRONT);
            ref.yaw = startYaw_;
            ref.dyaw = wz;
            ref.q = targetBalancePitch_;
            chassis_->bal<WBRController::WBRControlMode_e::BALANCE>(ref);
        }

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
    enum class Model_e : uint8_t { ST, WBR } model_ = Model_e::WBR;
    uint16_t balanceCnt_ = 0;
    bool isBalance_ = false;
    float startLength_ = 0.f;
    float startYaw_ = 0.f;
    float targetBalancePitch_ = 0.f;
    float sFeedforward_ = 0.f;
};

} // namespace CHASSIS
