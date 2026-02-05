#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "PidBasic.hpp"

namespace CHASSIS {
class ClimbState : public FSMState<FSMState_e> {
    static constexpr float OFFSET_Q = 0.f;
    static constexpr float MAX_ACC_BACK = 0.4f;
    static constexpr float MAX_ACC_FRONT = 2.f;

    static constexpr float CLIMB_PHI = 1.6f;
    static constexpr float CLIMB_DPHI = 10.f * 0.001f;
    static constexpr float CLIMB_PHI_DETECT_THESHOLD = 0.2f;

public:
    ClimbState(Bipedal *_chassis) : FSMState<FSMState_e>(FSMState_e::CLIMB), chassis_(_chassis) {}

    void enter() final
    {
        chassis_->enter();
        refYaw_ = chassis_->yaw();
        chassis_->setRoll(0);
        refLength_ = (chassis_->lengthL() + chassis_->lengthR()) / 2.f;
        isClimbReady_ = false;
        LOG::info("ChassisClimb", " enter");
    }

    void run() final
    {
        refLength_ += chassis_->msg.dl * 0.001f;
        chassis_->setLength(refLength_);
        chassis_->setFdForward();

        if (chassis_->isFloating()) {
            refPhiL_ += CLIMB_DPHI;
            refPhiR_ += CLIMB_DPHI;
            chassis_->setHipPhiL(refPhiL_);
            chassis_->setHipPhiR(refPhiR_);
            if (chassis_->detectPhiLRange(CLIMB_PHI + CLIMB_PHI_DETECT_THESHOLD,
                                          CLIMB_PHI - CLIMB_PHI_DETECT_THESHOLD) &&
                chassis_->detectPhiRRange(CLIMB_PHI + CLIMB_PHI_DETECT_THESHOLD,
                                          CLIMB_PHI - CLIMB_PHI_DETECT_THESHOLD)) {
                isClimbReady_ = true;
            }
            chassis_->noBal(chassis_->msg.vx, 0);
        } else {
            refPhiL_ = chassis_->phiL();
            refPhiR_ = chassis_->phiR();

            // WBR
            WBRController::States_u ref{};
            float wz = chassis_->msg.wz * 0.001f;
            refYaw_ = chassis_->isFloating() ? chassis_->yaw() : (PINYMOTOR::rangeMap(refYaw_ + wz, -PI, PI));
            ref.ds = std::clamp(chassis_->msg.vx, chassis_->vel() - MAX_ACC_BACK, chassis_->vel() + MAX_ACC_FRONT);
            ref.yaw = refYaw_;
            ref.dyaw = wz;
            ref.q = OFFSET_Q;
            chassis_->bal(ref);
        }
        chassis_->ctrl();
    }

    void exit() final
    {
        wzAngPid_.reset();
        LOG::info("ChassisClimb", " exit");
    }


    FSMState_e checkChange() final
    {
        if (isClimbReady_)
            return FSMState_e::LAUNCH;
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (chassis_->msg.state == FSMState_e::IDLE)
            return FSMState_e::IDLE;
        else if (chassis_->msg.state == FSMState_e::SPIN)
            return FSMState_e::SPIN;
        else if (chassis_->msg.state == FSMState_e::CLIMB)
            return FSMState_e::CLIMB;
        else
            return FSMState_e::CLIMB;
    }

private:
    Bipedal *chassis_;
    PositionalPid wzAngPid_{ 0, 0, 0, 0.001f, 0, 0, 0 };
    float refYaw_ = 0;
    float refLength_ = Bipedal::DEFAULT_L;
    float refPhiL_ = 0.f;
    float refPhiR_ = 0.f;
    bool isClimbReady_ = false;
};

} // namespace CHASSIS
