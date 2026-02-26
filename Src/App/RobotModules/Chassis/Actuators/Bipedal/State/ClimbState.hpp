#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "PidBasic.hpp"

namespace CHASSIS {
class ClimbState : public FSMState<FSMState_e> {
    static constexpr float CLIMB_PHI = 1.6f;
    static constexpr float CLIMB_DPHI = 3.f * 0.001f;
    static constexpr float CLIMB_PHI_DETECT_THESHOLD = 0.2f;

public:
    ClimbState(Bipedal *_chassis) : FSMState<FSMState_e>(FSMState_e::CLIMB), chassis_(_chassis) {}

    void enter() final
    {
        chassis_->enter();
        refLength_ = (chassis_->lengthL() + chassis_->lengthR()) / 2.f;
        isClimbReady_[0] = isClimbReady_[1] = false;
        LOG::info("ChassisClimb", " enter");
    }

    void run() final
    {
        chassis_->setLength(refLength_);
        chassis_->setFdFeedforward();

        refPhiR_ += CLIMB_DPHI;
        if (chassis_->detectPhiGlobalLRange(CLIMB_PHI + CLIMB_PHI_DETECT_THESHOLD,
                                            CLIMB_PHI - CLIMB_PHI_DETECT_THESHOLD) ||
            isClimbReady_[0]) {
            isClimbReady_[0] = true;
            refPhiL_ = chassis_->phiL();
        } else {
            refPhiL_ += CLIMB_DPHI;
        }
        chassis_->setHipPhiL(refPhiL_);

        if (chassis_->detectPhiGlobalRRange(CLIMB_PHI + CLIMB_PHI_DETECT_THESHOLD,
                                            CLIMB_PHI - CLIMB_PHI_DETECT_THESHOLD) ||
            isClimbReady_[1]) {
            isClimbReady_[1] = true;
            refPhiR_ = chassis_->phiR();
        } else {
            refPhiR_ += CLIMB_DPHI;
        }
        chassis_->setHipPhiR(refPhiR_);

        chassis_->noBal(chassis_->msg.vx, 0);

        chassis_->ctrl();
    }

    void exit() final
    {
        wzAngPid_.reset();
        LOG::info("ChassisClimb", " exit");
    }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        if (isClimbReady_[0] && isClimbReady_[1])
            return FSMState_e::LAUNCH;
        else
            return FSMState_e::CLIMB;
    }

private:
    Bipedal *chassis_;
    PositionalPid wzAngPid_{ 0, 0, 0, 0.001f, 0, 0, 0 };
    float refLength_ = Bipedal::DEFAULT_L;
    float refPhiL_ = 0.f;
    float refPhiR_ = 0.f;
    bool isClimbReady_[2] = { false, false };
};

} // namespace CHASSIS
