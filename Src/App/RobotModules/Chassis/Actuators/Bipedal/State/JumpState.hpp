#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace CHASSIS {

class JumpState : public FSMState<FSMState_e> {
    static constexpr float PREPARE_LENGTH = WheelLegged::MIN_L;
    static constexpr float JUMP_LENGTH = WheelLegged::MAX_L;

    static constexpr float JUMP_FD = 120.f;

    enum class JumpFSMState_e : uint8_t { PREPARE, JUMPING, SOAR, LAND };

public:
    JumpState(Bipedal *_chassis) : FSMState<FSMState_e>(FSMState_e::JUMP), chassis_(_chassis)
    {
        jumpFSMFactory_.addState(JumpFSMState_e::PREPARE, std::make_unique<PrepareState>(*this));
        jumpFSMFactory_.addState(JumpFSMState_e::JUMPING, std::make_unique<JumpingState>(*this));
        jumpFSMFactory_.addState(JumpFSMState_e::SOAR, std::make_unique<SoarState>(*this));
        jumpFSMFactory_.addState(JumpFSMState_e::LAND, std::make_unique<LandState>(*this));
    }

    void enter() final
    {
        chassis_->enter();
        jumpFinish_ = false;
        jumpFSMFactory_.init(jumpFSMFactory_.getNextState(JumpFSMState_e::PREPARE));
        LOG::info("ChassisJump", " enter");
    }

    void run() final
    {
        jumpFSMFactory_.update();
        WBRController::States_u ref{};
        ref.ds = chassis_->vel();
        ref.yaw = chassis_->yaw();
        chassis_->bal(ref);
        chassis_->ctrl();
    }

    void exit() final { LOG::info("ChassisJump", " exit"); }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;

        if (jumpFinish_)
            return FSMState_e::IDLE;
        else
            return FSMState_e::JUMP;
    }

private:
    Bipedal *chassis_;

    StateFactory<JumpFSMState_e> jumpFSMFactory_;

    bool jumpFinish_ = false;

    // Prepare jump state : shrink leg to be as short as possible
    class PrepareState : public FSMState<JumpFSMState_e> {
    public:
        explicit PrepareState(JumpState &_parent) : FSMState<JumpFSMState_e>(JumpFSMState_e::PREPARE), parent_(_parent)
        {
        }
        void enter() final { LOG::info("JumpPrepare", " enter"); }
        void run() final
        {
            parent_.chassis_->setLength(PREPARE_LENGTH);
            parent_.chassis_->setFdFeedforward();
        }
        void exit() final { LOG::info("JumpPrepare", " exit"); }
        JumpFSMState_e checkChange() final
        {
            if (parent_.chassis_->detectLengthRange(PREPARE_LENGTH))
                return JumpFSMState_e::JUMPING;
            else
                return JumpFSMState_e::PREPARE;
        }

    private:
        JumpState &parent_;
    };

    // Jumping state : extend leg rapidly by applying large force
    class JumpingState : public FSMState<JumpFSMState_e> {
    public:
        explicit JumpingState(JumpState &_parent) : FSMState<JumpFSMState_e>(JumpFSMState_e::JUMPING), parent_(_parent)
        {
        }
        void enter() final { LOG::info("Jumping", " enter"); }
        void run() final
        {
            parent_.chassis_->enableJump();
            parent_.chassis_->setFdFeedforward(JUMP_FD);
        }
        void exit() final { LOG::info("Jumping", " exit"); }
        JumpFSMState_e checkChange() final
        {
            if (parent_.chassis_->detectLengthRange(JUMP_LENGTH)) {
                return JumpFSMState_e::SOAR;
            } else
                return JumpFSMState_e::JUMPING;
        }

    private:
        JumpState &parent_;
    };

    // Soar state : in the air, shrink leg to cross obstacles
    class SoarState : public FSMState<JumpFSMState_e> {
    public:
        explicit SoarState(JumpState &_parent) : FSMState<JumpFSMState_e>(JumpFSMState_e::SOAR), parent_(_parent) {}
        void enter() final { LOG::info("JumpSoar", " enter"); }
        void run() final
        {
            parent_.chassis_->disbaleJump();
            parent_.chassis_->setLength(PREPARE_LENGTH);
            parent_.chassis_->setFdFeedforward(-WheelLegged::MASS * WheelLegged::GRAVITY);
        }
        void exit() final { LOG::info("JumpSoar", " exit"); }
        JumpFSMState_e checkChange() final
        {
            if (parent_.chassis_->detectLengthRange(PREPARE_LENGTH)) {
                return JumpFSMState_e::LAND;
            } else
                return JumpFSMState_e::SOAR;
        }

    private:
        JumpState &parent_;
    };

    // Land state : fall for landing (redundancy), then finish jump when touching ground
    class LandState : public FSMState<JumpFSMState_e> {
    public:
        explicit LandState(JumpState &_parent) : FSMState<JumpFSMState_e>(JumpFSMState_e::LAND), parent_(_parent) {}
        void enter() final { LOG::info("JumpLand", " enter"); }
        void run() final
        {
            parent_.chassis_->disbaleJump();
            parent_.chassis_->setLength(PREPARE_LENGTH);
            parent_.chassis_->setFdFeedforward(0.f);
        }
        void exit() final { LOG::info("JumpLand", " exit"); }
        JumpFSMState_e checkChange() final
        {
            if (!parent_.chassis_->isFloating()) {
                parent_.jumpFinish_ = true;
            }
            return JumpFSMState_e::LAND;
        }

    private:
        JumpState &parent_;
    };
};

} // namespace CHASSIS
