#include "Bipedal.hpp"

#include "MotorCommonMacros.hpp"

using namespace CHASSIS;
using namespace PINYMOTOR;

void Bipedal::enableWheels()
{
    motors_.WheelL->cmd(MotorCmdType_e::ON);
    motors_.WheelR->cmd(MotorCmdType_e::ON);
}

void Bipedal::enableJump()
{
    legL_.needJumping = true;
    legR_.needJumping = true;
}

void Bipedal::disableJump()
{
    legL_.needJumping = false;
    legR_.needJumping = false;
}

void Bipedal::disableHip()
{
    legL_.setTp(0);
    legR_.setTp(0);
    legL_.setLegLengthAngPidPD(0, 0);
    legR_.setLegLengthAngPidPD(0, 0);
    legL_.setLegLengthVelPidPD(0, 0);
    legR_.setLegLengthVelPidPD(0, 0);
    legL_.setFdFeedforward(0);
    legR_.setFdFeedforward(0);
}

void Bipedal::enableRoll() { rollControlFlag = true; }

void Bipedal::disableRoll() { rollControlFlag = false; }

void Bipedal::enableFdLimit()
{
    legL_.setFdLimitFlag(true);
    legR_.setFdLimitFlag(true);
}

void Bipedal::disableFdLimit()
{
    legL_.setFdLimitFlag(false);
    legR_.setFdLimitFlag(false);
}

void Bipedal::increaseS(float _deltaS) { s_ += _deltaS; }

void Bipedal::feedS(float _s) { s_ = _s; }

void Bipedal::lockS(uint16_t _durationMs) { lockSCount_ = _durationMs; }

void Bipedal::setRoll(float _roll) { setRoll_ = std::clamp(_roll, -MAX_ABS_ROLL, MAX_ABS_ROLL); }

void Bipedal::setLegLengthAngPidPD(float _kp, float _kd, float _outMax)
{
    legL_.setLegLengthAngPidPD(_kp, _kd, _outMax);
    legR_.setLegLengthAngPidPD(_kp, _kd, _outMax);
}

void Bipedal::setLegLengthAngPidPD(float _kp, float _kd)
{
    legL_.setLegLengthAngPidPD(_kp, _kd);
    legR_.setLegLengthAngPidPD(_kp, _kd);
}

void Bipedal::setLegLengthVelPidPD(float _kp, float _kd, float _outMax)
{
    legL_.setLegLengthVelPidPD(_kp, _kd, _outMax);
    legR_.setLegLengthVelPidPD(_kp, _kd, _outMax);
}

void Bipedal::setLegLengthVelPidPD(float _kp, float _kd)
{
    legL_.setLegLengthVelPidPD(_kp, _kd);
    legR_.setLegLengthVelPidPD(_kp, _kd);
}

void Bipedal::setLengthSmooth(float _l, float _step) { setLengthSmooth(_l, _l, _step); }
void Bipedal::setLengthSmooth(float _lL, float _lR, float _step)
{
    float deltaL = _lL - legL_.state().L0;
    deltaL = std::clamp(deltaL, -_step, _step);
    float deltaR = _lR - legR_.state().L0;
    deltaR = std::clamp(deltaR, -_step, _step);
    setLength(legL_.state().L0 + deltaL, legR_.state().L0 + deltaR);
}
void Bipedal::setLengthSmoothL(float _l, float _step)
{
    float delta = _l - legL_.state().L0;
    delta = std::clamp(delta, -_step, _step);
    setLengthL(legL_.state().L0 + delta);
}
void Bipedal::setLengthSmoothR(float _l, float _step)
{
    float delta = _l - legR_.state().L0;
    delta = std::clamp(delta, -_step, _step);
    setLengthR(legR_.state().L0 + delta);
}

void Bipedal::setLength(float _l) { setLength(_l, _l); }

void Bipedal::setLength(float _lL, float _lR)
{
    setLengthL(_lL + lengthOffset_);
    setLengthR(_lR - lengthOffset_);
}

void Bipedal::setLengthL(float _l) { legL_.setL(_l); }

void Bipedal::setLengthR(float _l) { legR_.setL(_l); }

static constexpr float INERATIAL_FD_FEEDFORWARD_GAIN = 3.5f;
void Bipedal::setFdFeedforward()
{
    // F_gravity = 1/2 * M * g * cos(phi)
    // F_inertial = 1/2 * M * (L / 2 / R_l * dyaw * ds)
    float gravityF = 0.5f * MASS_BODY * WheelLegged::GRAVITY * arm_cos_f32(phiGlobalL());
    float inertialF = 0.5f * MASS_BODY * (0.5f * lengthL() / W_DISTANCE_HALF * insData_.body.gz * ds_);
    float fdL = gravityF - (inertialF * INERATIAL_FD_FEEDFORWARD_GAIN);
    gravityF = 0.5f * MASS_BODY * WheelLegged::GRAVITY * arm_cos_f32(phiGlobalR());
    inertialF = 0.5f * MASS_BODY * (0.5f * lengthR() / W_DISTANCE_HALF * insData_.body.gz * ds_);
    float fdR = gravityF + (inertialF * INERATIAL_FD_FEEDFORWARD_GAIN);
    setFdFeedforward(fdL, fdR);
}

void Bipedal::setFdFeedforward(float _fd) { setFdFeedforward(_fd, _fd); }

void Bipedal::setFdFeedforward(float _fdL, float _fdR)
{
    legL_.setFdFeedforward(_fdL);
    legR_.setFdFeedforward(_fdR);
}

void Bipedal::setHipPhiRot(float _dphi) { setHipPhiRot(_dphi, _dphi); }
void Bipedal::setHipPhiRot(float _dphiL, float _dphiR)
{
    setHipPhiRotL(_dphiL);
    setHipPhiRotR(_dphiR);
}

void Bipedal::setHipPhiRotL(float _dphi)
{
    if (_dphi == 0)
        hipAngPidL_.reset();
    setHipPhiL(legL_.state().phi0 + (_dphi * 0.001f));
}
void Bipedal::setHipPhiRotR(float _dphi)
{
    if (_dphi == 0)
        hipAngPidR_.reset();
    setHipPhiR(legR_.state().phi0 + (_dphi * 0.001f));
}

void Bipedal::setHipPhi(float _phi) { setHipPhi(_phi, _phi); }

void Bipedal::setHipPhi(float _phiL, float _phiR)
{
    setHipPhiL(_phiL);
    setHipPhiR(_phiR);
}

void Bipedal::setHipPhiL(float _phi)
{
    float hipVelL = hipAngPidL_.calc(getMinorArc(_phi, phiL()), 0);
    float tpL = hipVelPidL_.calc(hipVelL, legL_.state().dphi0);
    legL_.setTp(tpL);
}

void Bipedal::setHipPhiR(float _phi)
{
    float hipVelR = hipAngPidR_.calc(getMinorArc(_phi, phiR()), 0);
    float tpR = hipVelPidR_.calc(hipVelR, legR_.state().dphi0);
    legR_.setTp(tpR);
}

void Bipedal::setHipPhiGlobal(float _phi) { setHipPhiGlobal(_phi, _phi); }

void Bipedal::setHipPhiGlobal(float _phiL, float _phiR)
{
    setHipPhiGlobalL(_phiL);
    setHipPhiGlobalR(_phiR);
}

void Bipedal::setHipPhiGlobalL(float _phi)
{
    float hipVelL = hipAngPidL_.calc(getMinorArc(_phi, phiGlobalL()), 0);
    float tpL = hipVelPidL_.calc(hipVelL, legL_.state().dphi0 + insData_.body.gy);
    legL_.setTp(tpL);
}

void Bipedal::setHipPhiGlobalR(float _phi)
{
    float hipVelR = hipAngPidR_.calc(getMinorArc(_phi, phiGlobalR()), 0);
    float tpR = hipVelPidR_.calc(hipVelR, legR_.state().dphi0 + insData_.body.gy);
    legR_.setTp(tpR);
}

void Bipedal::setHipTp(float _tp) { setHipTp(_tp, _tp); }

void Bipedal::setHipTp(float _tpL, float _tpR)
{
    legL_.setTp(_tpL);
    legR_.setTp(_tpR);
}

void Bipedal::resetPhiPid()
{
    hipAngPidL_.reset();
    hipVelPidL_.reset();
    hipAngPidR_.reset();
    hipVelPidR_.reset();
}

void Bipedal::resetPhiBalancePid()
{
    hipBalanceAngPidL_.reset();
    hipBalanceVelPidL_.reset();
    hipBalanceAngPidR_.reset();
    hipBalanceVelPidR_.reset();
}

void Bipedal::needGimbalLaunch(bool _need)
{
    gimbalLaunchStrategyTxPacket_.setData().isLegLFront = detectPhiLRange(-3.14f, -0.1f);
    gimbalLaunchStrategyTxPacket_.setData().isLegRFront = detectPhiRRange(-3.14f, -0.1f);
    gimbalLaunchStrategyTxPacket_.setData().isNeedGimbalLaunch = _need;
}

void Bipedal::allowGimbalExitLaunch(bool _allow) { gimbalLaunchStrategyTxPacket_.setData().isAllowExitLaunch = _allow; }