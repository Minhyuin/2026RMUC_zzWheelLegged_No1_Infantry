#include "WheelLegged.hpp"

#include "dsp/fast_math_functions.h"

using namespace CHASSIS;
using namespace PINYMOTOR;

void WheelLegged::init(PINYMOTOR::IMotor *_hip1, PINYMOTOR::IMotor *_hip4,
                       PINYMOTOR::IMotor *_wheel)
{
    hip1_ = _hip1;
    hip4_ = _hip4;
    wheel_ = _wheel;
}

void WheelLegged::stop()
{
    tp_ = fd_ = fdForward_ = tw_ = 0.f;

    lengthAngPid_.reset();
    lengthVelPid_.reset();

    fdLPF_.reset();

    // hip1_->disable();
    // hip4_->disable();
    // wheel_->disable();
    hip1_->cmd(MotorCmdType_e::OFF);
    hip4_->cmd(MotorCmdType_e::OFF);
    wheel_->cmd(MotorCmdType_e::OFF);
}

void WheelLegged::enter()
{
    // hip1_->enable();
    // hip4_->enable();
    // wheel_->enable();
    // hip1_->cmd(MotorCmdType_e::ON);
    // hip4_->cmd(MotorCmdType_e::ON);
    wheel_->cmd(MotorCmdType_e::ON);
}

void WheelLegged::stateUpdate(float _dt)
{
    // TODO: maybe need a filter to smooth data
    legState_.phi1 = hip1_->posNorm();
    legState_.phi4 = hip4_->posNorm();

    legState_.dphi1 = hip1_->vel();
    legState_.dphi4 = hip4_->vel();

    legState_.tau1 = hip1_->torq();
    legState_.tau4 = hip4_->torq();

    vmc_.update(legState_, _dt);
}

float WheelLegged::calcFn(float _az, float _q)
{
    float cosPhi0 = arm_cos_f32(legState_.phi0 + _q);
    float sinPhi0 = arm_sin_f32(legState_.phi0 + _q);
    fp_ = (legState_.fd * cosPhi0) + (legState_.tp * sinPhi0 / legState_.L0);
    float azWheel = _az - (legState_.ad * cosPhi0) +
                    (2.f * legState_.vd * legState_.dphi0 * sinPhi0) +
                    (legState_.L0 * legState_.ap * sinPhi0) +
                    (legState_.L0 * powf(legState_.dphi0, 2) * cosPhi0);
    nWheel_ = ((azWheel + GRAVITY) * MASS_WHEEL);
    rawFn_ = nWheel_ + fp_;
    if (isnan(rawFn_)) {
        return fn_;
    } else {
        return fn_ = fnLPF_.process(rawFn_);
    }
}

void WheelLegged::ctrl()
{
    float setVd = lengthAngPid_.calc(setL_, legState_.L0);
    // float setVd = lengthAngPid_.calc(legState_.L0, setL_);
    fd_ = lengthVelPid_.calc(setVd, legState_.vd) + fdForward_;

    Matrix<2, 1> u = vmc_.forwardTorq(tp_, fdLPF_.process(fd_));

    u[0][0] = std::fmin(std::fmax(u[0][0], -MAX_HIP_CTRL_TORQ),
                        MAX_HIP_CTRL_TORQ);
    u[1][0] = std::fmin(std::fmax(u[1][0], -MAX_HIP_CTRL_TORQ),
                        MAX_HIP_CTRL_TORQ);

    hip1_->cmdTorq(u[0][0]);
    hip4_->cmdTorq(u[1][0]);

    tw_ = std::fmin(std::fmax(tw_, -MAX_WHEEL_CTRL_TORQ), MAX_WHEEL_CTRL_TORQ);
    wheel_->cmdTorq(tw_);

    // hip1_->cmdTorq(0);
    // hip4_->cmdTorq(0);
    // wheel_->cmdTorq(0);
}

float WheelLegged::minL(float _phi) const
{
    // TODO: a fitted polynomial : minL = f(phi)
    return MIN_L;
}

void WheelLegged::setL(float _l)
{
    setL_ = fmin(fmax(_l, minL(legState_.phi0)), MAX_L);
}


void WheelLegged::setFdForward(float _fd) { fdForward_ = _fd; }
void WheelLegged::setTp(float _tp) { tp_ = _tp; }
void WheelLegged::setTw(float _tw) { tw_ = _tw; }
