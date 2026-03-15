#include "WheelLegged.hpp"

#include "dsp/fast_math_functions.h"

using namespace CHASSIS;
using namespace PINYMOTOR;

void WheelLegged::init(PINYMOTOR::IMotor *_hip1, PINYMOTOR::IMotor *_hip4, PINYMOTOR::IMotor *_wheel)
{
    hip1_ = _hip1;
    hip4_ = _hip4;
    wheel_ = _wheel;
}

void WheelLegged::stop()
{
    tp_ = fd_ = fdFeedforward_ = tw_ = 0.f;

    lengthAngPid_.reset();
    lengthVelPid_.reset();

    hip1_->cmd(MotorCmdType_e::OFF);
    hip4_->cmd(MotorCmdType_e::OFF);
    wheel_->cmd(MotorCmdType_e::OFF);
}

void WheelLegged::enter()
{
    hip1_->cmd(MotorCmdType_e::ON);
    hip4_->cmd(MotorCmdType_e::ON);
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

float OFFSET_THRESHOLD = 0.f;
float OFFSET_THRESHOLD_FOR_LENGTH = 60.f;
bool WheelLegged::isFloating(float _az, float _q)
{
    float cosPhi0 = arm_cos_f32(legState_.phi0 + _q);
    float sinPhi0 = arm_sin_f32(legState_.phi0 + _q);
    fp_ = ((legState_.fd + fdFromSpring_) * cosPhi0) + (legState_.tp * sinPhi0 / legState_.L0);
    float azWheel = _az - (legState_.ad * cosPhi0) + (2.f * legState_.vd * legState_.dphi0 * sinPhi0) +
                    (legState_.L0 * legState_.ap * sinPhi0) + (legState_.L0 * powf(legState_.dphi0, 2) * cosPhi0);
    nWheel_ = ((azWheel + GRAVITY) * MASS_WHEEL);
    rawFn_ = nWheel_ + fp_;
    if (!isnan(rawFn_)) {
        fn_ = fnLPF_.process(rawFn_);
    }
    fnThreshold_ = FLOATING_THRESHOLD + OFFSET_THRESHOLD +
                   ((legState_.L0 - WheelLegged::MIN_L) / (WheelLegged::MAX_L - WheelLegged::MIN_L) *
                    OFFSET_THRESHOLD_FOR_LENGTH);
    return floatFlag = fn_ < fnThreshold_;
}

void WheelLegged::ctrl()
{
    fdFromSpring_ = spring_.computeForce(0); // TODO: maybe the spring force will change with other params
    float setVd = lengthAngPid_.calc(setL_, legState_.L0);
    fd_ = lengthVelPid_.calc(setVd, legState_.vd) + fdFeedforward_;
    if (!needJumping)
        fd_ -= fdFromSpring_;

    Matrix<2, 1> u = vmc_.forwardTorq(tp_, fd_);

    tau1_ = std::fmin(std::fmax(u(0, 0), -MAX_HIP_CTRL_TORQ), MAX_HIP_CTRL_TORQ);
    tau4_ = std::fmin(std::fmax(u(1, 0), -MAX_HIP_CTRL_TORQ), MAX_HIP_CTRL_TORQ);

    hip1_->cmdTorq(tau1_);
    hip4_->cmdTorq(tau4_);

    tw_ = std::fmin(std::fmax(tw_, -MAX_WHEEL_CTRL_TORQ), MAX_WHEEL_CTRL_TORQ);
    wheel_->cmdTorq(tw_);

    // hip1_->cmdTorq(0);
    // hip4_->cmdTorq(0);
    // wheel_->cmdTorq(0);
}

float WheelLegged::minL(float _phi) const
{
    // TODO: a fitted polynomial : minL = f(phi)
    (void)_phi;
    return MIN_L;
}

void WheelLegged::setLegLengthAngPidPD(float _kp, float _kd)
{
    lengthAngPid_.setParam(_kp, 0.f, _kd, 0.001f, 0.f, LENGTH_ANG_PID_OUTMAX);
}

void WheelLegged::setLegLengthVelPidPD(float _kp, float _kd)
{
    lengthVelPid_.setParam(_kp, 0.f, _kd, 0.001f, 0.f, LENGTH_VEL_PID_OUTMAX);
}

void WheelLegged::setL(float _l) { setL_ = fmin(fmax(_l, minL(legState_.phi0)), MAX_L); }

void WheelLegged::setFdFeedforward(float _fd) { fdFeedforward_ = _fd; }
void WheelLegged::setTp(float _tp) { tp_ = _tp; }
void WheelLegged::setTw(float _tw) { tw_ = _tw; }
