#include "Bipedal.hpp"
#include "MotorCommonMacros.hpp"
using namespace CHASSIS;
using namespace PINYMOTOR;

bool Bipedal::isGimbalReset() const
{
    return chassisCtrlRxPacket_.data().gimbalReset;
    // return true;
}

bool Bipedal::isTumble() const
{
    // TODO: maybe this is a weighted linear regression
    return fabs(phiGlobalL()) > 1.1f || fabs(phiGlobalR()) > 1.1f || fabs(phiGlobalL()) < -0.8f ||
           fabs(phiGlobalR()) < -0.8f;
}

static constexpr float DANG_FREEZE_THRESHOLD = 0.2f;
static constexpr uint16_t FREEZE_MAX_TIME = 500;
bool Bipedal::isFreeze() const
{
    static uint16_t freezeTime = 0;
    if (fabs(insData_.body.gx) < DANG_FREEZE_THRESHOLD && fabs(insData_.body.gy) < DANG_FREEZE_THRESHOLD &&
        fabs(insData_.body.gz) < DANG_FREEZE_THRESHOLD)
        freezeTime++;
    else
        freezeTime = 0;
    if (freezeTime > FREEZE_MAX_TIME) {
        freezeTime = 0;
        return true;
    } else {
        return false;
    }
}

bool Bipedal::isOverturn() const { return azBody_ > -3.f; }

bool Bipedal::isFloating() const { return legL_.floatFlag && legR_.floatFlag; }

bool Bipedal::detectFloating()
{
    // 合一起写会被编译优化
    bool l = legL_.isFloating(az_, insData_.pitch);
    bool r = legR_.isFloating(az_, insData_.pitch);
    return l && r;
}

bool Bipedal::detectNeedStop(float _expVel)
{
    static constexpr float STOP_CMD_THRESHOLD = 0.3f;
    if ((_expVel > -STOP_CMD_THRESHOLD && _expVel < STOP_CMD_THRESHOLD)) {
        return true;
    } else {
        dsCmdBeforeStop_ = _expVel;
        return false;
    }
}

void Bipedal::detectStop()
{
    if (lockSCount_ > 0)
        lockSCount_--;
    static bool slowSpeedFlag = false;
    if (detectNeedStop(this->msg.vx)) {
        s_ = clamp(s_ + (ds_ * dt_), -MAX_S, MAX_S);
        if (fabs(insData_.earth.ax) < 0.0025f && !slowSpeedFlag) {
            // LOG::info("sForward", "%f", s_);
        }
        if (ds_ < 0.1f && !slowSpeedFlag) {
            slowSpeedFlag = true;
            if (dsCmdBeforeStop_ > 0.f)
                s_ = S_FEEDFORWARD_FORWARD;
            else
                s_ = S_FEEDFORWARD_BACKWARD;
        }
    } else {
        s_ = 0;
        slowSpeedFlag = false;
    }
    if (isFloating() && lockSCount_ == 0)
        s_ = 0;
}

bool Bipedal::detectPitchRange(float _endAng, float _startAng) const { return detectAng(pitch(), _startAng, _endAng); }

bool Bipedal::detectPhiLRange(float _endAng, float _startAng) const { return detectAng(phiL(), _startAng, _endAng); }

bool Bipedal::detectPhiRRange(float _endAng, float _startAng) const { return detectAng(phiR(), _startAng, _endAng); }

bool Bipedal::detectPhiGlobalLRange(float _endAng, float _startAng) const
{
    return detectAng(phiGlobalL(), _startAng, _endAng);
}

bool Bipedal::detectPhiGlobalRRange(float _endAng, float _startAng) const
{
    return detectAng(phiGlobalR(), _startAng, _endAng);
}

bool Bipedal::detectLengthRange(float _l, float _threthold) const
{
    return detectLengthLRange(_l, _threthold) && detectLengthRRange(_l, _threthold);
}

bool Bipedal::detectLengthLRange(float _l, float _threthold) const
{
    return legL_.state().L0 < _l + _threthold && legL_.state().L0 > _l - _threthold;
}
bool Bipedal::detectLengthRRange(float _l, float _threthold) const
{
    return legR_.state().L0 < _l + _threthold && legR_.state().L0 > _l - _threthold;
}

bool Bipedal::detectAng(float _input, float _startAng, float _endAng) const
{
    // NOLINTBEGIN(readability-suspicious-call-argument)
    return getMinorArc(_endAng, _input) > 0 && getMinorArc(_startAng, _input) < 0;
    // NOLINTEND(readability-suspicious-call-argument)
}

void Bipedal::updateBody()
{
    // update odom
    // w = w_eb + dq + w_ecd
    float wheelOmega[2]{};
    wheelOmega[0] = motors_.WheelL->vel() + insData_.body.gy + legL_.state().dphi0;
    wheelOmega[1] = motors_.WheelR->vel() - insData_.body.gy + legR_.state().dphi0;

    // x_dot = w * r + L * phi_dot * cos(phi) + L_dot * sin(phi)
    float wheelVel[2]{};
    wheelVel[0] = wheelOmega[0] * W_RADIUS + legL_.state().L0 * legL_.state().dphi0 * arm_cos_f32(legL_.state().phi0) +
                  legL_.state().vd * arm_sin_f32(legL_.state().phi0);
    wheelVel[1] = wheelOmega[1] * W_RADIUS + legR_.state().L0 * legR_.state().dphi0 * arm_cos_f32(legR_.state().phi0) +
                  legR_.state().vd * arm_sin_f32(legR_.state().phi0);
    wheelVel[0] = wheelVelFilter_[0].process(wheelVel[0]);
    wheelVel[1] = wheelVelFilter_[1].process(wheelVel[1]);
    float vxFromWheel = (wheelVel[0] + wheelVel[1]) / 2.f;

    if (!isnan(vxFromWheel))
        odom_.update(dt_, vxFromWheel, insData_.earth.ax, yaw_);

    ds_ = odom_.normVel();
    // ds_ = vxFromWheel;
    as_ = odom_.normAcc();

    az_ = insData_.earth.az + WheelLegged::GRAVITY;
    azBody_ = azBodyFilter_.process(insData_.body.az);

    yaw_ = insData_.yaw;

    wz_ = insData_.earth.gz;
    wx_ = insData_.earth.gx;
}
