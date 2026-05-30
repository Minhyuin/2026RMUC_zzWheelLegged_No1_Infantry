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
    bool result = false;
    result |= fabs(phiGlobalL()) > 0.7f || fabs(phiGlobalR()) > 0.7f || fabs(pitch()) > 0.35f;
    result |= phiL() > 1.f || phiR() > 1.f;
    result |= phiL() < -0.4f || phiR() < -0.4f;
    return result;
}

static constexpr float DANG_FREEZE_THRESHOLD = 0.3f;
static constexpr uint16_t FREEZE_MAX_TIME = 100;
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
    static constexpr float STOP_CMD_THRESHOLD = 0.1f;
    static constexpr float STOP_VEL_FRONT_THRESHOLD = 0.6f;
    static constexpr float STOP_VEL_BACK_THRESHOLD = -0.7f;
    if (std::fabs(_expVel) < STOP_CMD_THRESHOLD && STOP_VEL_BACK_THRESHOLD < ds_ && ds_ < STOP_VEL_FRONT_THRESHOLD) {
        return true;
    } else {
        if (_expVel != 0.f)
            dsCmdBeforeStop_ = _expVel;
        return false;
    }
}

void Bipedal::detectStop()
{
    if (lockSCount_ > 0)
        lockSCount_--;
    static bool slowSpeedFlag = false;
    static float prevDs = 0.f;
    static bool prevDsValid = false;
    if (lockSCount_ == 0) {
        if (detectNeedStop(this->msg.vx)) {
            if (!prevDsValid) {
                prevDs = ds_;
                prevDsValid = true;
            }
            // Trapezoidal integration: s(k)=s(k-1)+((v(k-1)+v(k))/2)*dt
            const float dsAvg = 0.5f * (prevDs + ds_);
            const float feedforwardFromLeg = (legL_.state().vd * arm_cos_f32(legL_.state().phi0)) +
                                             (legR_.state().vd * arm_cos_f32(legR_.state().phi0));
            s_ = clamp(s_ + (dsAvg * dt_) + (feedforwardFromLeg * S_FEEDFORWARD_LEG_VD_GAIN), -MAX_S, MAX_S);
            prevDs = ds_;
            if (!slowSpeedFlag) {
                slowSpeedFlag = true;
                if (dsCmdBeforeStop_ > 0.f)
                    s_ = S_FEEDFORWARD_FORWARD;
                else
                    s_ = S_FEEDFORWARD_BACKWARD;
            }
        } else {
            s_ = 0;
            slowSpeedFlag = false;
            prevDs = ds_;
            prevDsValid = true;
        }
        if (isFloating()) {
            s_ = 0;
            prevDs = ds_;
            prevDsValid = true;
        }
    }
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
    wheelOmega[0] = wheelVelFltr_[0].process(motors_.WheelL->vel()) + insData_.body.gy + legL_.state().dphi0;
    wheelOmega[1] = wheelVelFltr_[1].process(motors_.WheelR->vel()) + insData_.body.gy + legR_.state().dphi0;

    // x_dot = w * r + L * phi_dot * cos(phi) + L_dot * sin(phi)
    float wheelVel[2]{};
    wheelVel[0] = wheelOmega[0] * W_RADIUS + legL_.state().L0 * legL_.state().dphi0 * arm_cos_f32(legL_.state().phi0) +
                  legL_.state().vd * arm_sin_f32(legL_.state().phi0);
    wheelVel[1] = wheelOmega[1] * W_RADIUS + legR_.state().L0 * legR_.state().dphi0 * arm_cos_f32(legR_.state().phi0) +
                  legR_.state().vd * arm_sin_f32(legR_.state().phi0);
    vxFromWheel_ = (wheelVel[0] + wheelVel[1]) / 2.f;

    if (!isnan(vxFromWheel_))
        odom_.update(dt_, vxFromWheel_, insData_.earth.ax, yaw_);

    ds_ = odom_.normVel();
    // ds_ = vxFromWheel_;
    as_ = odom_.normAcc();

    az_ = insData_.earth.az + WheelLegged::GRAVITY;
    azBody_ = azBodyFilter_.process(insData_.body.az);

    yaw_ = insData_.yaw;

    wz_ = insData_.earth.gz;
    wx_ = insData_.earth.gx;
}

void Bipedal::updatePowerLimit()
{
    auto powerLimitFactor = std::max(powerLimit_ - 80.f, 0.f) / (120.f - 80.f);
    static constexpr auto MAX_VEL_ATTENUATION_FACTOR = 0.3f;
    static constexpr auto LENGTH_RANGE = WheelLegged::MAX_L - WheelLegged::MIN_L;
    auto normalizedLength = std::clamp((length() - WheelLegged::MIN_L) / LENGTH_RANGE, 0.f, 1.f);
    auto lengthAttenuationFactor = 1.f - (normalizedLength * MAX_VEL_ATTENUATION_FACTOR);
    static constexpr auto RANGE_TRANSLATION_FRONT_ACC = MAX_TRANSLATION_FRONT_ACC - BASE_TRANSLATION_FRONT_ACC;
    static constexpr auto RANGE_TRANSLATION_BACK_ACC = MAX_TRANSLATION_BACK_ACC - BASE_TRANSLATION_BACK_ACC;
    static constexpr auto RANGE_FRONT_VEL = MAX_VEL_RUSH_FRONT - BASE_VEL_FRONT;
    static constexpr auto RANGE_BACK_VEL = MAX_VEL_RUSH_BACK - BASE_VEL_BACK;
    static constexpr auto RANGE_ROTATION_ACC = MAX_ROTATION_ACC - BASE_ROTATION_ACC;
    if (msg.needRush) {
        dsMaxF_ = MAX_VEL_RUSH_FRONT * lengthAttenuationFactor;
        dsMaxB_ = MAX_VEL_RUSH_BACK * lengthAttenuationFactor;
        translationFrontAcc_ = MAX_TRANSLATION_FRONT_ACC * lengthAttenuationFactor;
        translationBackAcc_ = MAX_TRANSLATION_BACK_ACC * lengthAttenuationFactor;
        rotationAcc_ = MAX_ROTATION_ACC * lengthAttenuationFactor;
    } else {
        dsMaxF_ = (BASE_VEL_FRONT + powerLimitFactor * RANGE_FRONT_VEL) * lengthAttenuationFactor;
        dsMaxB_ = (BASE_VEL_BACK + powerLimitFactor * RANGE_BACK_VEL) * lengthAttenuationFactor;
        translationFrontAcc_ =
                BASE_TRANSLATION_FRONT_ACC + powerLimitFactor * RANGE_TRANSLATION_FRONT_ACC * lengthAttenuationFactor;
        translationBackAcc_ =
                BASE_TRANSLATION_BACK_ACC + powerLimitFactor * RANGE_TRANSLATION_BACK_ACC * lengthAttenuationFactor;
        rotationAcc_ = (BASE_ROTATION_ACC + powerLimitFactor * RANGE_ROTATION_ACC) * lengthAttenuationFactor;
    }
}