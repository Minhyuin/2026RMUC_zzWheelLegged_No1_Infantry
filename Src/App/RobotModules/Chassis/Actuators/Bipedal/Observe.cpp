#include "Bipedal.hpp"
#include "MotorCommonMacros.hpp"

using namespace CHASSIS;
using namespace PINYMOTOR;


bool Bipedal::isGimbalReset() const
{
    // return chassisCtrlRxPacket_.data().gimbalReset;
    return true;
}

bool Bipedal::isTumble() const
{
    // TODO: maybe this is a weighted linear regression
    return fabs(phiL()) > 1.f || fabs(phiR()) > 1.f;
}

bool Bipedal::isFreeze() const
{
    static constexpr float DANG_FREEZE_THRESHOLD = 0.2f;
    static constexpr uint16_t FREEZE_MAX_TIME = 700;
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

bool Bipedal::isOverturn() const { return insData_.body.az > 0.1f; }

bool Bipedal::isFloating() const { return legL_.floatFlag && legR_.floatFlag; }

bool Bipedal::detectFloating()
{
    return legL_.isFloating(az_, insData_.pitch) && legR_.isFloating(az_, insData_.pitch);
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

bool Bipedal::detectPitchRange(float _ang1, float _ang2) const
{
    return getMinorArc(_ang1, insData_.pitch) > 0 && getMinorArc(_ang2, insData_.pitch) < 0;
}

/**
 * @brief detect if the leg is in the swing phase
 * 
 * @param _ang1 clockwise leading phase
 * @param _ang2 clockwise lag phase
 * @return true if the leg is in the swing phase
 * @return false if the leg is in the stance phase
 */
bool Bipedal::detectPhiLRange(float _ang1, float _ang2) const
{
    return getMinorArc(_ang1, legL_.state().phi0) > 0 && getMinorArc(_ang2, legL_.state().phi0) < 0;
}

bool Bipedal::detectPhiRRange(float _ang1, float _ang2) const
{
    return getMinorArc(_ang1, legR_.state().phi0) > 0 && getMinorArc(_ang2, legR_.state().phi0) < 0;
}

/**
 * @brief detect if the leg is in the target length
 * 
 * @param _l target length
 * @param _threthold tolerance
 * @return true if the leg is in the target length
 * @return false if the leg is in the target length
 */

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

void Bipedal::updateVel()
{
    // update ax az
    float sinPitch = arm_sin_f32(insData_.pitch);
    float cosPitch = arm_cos_f32(insData_.pitch);
    float cosRoll = arm_cos_f32(insData_.roll);

    az_ = insData_.earth.az + WheelLegged::GRAVITY;

    // update odom
    // w = w_eb + dq + w_ecd
    float wheelOmega[2]{};
    // wheelOmega[0] = wheelFltr[0].process(motors_.WheelL->vel()) + insData_.body.gy + legL_.state().dphi0;
    // wheelOmega[1] = wheelFltr[1].process(motors_.WheelR->vel()) - insData_.body.gy + legR_.state().dphi0;
    wheelOmega[0] = motors_.WheelL->vel() + insData_.body.gy + legL_.state().dphi0;
    wheelOmega[1] = motors_.WheelR->vel() - insData_.body.gy + legR_.state().dphi0;

    // x_dot = w * r + L * phi_dot * cos(phi) + L_dot * sin(phi)
    float wheelVel[2]{};
    wheelVel[0] = wheelOmega[0] * W_RADIUS + legL_.state().L0 * legL_.state().dphi0 * arm_cos_f32(legL_.state().phi0) +
                  legL_.state().vd * arm_sin_f32(legL_.state().phi0);
    wheelVel[1] = wheelOmega[1] * W_RADIUS + legR_.state().L0 * legR_.state().dphi0 * arm_cos_f32(legR_.state().phi0) +
                  legR_.state().vd * arm_sin_f32(legR_.state().phi0);
    float vxFromWheel = (wheelVel[0] + wheelVel[1]) / 2.f;

    if (!isnan(vxFromWheel))
        odom_.update(dt_, vxFromWheel, insData_.earth.ax, yaw_);

    // ds_ = odom_.normVel();
    ds_ = vxFromWheel;
    as_ = odom_.normAcc();

    // stop detect
    static bool slowSpeedFlag = false;
    if (detectNeedStop(this->msg.vx)) {
        s_ = clamp(s_ + (ds_ * dt_), -MAX_S, MAX_S);
        if (fabs(insData_.earth.ax) < 0.0025f && !slowSpeedFlag) {
            // LOG::info("sForward", "%f", s_);
        }
        if (ds_ < 0.1f && !slowSpeedFlag) {
            slowSpeedFlag = true;
            if (dsCmdBeforeStop_ > 0.f)
                s_ = S_FEEDBACK_FORWARD;
            else
                s_ = S_FEEDBACK_BACKWARD;
        }
    } else {
        s_ = 0;
        slowSpeedFlag = false;
    }
    // if (isFloating())
    //     s_ = 0;
}