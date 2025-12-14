#include "Bipedal.hpp"
#include "MotorCommonMacros.hpp"

using namespace CHASSIS;
using namespace PINYMOTOR;


bool Bipedal::isGimbalReset() const { return chassisCtrlRxPacket_.data().gimbalReset; }

bool Bipedal::isTumble() { return detector_.detectTumble(legL_.state(), legR_.state(), insData_); }

bool Bipedal::isFloating() const { return legL_.isFloating || legR_.isFloating; }

bool Bipedal::isFreeze() { return detector_.detectFreeze(insData_); }

bool Bipedal::isOverturn() { return insData_.body.az > 0.1f; }

bool Bipedal::detectPitch(float _ang1, float _ang2)
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
bool Bipedal::detectPhiL(float _ang1, float _ang2)
{
    return getMinorArc(_ang1, legL_.state().phi0) > 0 && getMinorArc(_ang2, legL_.state().phi0) < 0;
}

bool Bipedal::detectPhiR(float _ang1, float _ang2)
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

bool Bipedal::detectLength(float _l, float _threthold)
{
    return detectLengthL(_l, _threthold) && detectLengthR(_l, _threthold);
}

bool Bipedal::detectLengthL(float _l, float _threthold)
{
    return legL_.state().L0 < _l + _threthold && legL_.state().L0 > _l - _threthold;
}
bool Bipedal::detectLengthR(float _l, float _threthold)
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
    wheelOmega[0] = wheelFltr[0].process(motors_.WheelL->vel()) + insData_.body.gy + legL_.state().dphi0;
    wheelOmega[1] = wheelFltr[1].process(motors_.WheelR->vel()) - insData_.body.gy + legR_.state().dphi0;

    // x_dot = w * r + L * phi_dot * cos(phi) + L_dot * sin(phi)
    float wheelVel[2]{};
    wheelVel[0] = wheelOmega[0] * W_RADIUS + legL_.state().L0 * legL_.state().dphi0 * arm_cos_f32(legL_.state().phi0) +
                  legL_.state().vd * arm_sin_f32(legL_.state().phi0);
    wheelVel[1] = wheelOmega[1] * W_RADIUS + legR_.state().L0 * legR_.state().dphi0 * arm_cos_f32(legR_.state().phi0) +
                  legR_.state().vd * arm_sin_f32(legR_.state().phi0);
    float vxFromWheel = (wheelVel[0] + wheelVel[1]) / 2.f;

    odom_.update(dt_, vxFromWheel, insData_.earth.ax, yaw_);

    // ds_ = odom_.normVel();
    ds_ = vxFromWheel;
    as_ = odom_.normAcc();

    // stop detect
    bool slowSpeedFlag = false;
    if (detector_.isStop(this->msg.vx, ds_)) {
        s_ = clamp(s_ + (ds_ * dt_), -MAX_S, MAX_S);
        if (fabs(insData_.earth.ax) < 0.0025f && !slowSpeedFlag) {
            // LOG::info("sForward", "%f", s_);
        }
        if (ds_ < 0.1f && !slowSpeedFlag) {
            slowSpeedFlag = true;
            // s_ = sForward_;
        }
    } else {
        s_ = 0;
        slowSpeedFlag = false;
    }
    if (isFloating())
        s_ = 0;
}