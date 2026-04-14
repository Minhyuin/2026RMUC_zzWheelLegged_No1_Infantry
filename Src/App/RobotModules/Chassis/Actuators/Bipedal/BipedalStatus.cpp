#include "Bipedal.hpp"

using namespace CHASSIS;

float Bipedal::getTranslationFrontAcc() const { return translationFrontAcc_; }

float Bipedal::getTranslationBackAcc() const { return translationBackAcc_; }

float Bipedal::getTranslationMaxFrontVel() const { return dsMaxF_; }

float Bipedal::getTranslationMaxBackVel() const { return dsMaxB_; }

float Bipedal::getRotationAcc() const { return rotationAcc_; }

float Bipedal::yaw() const { return yaw_; }

float Bipedal::pitch() const { return insData_.pitch; }

float Bipedal::roll() const { return insData_.roll; }

float Bipedal::phiL() const { return legL_.state().phi0; }

float Bipedal::phiR() const { return legR_.state().phi0; }

float Bipedal::phiGlobalL() const { return phiGlobalL_; }

float Bipedal::phiGlobalR() const { return phiGlobalR_; }

float Bipedal::lengthL() const { return legL_.state().L0; }

float Bipedal::lengthR() const { return legR_.state().L0; }

float Bipedal::length() const { return (legL_.state().L0 + legR_.state().L0) / 2.f; }

float Bipedal::vel() const { return ds_; }

float Bipedal::acc() const { return as_; }

float Bipedal::wz() const { return wz_; }

float Bipedal::wheelVel() const { return (motors_.WheelL->vel() + motors_.WheelR->vel()) * W_RADIUS / 2.f; }