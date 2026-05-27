#include "Bipedal.hpp"

#include "MotorCommonMacros.hpp"

using namespace CHASSIS;
using namespace PINYMOTOR;

template <WBRController::WBRControlMode_e M> void Bipedal::bal(const WBRController::States_s &_ref)
{
    WBRController ::States_s errData;
    errData.s = _ref.s - s_;
    errData.ds = std::clamp(_ref.ds, -dsMaxB_, dsMaxF_) - ds_;
    errData.yaw = getMinorArc(_ref.yaw, insData_.yaw);
    errData.dyaw = _ref.dyaw - insData_.body.gz;
    errData.phil = getMinorArc(_ref.phil + EXPT_PHI_OFFSET, phiGlobalL());
    errData.dphil = _ref.dphil - (legL_.state().dphi0 + insData_.body.gy);
    errData.phir = getMinorArc(_ref.phir + EXPT_PHI_OFFSET, phiGlobalR());
    errData.dphir = _ref.dphir - (legR_.state().dphi0 + insData_.body.gy);
    errData.q = getMinorArc(_ref.q + EXPT_Q_OFFSET, insData_.pitch);
    errData.dq = _ref.dq - insData_.body.gy;
    Matrix<10, 1> err{ errData.data };
    auto k = WBRController::gain<M>(legL_.state().L0, legR_.state().L0);
    Matrix<4, 1> u = k * err;
    if (legL_.floatFlag) {
        u(0, 0) = 0;
        u(2, 0) = err(4, 0) * k(2, 4) + err(5, 0) * k(2, 5) + err(6, 0) * k(2, 6) + err(7, 0) * k(2, 7);
    }
    if (legR_.floatFlag) {
        u(1, 0) = 0;
        u(3, 0) = err(4, 0) * k(3, 4) + err(5, 0) * k(3, 5) + err(6, 0) * k(3, 6) + err(7, 0) * k(3, 7);
    }
    legL_.setTw(u(0, 0));
    legR_.setTw(u(1, 0));
    legL_.setTp(u(2, 0) - antiSplitTp_);
    legR_.setTp(u(3, 0) + antiSplitTp_);
}
template void Bipedal::bal<WBRController::WBRControlMode_e::BALANCE>(const WBRController::States_s &_ref);
template void Bipedal::bal<WBRController::WBRControlMode_e::SPIN>(const WBRController::States_s &_ref);
template void Bipedal::bal<WBRController::WBRControlMode_e::CLIMB>(const WBRController::States_s &_ref);
template void Bipedal::bal<WBRController::WBRControlMode_e::JUMP>(const WBRController::States_s &_ref);

void Bipedal::bal(const WLController::States_s &_ref, float _yaw) { bal(_ref, _ref, _yaw); }

void Bipedal::bal(const WLController::States_s &_refL, const WLController::States_s &_refR, float _yaw)
{
    WLController::States_s errDataL;
    errDataL.phi = getMinorArc(_refL.phi + EXPT_PHI_OFFSET, phiGlobalL());
    errDataL.dphi = _refL.dphi - (legL_.state().dphi0 + insData_.body.gy);
    errDataL.s = _refL.s - s_;
    errDataL.ds = std::clamp(_refL.ds, -dsMaxB_, dsMaxF_) - ds_;
    errDataL.q = getMinorArc(_refL.q + EXPT_Q_OFFSET, insData_.pitch);
    errDataL.dq = _refL.dq - insData_.body.gy;
    Matrix<6, 1> errL{ errDataL.data };
    WLController::States_s errDataR;
    errDataR.phi = getMinorArc(_refR.phi + EXPT_PHI_OFFSET, phiGlobalR());
    errDataR.dphi = _refR.dphi - (legR_.state().dphi0 + insData_.body.gy);
    errDataR.s = _refR.s - s_;
    errDataR.ds = std::clamp(_refR.ds, -dsMaxB_, dsMaxF_) - ds_;
    errDataR.q = getMinorArc(_refR.q + EXPT_Q_OFFSET, insData_.pitch);
    errDataR.dq = _refR.dq - insData_.body.gy;
    Matrix<6, 1> errR{ errDataR.data };
    Matrix<2, 1> uL = WLController::solve(errL, legL_.state().L0, legL_.floatFlag);
    Matrix<2, 1> uR = WLController::solve(errR, legR_.state().L0, legR_.floatFlag);
    float tr = yawPid_[1].calc(yawPid_[0].calc(_yaw, insData_.yaw), wz_);
    legL_.setTw(uL(0, 0) - tr);
    legL_.setTp(uL(1, 0) - antiSplitTp_);
    legR_.setTw(uR(0, 0) + tr);
    legR_.setTp(uR(1, 0) + antiSplitTp_);
}

template <STController::STControlMode_e M> void Bipedal::bal(const STController::States_s &_ref, float _yaw)
{
    STController::States_s errData;
    errData.s = _ref.s - s_;
    errData.q = -getMinorArc(_ref.q + EXPT_Q_OFFSET, insData_.pitch);
    errData.ds = std::clamp(_ref.ds, -dsMaxB_, dsMaxF_) - ds_;
    errData.dq = -(_ref.dq - insData_.body.gy);

    Matrix<4, 1> err{ errData.data };
    float lAvg = (legL_.state().L0 + legR_.state().L0) / 2.f;
    auto k = STController::gain<M>(lAvg);
    Matrix uDiv = k * err / 2;
    if (isFloating()) {
        uDiv(0, 0) = 0;
    }
    float tr = yawPid_[1].calc(yawPid_[0].calc(_yaw, insData_.yaw), wz_);
    legL_.setTw((uDiv(0, 0) * W_RADIUS) - tr);
    legR_.setTw((uDiv(0, 0) * W_RADIUS) + tr);

    float hipVelL = hipBalanceAngPidL_.calc(getMinorArc(EXPT_PHI_OFFSET, phiL()), 0);
    float tpL = hipBalanceVelPidL_.calc(hipVelL, legL_.state().dphi0);
    legL_.setTp(tpL - antiSplitTp_);
    float hipVelR = hipBalanceAngPidR_.calc(getMinorArc(EXPT_PHI_OFFSET, phiR()), 0);
    float tpR = hipBalanceVelPidR_.calc(hipVelR, legR_.state().dphi0);
    legR_.setTp(tpR + antiSplitTp_);
}

template void Bipedal::bal<STController::STControlMode_e::BALANCE>(const STController::States_s &_ref, float _yaw);

void Bipedal::noBal(float _vx, float _wz)
{
    float twL = dsPid_[0].calc(_vx + (_wz * W_DISTANCE_HALF), motors_.WheelL->vel() * W_RADIUS);
    float twR = dsPid_[1].calc(_vx - (_wz * W_DISTANCE_HALF), motors_.WheelR->vel() * W_RADIUS);
    legL_.setTw(twL);
    legR_.setTw(twR);
}
