#include "Bipedal.hpp"

#include "MotorCommonMacros.hpp"

using namespace CHASSIS;
using namespace PINYMOTOR;

void Bipedal::bal(const WBRController::States_u &_ref)
{
    WBRController ::States_u errData = {
        .s = _ref.s - s_,
        .ds = _ref.ds - ds_,
        .yaw = getMinorArc(_ref.yaw, insData_.yaw),
        .dyaw = _ref.dyaw - insData_.body.gz,
        .phil = getMinorArc(_ref.phil, legL_.state().phi0),
        .dphil = _ref.dphil - legL_.state().dphi0,
        .phir = getMinorArc(_ref.phir, legR_.state().phi0),
        .dphir = _ref.dphir - legR_.state().dphi0,
        .q = getMinorArc(_ref.q, insData_.pitch),
        .dq = _ref.dq - insData_.body.gy
    };
    Matrix<10, 1> err{ errData._ };
    Matrix<4, 1> u = WBRController::solve(err, legL_.state().L0,
                                          legR_.state().L0, legL_.isFloating,
                                          legR_.isFloating);

    // u[0][0] = isnan(u[0][0]) ? u[0][0] : twLPF_[0].process(u[0][0]);
    // u[1][0] = isnan(u[1][0]) ? u[1][0] : twLPF_[1].process(u[1][0]);
    // u[2][0] = isnan(u[2][0]) ? u[2][0] : tpLPF_[0].process(u[2][0]);
    // u[3][0] = isnan(u[3][0]) ? u[3][0] : tpLPF_[1].process(u[3][0]);

    legL_.setTw(u[0][0]);
    legR_.setTw(u[1][0]);
    legL_.setTp(u[2][0]);
    legR_.setTp(u[3][0]);
}

void Bipedal::bal(const WLController::States_u &_ref, float _wz)
{
    bal(_ref, _ref, _wz);
}

void Bipedal::bal(const WLController::States_u &_refL,
                  const WLController::States_u &_refR, float _wz)
{
    WLController::States_u errDataL = {
        .phi = getMinorArc(legL_.state().phi0, _refL.phi),
        .dphi = legL_.state().dphi0,
        .s = s_,
        .ds = ds_,
        .q = getMinorArc(insData_.pitch, _refL.q),
        .dq = insData_.body.gy,
    };
    Matrix<6, 1> errL{ errDataL._ };
    WLController::States_u errDataR = {
        .phi = getMinorArc(_refR.phi, legR_.state().phi0),
        .dphi = _refR.dphi - legR_.state().dphi0,
        .s = _refR.s - s_,
        .ds = _refR.ds - ds_,
        .q = getMinorArc(_refR.q, insData_.pitch),
        .dq = _refR.dq - insData_.body.gy,
    };
    Matrix<6, 1> errR{ errDataR._ };
    Matrix<2, 1> uL =
            WLController::solve(errL, legL_.state().L0, legL_.isFloating);
    Matrix<2, 1> uR =
            WLController::solve(errR, legR_.state().L0, legR_.isFloating);
    float tw2 = wzVelPid_.calc(_wz, wz_);
    legL_.setTw(uL[0][0] + tw2);
    legL_.setTp(uL[1][0]);
    legR_.setTw(uR[0][0] - tw2);
    legR_.setTp(uR[1][0]);
}

void Bipedal::bal(const STController::States_u &_ref, float _wz)
{
    STController::States_u errData = {
        .s = _ref.s - s_,
        .ds = _ref.ds - ds_,
        .q = getMinorArc(_ref.q, insData_.pitch),
        .dq = _ref.dq - insData_.body.gy,
    };
    Matrix<4, 1> err{ errData._ };
    float lAvg = (legL_.state().L0 + legR_.state().L0) / 2.f;
    bool isf = legL_.isFloating & legR_.isFloating;
    Matrix<1, 1> uDiv = STController::solve(err, lAvg, isf) / 2.f;
    float tw2 = wzVelPid_.calc(_wz, wz_);
    legL_.setTw(uDiv[0][0] + tw2);
    legR_.setTw(uDiv[0][0] - tw2);
}

void Bipedal::noBal(float _vx, float _wz)
{
    float twL = dsPid_[0].calc(_vx + (_wz * B_RADIUS),
                               motors_.WheelL->vel() * W_RADIUS);
    float twR = dsPid_[1].calc(_vx - (_wz * B_RADIUS),
                               motors_.WheelR->vel() * W_RADIUS);
    legL_.setTw(twL);
    legR_.setTw(twR);
}