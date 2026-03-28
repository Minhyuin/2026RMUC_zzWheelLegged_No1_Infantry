#pragma once

#include "KF.hpp"
#include <cmath>

namespace CHASSIS {

class SimpleTracker {
    static constexpr float DEFAULT_DT = 0.001f;

public:
    SimpleTracker()
    {
        CAKF.F = {
            { 1, DEFAULT_DT }, //
            { 0, 1 }           //
        };
        CAKF.H = Matrix<2, 2>::eye();
        CAKF.Q = Matrix<2, 2>::ones();
        CAKF.R(0, 0) = sigmaVelR_;
        CAKF.R(1, 1) = sigmaAccR_;
    }

    void updateCA(float _vel, float _acc) { updateCA(DEFAULT_DT, _vel, _acc); }

    void updateCA(float _dt, float _vel, float _acc)
    {
        // Reject NaN/Inf inputs before they corrupt variance statistics
        if (!std::isfinite(_vel) || !std::isfinite(_acc)) {
            return;
        }

        // update matrix
        const float dt2 = _dt * _dt;
        const float dt3 = _dt * dt2;
        CAKF.F(0, 1) = _dt;

        CAKF.Q(0, 0) = dt3 * sigmaQ_ / 3.f;
        CAKF.Q(0, 1) = dt2 * sigmaQ_ / 2.f;
        CAKF.Q(1, 0) = dt2 * sigmaQ_ / 2.f;
        CAKF.Q(1, 1) = _dt * sigmaQ_ / 1.f;

        CAKF.R(0, 0) = sigmaVelR_;
        CAKF.R(1, 1) = sigmaAccR_;

        CAKF.z(0, 0) = _vel;
        CAKF.z(1, 0) = _acc;

        // predict
        CAKF.process();
    }

    const float &obsVel() const { return CAKF.x(0); }
    const float &obsAcc() const { return CAKF.x(1); }

    KF<2, 2> CAKF; // constant acceleration model

private:
    float sigmaQ_ = 0.001f;     // process noise
    float sigmaVelR_ = 0.0005f; // measurement noise for velocity
    float sigmaAccR_ = 5.f;     // measurement noise for acceleration
};

} // namespace CHASSIS
