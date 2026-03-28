#pragma once

#include "KF.hpp"
#include "StmLog.hpp"
#include <cmath>

namespace CHASSIS {

class SimpleTracker {
    static constexpr float DEFAULT_DT = 0.001f;

    struct WelFordData_s {
        uint16_t n = 0;
        float mean = 0.f, center = 0.f, variance = 0.f, val = 0.f;

        void reset() { *this = WelFordData_s{}; }
    };

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
        // ========== DEFENSE LAYER 1: Input validation ==========
        // Reject NaN/Inf inputs before they corrupt variance statistics
        if (!std::isfinite(_vel) || !std::isfinite(_acc)) {
            // Skip this update, keep last valid state
            return;
        }

        // update data variance
        updateVariance(accData_, _acc);
        updateVariance(velData_, _vel);

        // ========== DEFENSE LAYER 2: Variance validation ==========
        // Detect and isolate NaN in variance computation
        if (!std::isfinite(velData_.variance) || !std::isfinite(accData_.variance)) {
            // Reset statistics to recover from corruption
            velData_.reset();
            accData_.reset();
            return;
        }

        // Adaptive parameter tuning based on measurement variance
        // When slip happens, measurement variance increases suddenly
        // TODO: adaptive tuning
        float adaptiveVelR = sigmaVelR_;
        float adaptiveAccR = sigmaAccR_;
        float adaptiveQ = sigmaQ_;

        // update matrix
        const float dt2 = _dt * _dt;
        const float dt3 = _dt * dt2;
        CAKF.F(0, 1) = _dt;

        CAKF.Q(0, 0) = dt3 * adaptiveQ / 3.f;
        CAKF.Q(0, 1) = dt2 * adaptiveQ / 2.f;
        CAKF.Q(1, 0) = dt2 * adaptiveQ / 2.f;
        CAKF.Q(1, 1) = _dt * adaptiveQ / 1.f;

        CAKF.R(0, 0) = adaptiveVelR;
        CAKF.R(1, 1) = adaptiveAccR;

        CAKF.z(0, 0) = _vel;
        CAKF.z(1, 0) = _acc;

        // predict
        CAKF.process();

        // ========== DEFENSE LAYER 4: Kalman state validation ==========
        // After filtering, check if output became invalid
        if (!std::isfinite(CAKF.x(0)) || !std::isfinite(CAKF.x(1))) {
            // Kalman filter diverged, reset to recover
            CAKF.reset();
            velData_.reset();
            accData_.reset();
        }

        // DEBUG: Log variance for tuning threshold
        // LOG::info("Odom", "Var: vel=%.2e acc=%.2e | R=%.3f Q=%.2e | Slip:%d", velData_.variance, accData_.variance,
        //           adaptiveR, adaptiveQ, (velData_.variance > SLIP_VAR_THRESHOLD) ? 1 : 0);
    }

    const float &obsVel() const { return CAKF.x(0); }
    const float &obsAcc() const { return CAKF.x(1); }

    KF<2, 2> CAKF; // constant acceleration model

private:
    void updateVariance(WelFordData_s &_data, float _val)
    {
        // Welch-Ford algorithm for online variance computation - branch-free implementation
        _data.val = _val;
        _data.n++;
        float delta = _val - _data.mean;
        _data.mean += delta / static_cast<float>(_data.n);
        _data.center = delta * (_val - _data.mean); // incremental sum of squared differences

        // Branch-free variance calculation:
        // When n=1: divisor=1, center≈0 -> variance≈0
        // When n≥2: divisor=n-1 > 0 -> variance computed correctly
        // fmaxf ensures divisor never becomes 0 or negative, eliminating branch
        float divisor = fmaxf(1.0f, static_cast<float>(_data.n - 1));
        _data.variance = _data.center / divisor;
    }

    WelFordData_s accData_{};
    WelFordData_s velData_{};

    float sigmaQ_ = 10000.f;
    float sigmaVelR_ = 0.01f;
    float sigmaAccR_ = 1.25f;
};

} // namespace CHASSIS
