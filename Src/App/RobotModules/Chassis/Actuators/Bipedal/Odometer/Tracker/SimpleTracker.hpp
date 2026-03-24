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

    // ============ Adaptive Kalman Parameters ============
    // Measurement noise covariance (R) bounds and scaling
    static constexpr float R_BASELINE = 0.5f;   // Nominal measurement uncertainty
    static constexpr float R_MAX_BOUND = 50.0f; // Upper limit during slip detection
    static constexpr float R_VAR_SCALE = 0.01f; // Scaling factor: variance → R

    // Process noise covariance (Q) bounds and thresholds
    static constexpr float Q_MAX_NORMAL = 4e6f;        // Q value during normal operation (trust model)
    static constexpr float Q_MIN_SLIP = 1e4f;          // Q value during wheel slip (trust measurements)
    static constexpr float SLIP_VAR_THRESHOLD = 5e-5f; // Tuned: Normal<1e-6, Slip peak=7.86e-5
    static constexpr float SLIP_VAR_UPPER = 1e-4f;     // Tuned: Slip variance upper bound

    // Q adaptation factors
    static constexpr float Q_SLIP_BLEND_FACTOR = 0.95f;  // Interpolation sensitivity during slip
    static constexpr float Q_NORMAL_DAMP_FACTOR = 0.05f; // Damping applied during normal operation
    static constexpr float Q_NORMAL_DAMP_DIVISOR = 1e4f; // Variance normalization for damping
    static constexpr float Q_MIN_DAMP = 0.5f;            // Minimum damping factor

    // Measurement variance weighting
    static constexpr float ACC_VAR_WEIGHT = 0.1f; // Acceleration variance contribution to R


public:
    SimpleTracker()
    {
        const float &dt = DEFAULT_DT;
        const float dt2 = dt * dt;
        const float dt3 = dt * dt2;
        CAKF.F = {
            { 1, DEFAULT_DT }, //
            { 0, 1 }           //
        };
        CAKF.H = Matrix<2, 2>::eye();
        CAKF.Q = {
            { dt3 * sigmaQ_ / 3.f, dt2 * sigmaQ_ / 2.f }, //
            { dt2 * sigmaQ_ / 2.f, dt * sigmaQ_ / 1.f }   //
        };
        CAKF.R = Matrix<2, 2>::eye() * sigmaR_;
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
        float adaptiveR = computeAdaptiveR();
        float adaptiveQ = computeAdaptiveQ();

        // ========== DEFENSE LAYER 3: Parameter validation ==========
        // Ensure adaptive parameters are always finite and bounded
        if (!std::isfinite(adaptiveR) || !std::isfinite(adaptiveQ)) {
            // Fallback to safe defaults
            adaptiveR = R_BASELINE;
            adaptiveQ = Q_MAX_NORMAL;
        }

        // update matrix
        const float dt2 = _dt * _dt;
        const float dt3 = _dt * dt2;
        CAKF.F(0, 1) = _dt;

        CAKF.Q(0, 0) = dt3 * adaptiveQ / 3.f;
        CAKF.Q(0, 1) = dt2 * adaptiveQ / 2.f;
        CAKF.Q(1, 0) = dt2 * adaptiveQ / 2.f;
        CAKF.Q(1, 1) = _dt * adaptiveQ / 1.f;

        CAKF.R(0, 0) = adaptiveR;
        CAKF.R(1, 1) = adaptiveR;

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

    /**
     * Compute measurement noise covariance adaptively.
     * When wheel slip occurs, measurement variance increases suddenly.
     * R should reflect the actual measurement uncertainty.
     */
    float computeAdaptiveR()
    {
        // Safety clamp for variance (prevent Inf propagation)
        float safeVelVar = fmin(velData_.variance, 1e8f);
        float safeAccVar = fmin(accData_.variance, 1e8f);

        // Use velocity measurement variance as primary indicator
        // Add acceleration variance with lower weight for slip detection
        float totalVar = safeVelVar + (ACC_VAR_WEIGHT * safeAccVar);

        // Map measurement variance to R value
        // Higher variance → higher R (trust measurements less, filter more)
        float adaptiveR = R_BASELINE + (totalVar * R_VAR_SCALE);

        return fmin(fmax(adaptiveR, R_BASELINE), R_MAX_BOUND);
    }

    /**
     * Compute process noise covariance adaptively.
     * The CA model assumption may break down during slip.
     * Reduce Q when model mismatch is detected via high measurement variance.
     */
    float computeAdaptiveQ()
    {
        // Safety clamp for variance (prevent Inf/NaN propagation)
        float safeVar = fmin(velData_.variance, 1e8f);
        safeVar = fmax(safeVar, 0.0f); // Ensure non-negative

        if (safeVar > SLIP_VAR_THRESHOLD) {
            // Wheel slip detected: reduce model confidence
            // Interpolate from Q_MAX_NORMAL to Q_MIN_SLIP based on variance magnitude
            float ratio = fmin((safeVar - SLIP_VAR_THRESHOLD) / (SLIP_VAR_UPPER - SLIP_VAR_THRESHOLD), 1.0f);
            return (Q_MAX_NORMAL * (1.0f - Q_SLIP_BLEND_FACTOR * ratio)) + (Q_MIN_SLIP * (Q_SLIP_BLEND_FACTOR * ratio));
        }

        // Normal operation: use full model confidence with slight adaptive damping
        float dampingFactor = 1.0f - (Q_NORMAL_DAMP_FACTOR * (safeVar / Q_NORMAL_DAMP_DIVISOR));
        dampingFactor = fmax(dampingFactor, Q_MIN_DAMP);

        return Q_MAX_NORMAL * dampingFactor;
    }

    WelFordData_s accData_{};
    WelFordData_s velData_{};

    float sigmaQ_ = Q_MAX_NORMAL; // fallback/initialization value
    float sigmaR_ = R_BASELINE;   // fallback/initialization value
};

} // namespace CHASSIS
