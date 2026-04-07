#pragma once

#include <algorithm>
#include <cstdint>
#include "KF.hpp"
#include <cmath>

namespace CHASSIS {

class AdaptiveTracker {
    static constexpr float DEFAULT_DT = 0.001f;

    // Numerical constants used by adaptive tuning and CA discretization
    static constexpr float EPSILON = 1e-6f;

    static constexpr float WEIGHT_VEL_VAR = 0.45f;
    static constexpr float WEIGHT_ACC_VAR_SUPPRESS = 0.55f;
    static constexpr float WEIGHT_VEL_INNOVATION = 0.00f;

    static constexpr float SLIP_LPF_ALPHA = 0.000010f;
    static constexpr float SLIP_VEL_VAR_THRESHOLD = 0.02f;        // (m/s)^2
    static constexpr float SLIP_ACC_VAR_THRESHOLD = 5.00f;        // (m/s^2)^2
    static constexpr float SLIP_VEL_INNOVATION_THRESHOLD = 0.30f; // m/s

    static constexpr float VEL_R_SCALE_AT_SLIP = 5.f;   // speed confidence down: Rvv up
    static constexpr float ACC_R_SCALE_AT_SLIP = 0.01f; // acceleration confidence up: Raa down
    static constexpr float Q_SCALE_AT_SLIP = 2.5f;

    // Variance tracking speed (EMA): larger means faster response.
    // Rise/Fall split allows both slip-on and slip-off transitions to be quick.
    static constexpr float VAR_MEAN_ALPHA = 0.08f;
    static constexpr float VAR_RISE_ALPHA = 0.16f;
    static constexpr float VAR_FALL_ALPHA = 0.12f;

    // No slip if vel variance below this threshold; above this, slip likelihood increases rapidly
    static constexpr float SIGMA_Q = 0.001f;
    static constexpr float SIGMA_VEL_R = 0.001f;
    static constexpr float SIGMA_ACC_R = 1.25f;

    struct VarianceData_s {
        uint32_t n = 0;
        float mean = 0.f, center = 0.f, variance = 0.f, val = 0.f;

        void reset() { *this = VarianceData_s{}; }
    };

public:
    AdaptiveTracker()
    {
        CAKF.F = {
            { 1, DEFAULT_DT }, //
            { 0, 1 }           //
        };
        CAKF.H = Matrix<2, 2>::eye();
        CAKF.Q = Matrix<2, 2>::ones();
        CAKF.R(0, 0) = SIGMA_VEL_R;
        CAKF.R(1, 1) = SIGMA_ACC_R;
    }

    void updateCA(float _vel, float _acc) { updateCA(DEFAULT_DT, _vel, _acc); }

    void updateCA(float _dt, float _vel, float _acc)
    {
        // ========== DEFENSE LAYER 1: Input validation ==========
        // Reject NaN/Inf inputs before they corrupt variance statistics
        if (!std::isfinite(_dt) || (_dt <= 0.f) || !std::isfinite(_vel) || !std::isfinite(_acc)) {
            // Skip this update, keep last valid state
            return;
        }

        // update data variance
        updateVariance(accData_, _acc);
        updateVariance(velData_, _vel);

        // Reject invalid statistics to prevent NaN propagation into adaptive Q/R
        if (!std::isfinite(velData_.variance) || !std::isfinite(accData_.variance)) {
            velData_.reset();
            accData_.reset();
            return;
        }

        // --------- Slip-aware adaptive tuning ---------
        // Core idea:
        // 1) wheel speed variance rises -> wheel may slip, speed measurement less trustworthy
        // 2) then increase Rvv (reduce speed confidence), decrease Raa (increase acceleration confidence)
        // 3) use low-pass filtered slipFactor to avoid rapid matrix jitter
        const float velVarNorm = velData_.variance / (SLIP_VEL_VAR_THRESHOLD + EPSILON);
        const float accVarNorm = accData_.variance / (SLIP_ACC_VAR_THRESHOLD + EPSILON);

        // velocity innovation: if measured speed deviates much from estimated speed, slip likelihood increases
        const float velInnovation = std::fabs(_vel - CAKF.x(0));
        const float innovationNorm = velInnovation / (SLIP_VEL_INNOVATION_THRESHOLD + EPSILON);

        // emphasize velocity-side inconsistency; acceleration variance is a weak suppression term
        float slipRaw = ((WEIGHT_VEL_VAR * velVarNorm) + (WEIGHT_VEL_INNOVATION * innovationNorm)) -
                        (WEIGHT_ACC_VAR_SUPPRESS * accVarNorm);
        slipRaw = std::clamp(slipRaw, 0.f, 1.f);

        // First-order LPF: higher value means slower adaptation
        slipFactor_ = (SLIP_LPF_ALPHA * slipFactor_) + ((1.f - SLIP_LPF_ALPHA) * slipRaw);
        slipFactor_ = std::clamp(slipFactor_, 0.f, 1.f);

        // R adaptation:
        // slip=0 -> use nominal R
        // slip=1 -> trust wheel speed less, trust acceleration more
        const float velRScale = std::lerp(1.f, VEL_R_SCALE_AT_SLIP, slipFactor_);
        const float accRScale = std::lerp(1.f, ACC_R_SCALE_AT_SLIP, slipFactor_);
        adaptiveVelR = SIGMA_VEL_R * velRScale;
        adaptiveAccR = SIGMA_ACC_R * accRScale;

        // Optional Q adaptation: during slip, modestly increase process noise for faster model correction
        const float qScale = std::lerp(1.f, Q_SCALE_AT_SLIP, slipFactor_);
        adaptiveQ = SIGMA_Q * qScale;

        // update matrix
        const float dt2 = _dt * _dt;
        const float dt3 = _dt * dt2;
        CAKF.F(0, 1) = _dt;

        CAKF.Q(0, 0) = dt3 * adaptiveQ / 3.f;
        CAKF.Q(0, 1) = dt2 * adaptiveQ / 2.f;
        CAKF.Q(1, 0) = dt2 * adaptiveQ / 2.f;
        CAKF.Q(1, 1) = _dt * adaptiveQ;

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
    void updateVariance(VarianceData_s &_data, float _val)
    {
        // EMA (Exponential Moving Average) for mean and variance.
        // Exponentially weighted mean/variance for fast transient tracking.
        // Compared with cumulative variance, this has finite memory and reacts much faster.
        _data.val = _val;

        if (_data.n == 0U) {
            _data.n = 1U;
            _data.mean = _val;
            _data.center = 0.f;
            _data.variance = 0.f;
            return;
        }

        _data.n++; // 119 hour to UINT32_MAX, no overflow concern

        const float prevMean = _data.mean;
        const float meanDelta = _val - prevMean;
        _data.mean = prevMean + (VAR_MEAN_ALPHA * meanDelta);

        // Instantaneous squared residual around updated mean.
        const float residual = _val - _data.mean;
        const float instVar = residual * residual;
        const float varAlpha = (instVar > _data.variance) ? VAR_RISE_ALPHA : VAR_FALL_ALPHA;

        _data.variance += varAlpha * (instVar - _data.variance);
        _data.center = instVar;
    }

    VarianceData_s accData_{};
    VarianceData_s velData_{};

    // Adaptive parameter tuning based on measurement variance
    // When slip happens, measurement variance increases suddenly
    float adaptiveVelR = SIGMA_VEL_R;
    float adaptiveAccR = SIGMA_ACC_R;
    float adaptiveQ = SIGMA_Q;

    // ---------- Adaptive tuning parameters ----------
    float slipFactor_ = 0.f;
};

} // namespace CHASSIS
