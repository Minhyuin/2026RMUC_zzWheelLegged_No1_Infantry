#pragma once

#include "Matrix.hpp"

namespace CHASSIS {

class VMCTest {
public:
    Matrix<2, 1> inverseTorq(float _t1, float _t4, float _phi1, float _phi2,
                             float _phi3, float _phi4, float _l0)
    {
        updateJ(_phi1, _phi2, _phi3, _phi4);
        updateR(_phi1, _phi2);
        updateM(_l0);

        J_T_[0][0] = J[0][0];
        J_T_[0][1] = J[1][0];
        J_T_[1][0] = J[0][1];
        J_T_[1][1] = J[1][1];

        J_T_inv_ = inv(J_T_);
        R_inv_ = inv(R);
        M_inv_ = inv(M);

        float fx = (J_T_inv_[0][0] * _t1) + (J_T_inv_[0][1] * _t4);
        float fy = (J_T_inv_[1][0] * _t1) + (J_T_inv_[1][1] * _t4);

        float fq = (R_inv_[0][0] * fx) + (R_inv_[0][1] * fy);
        float fs = (R_inv_[1][0] * fx) + (R_inv_[1][1] * fy);

        Matrix<2, 1> tqtd;
        tqtd[0][0] = (M_inv_[0][0] * fq) + (M_inv_[0][1] * fs);
        tqtd[1][0] = (M_inv_[1][0] * fq) + (M_inv_[1][1] * fs);
        return tqtd;
    }

    Matrix<2, 1> inverseVel(float _w1, float _w4, float _phi1, float _phi2,
                            float _phi3, float _phi4, float _l0)
    {
        updateJ(_phi1, _phi2, _phi3, _phi4);
        updateR(_phi1, _phi2);
        updateM(_l0);
        float vx = (J[0][0] * _w1) + (J[0][1] * _w4);
        float vy = (J[1][0] * _w1) + (J[1][1] * _w4);
        Matrix<2, 1> vqd;
        vqd[0][0] = (R[0][0] * vx) + (R[0][1] * vy);
        vqd[1][0] = (R[1][0] * vx) + (R[1][1] * vy);
        return vqd;
    }

private:
    float L1_ = 0.215f;
    float L2_ = 0.258f;
    // float L3_ = 0.258f;
    float L4_ = 0.215f;
    float L5_ = 0.f;

    Matrix<2, 2> J;
    Matrix<2, 2> J_T_;
    Matrix<2, 2> R;
    Matrix<2, 2> M;

    Matrix<2, 2> J_T_inv_; // Inverse of J_T
    Matrix<2, 2> R_inv_;   // Inverse of R
    Matrix<2, 2> M_inv_;   // Inverse of M

    void updateJ(float _phi1, float _phi2, float _phi3, float _phi4)
    {
        float sinf12 = sinf(_phi1 - _phi2);
        float sinf34 = sinf(_phi3 - _phi4);
        float sinf23 = sinf(_phi2 - _phi3);

        J[0][0] = +L1_ * sinf(_phi3) * sinf12 / sinf23;
        J[0][1] = +L4_ * sinf(_phi2) * sinf34 / sinf23;
        J[1][0] = -L1_ * cosf(_phi3) * sinf12 / sinf23;
        J[1][1] = -L4_ * cosf(_phi2) * sinf34 / sinf23;
    }
    void updateR(float _phi1, float _phi2)
    {
        float xC = (L2_ * cosf(_phi2)) + (L1_ * cosf(_phi1)) - (L5_ / 2);
        float yC = (L2_ * sinf(_phi2)) + (L1_ * sinf(_phi1));

        float phiC = atan2f(yC, xC);
        R[0][0] = -sinf(phiC);
        R[0][1] = +cosf(phiC);
        R[1][0] = +cosf(phiC);
        R[1][1] = +sinf(phiC);
    }
    void updateM(float _l0)
    {
        M[0][0] = 1.f / _l0;
        M[0][1] = 0.f;
        M[1][0] = 0.f;
        M[1][1] = 1.f;
    }

    static Matrix<2, 2> inv(Matrix<2, 2> &_m)
    {
        float det = (_m[0][0] * _m[1][1]) - (_m[0][1] * _m[1][0]);
        Matrix<2, 2> res;
        res[0][0] = _m[1][1] / det;
        res[0][1] = -_m[0][1] / det;
        res[1][0] = -_m[1][0] / det;
        res[1][1] = _m[0][0] / det;
        return res;
    }
};

} // namespace CHASSIS