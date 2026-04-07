#pragma once

#include "Matrix.hpp"

namespace CHASSIS {

class WBRController {
public:
    enum class WBRControlMode_e : uint8_t { BALANCE = 0u, SPIN, CLIMB, JUMP };
    struct States_s {
        float data[10];
        float &s = data[0];
        float &ds = data[1];
        float &yaw = data[2];
        float &dyaw = data[3];
        float &phil = data[4];
        float &dphil = data[5];
        float &phir = data[6];
        float &dphir = data[7];
        float &q = data[8];
        float &dq = data[9];
    };

    struct Controls_s {
        float data[4];
        float &twl = data[0];
        float &twr = data[1];
        float &tql = data[2];
        float &tqr = data[3];
    };

    WBRController() = default;

    template <WBRControlMode_e M>
    static Matrix<4, 1> solve(const Matrix<10, 1> &_errX, float _lL, float _lR, bool _isFloatingL, bool _isFloatingR)
    {
        Matrix<4, 10> k = Matrix<4, 10>::zeros();
        if constexpr (M == WBRControlMode_e::BALANCE) {
            k = kComputeBalance(_lL, _lR);
        } else if (M == WBRControlMode_e::SPIN) {
            k = kComputeSpin(_lL, _lR);
        } else if (M == WBRControlMode_e::CLIMB) {
            k = kComputeClimb(_lL, _lR);
        } else if (M == WBRControlMode_e::JUMP) {
            k = kComputeJump(_lL, _lR);
        }
        Matrix<4, 1> u = k * _errX;
        if (_isFloatingL) {
            u(0, 0) = 0;
            u(2, 0) = _errX(4, 0) * k(2, 4) + _errX(5, 0) * k(2, 5) + _errX(6, 0) * k(2, 6) + _errX(7, 0) * k(2, 7);
        }
        if (_isFloatingR) {
            u(1, 0) = 0;
            u(3, 0) = _errX(4, 0) * k(3, 4) + _errX(5, 0) * k(3, 5) + _errX(6, 0) * k(3, 6) + _errX(7, 0) * k(3, 7);
        }
        return u;
    }

private:
    static Matrix<4, 10> kComputeBalance(float _lL, float _lR);
    static Matrix<4, 10> kComputeSpin(float _lL, float _lR);
    static Matrix<4, 10> kComputeClimb(float _lL, float _lR);
    static Matrix<4, 10> kComputeJump(float _lL, float _lR);
};

} // namespace CHASSIS
