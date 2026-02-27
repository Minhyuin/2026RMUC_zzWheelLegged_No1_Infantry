#pragma once

#include "Matrix.hpp"

namespace CHASSIS {

class WBRController {
public:
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

    WBRController();

    static Matrix<4, 1> solve(const Matrix<10, 1> &_errX, float _lL, float _lR, bool _isFloatingL, bool _isFloatingR);
};

} // namespace CHASSIS
