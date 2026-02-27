#pragma once

#include "Matrix.hpp"

namespace CHASSIS {

class WLController {
public:
    struct States_s {
        float data[6];
        float &phi = data[0];
        float &dphi = data[1];
        float &s = data[2];
        float &ds = data[3];
        float &q = data[4];
        float &dq = data[5];
    };

    struct Controls_s {
        float data[2];
        float &tw = data[0];
        float &tp = data[1];
    };

    WLController();

    static Matrix<2, 1> solve(const Matrix<6, 1> &_errX, float _l, bool _isFloating);
};

} // namespace CHASSIS
