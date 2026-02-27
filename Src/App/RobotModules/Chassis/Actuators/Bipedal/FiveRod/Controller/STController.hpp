#pragma once

#include "Matrix.hpp"

namespace CHASSIS {

class STController {
public:
    struct States_s {
        float data[4];
        float &s = data[0];
        float &q = data[1];
        float &ds = data[2];
        float &dq = data[3];
    };

    struct Controls_s {
        float data[1];
        float &tw = data[0];
    };

    STController();

    static Matrix<1, 1> solve(const Matrix<4, 1> &_errX, float _l, bool _isFloating);
};

} // namespace CHASSIS
