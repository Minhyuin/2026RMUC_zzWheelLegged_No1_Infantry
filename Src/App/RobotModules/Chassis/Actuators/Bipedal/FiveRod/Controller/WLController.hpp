#pragma once

#include "Matrix.hpp"

namespace CHASSIS {

class WLController {
public:
    union States_u {
        struct {
            float phi, dphi;
            float s, ds;
            float q, dq;
        };
        float _[6];
    };

    union Controls_u {
        struct {
            float tw;
            float tp;
        };
        float _[2];
    };

    WLController();

    static Matrix<2, 1> solve(const Matrix<6, 1> &_errX, float _l,
                              bool _isFloating);
};

} // namespace CHASSIS
