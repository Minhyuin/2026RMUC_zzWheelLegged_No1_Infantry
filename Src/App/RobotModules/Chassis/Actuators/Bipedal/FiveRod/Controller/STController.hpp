#pragma once

#include "Matrix.hpp"

namespace CHASSIS {

class STController {
public:
    union States_u {
        struct {
            float s, q;
            float ds, dq;
        };
        float _[4];
    };

    union Controls_u {
        struct {
            float tw;
        };
        float _[1];
    };

    STController();

    static Matrix<1, 1> solve(const Matrix<4, 1> &_errX, float _l,
                              bool _isFloating);
};

} // namespace CHASSIS
