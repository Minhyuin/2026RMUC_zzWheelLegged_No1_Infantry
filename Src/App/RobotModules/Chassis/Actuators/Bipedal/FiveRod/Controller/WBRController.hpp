#pragma once

#include "Matrix.hpp"

namespace CHASSIS {

class WBRController {
public:
    union States_u {
        struct {
            float s, ds;
            float yaw, dyaw;
            float phil, dphil;
            float phir, dphir;
            float q, dq;
        };
        float _[10];
    };

    union Controls_u {
        struct {
            float twl, twr;
            float tql, tqr;
        };
        float _[4];
    };

    WBRController();

    Matrix<4, 1> solve(const Matrix<10, 1> &_errX, float _lL, float _lR,
                              bool _isFloatingL, bool _isFloatingR);
};

} // namespace CHASSIS
