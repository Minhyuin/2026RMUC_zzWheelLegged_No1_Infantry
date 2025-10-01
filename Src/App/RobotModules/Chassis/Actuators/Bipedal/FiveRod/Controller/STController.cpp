#include "STController.hpp"

#include <cmath>

using namespace CHASSIS;

STController::STController() = default;

Matrix<1, 1> STController::solve(const Matrix<4, 1> &_errX, float _l,
                                 bool _isFloating)
{
    const float &x = _l;
    float pow2f = x * x;
    float pow3f = pow2f * x;
    float arrayK[1 * 4] = { (82.36945565f * powf(x, 2)) - (26.72296633f * x) -
                                    (83.6698081f * powf(x, 3)) - (10.55188428f),
                            (408.7636007f * x) - (1086.824088f * powf(x, 2)) +
                                    (1089.179834f * powf(x, 3)) +
                                    (298.9067399f),
                            (147.5477642f * powf(x, 2)) - (51.05181056f * x) -
                                    (148.8875148f * powf(x, 3)) - (18.4852547f),
                            (77.41491432f * x) - (77.96371544f * powf(x, 2)) +
                                    (71.12149053f * powf(x, 3)) +
                                    (5.804348264f) };

    Matrix<1, 4> k(arrayK);
    Matrix<1, 1> u;
    if (!_isFloating)
        u = k * _errX;
    else
        u[0][0] = 0;
    return u;
}