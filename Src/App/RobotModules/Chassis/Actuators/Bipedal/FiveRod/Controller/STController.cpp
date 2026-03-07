#include "STController.hpp"

#include <cmath>

using namespace CHASSIS;

STController::STController() = default;

Matrix<1, 1> STController::solve(const Matrix<4, 1> &_errX, float _l, bool _isFloating)
{
    const float &x1 = _l;
    float x1x1 = x1 * x1;
    float x1x1x1 = x1x1 * x1;
    // NOLINTBEGIN(readability-math-missing-parentheses, readability-identifier-naming)
    float LQR_K_DATA[1 * 4] = { 83.783298f * x1x1x1 - 21.243629f * x1x1 - 31.23614f * x1 - 76.171167f,
                                70.116009f * x1 + 336.89572f * x1x1 - 594.33134f * x1x1x1 + 325.85292f,
                                111.71215f * x1x1x1 - 50.377685f * x1x1 - 23.662423f * x1 - 64.39917f,
                                150.6202f * x1x1 - 1.7455609f * x1 - 208.34991f * x1x1x1 + 26.753413f };
    // NOLINTEND(readability-math-missing-parentheses, readability-identifier-naming)
    Matrix<1, 4> k(LQR_K_DATA);
    Matrix<1, 1> u;
    (void)_isFloating;
    // if (!_isFloating)
    //     u = k * _errX;
    // else
    //     u(0, 0) = 0;
    u = k * _errX;
    return u;
}