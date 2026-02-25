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
    float LQR_K_DATA[1 * 4] = { 26.49948f * x1x1x1 - 6.6872392f * x1x1 - 9.9054291f * x1 - 24.145107f,
                                44.050147f * x1 + 211.04259f * x1x1 - 372.75205f * x1x1x1 + 223.71706f,
                                54.104345f * x1x1x1 - 21.582604f * x1x1 - 13.754285f * x1 - 35.921195f,
                                2.0270105f * x1 + 90.698775f * x1x1 - 130.72475f * x1x1x1 + 23.421487f };
    // NOLINTEND(readability-math-missing-parentheses, readability-identifier-naming)
    Matrix<1, 4> k(LQR_K_DATA);
    Matrix<1, 1> u;
    // if (!_isFloating)
    //     u = k * _errX;
    // else
    //     u(0, 0) = 0;
    u = k * _errX;
    return u;
}