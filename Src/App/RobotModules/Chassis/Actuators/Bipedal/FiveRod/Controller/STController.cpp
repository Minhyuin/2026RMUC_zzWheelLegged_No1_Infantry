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
    float LQR_K_DATA[1 * 4] = { 83.778534f * x1x1x1 - 21.220596f * x1x1 - 31.251938f * x1 - 76.20426f,
                                68.233096f * x1 + 336.69113f * x1x1 - 590.76774f * x1x1x1 + 305.27392f,
                                111.60892f * x1x1x1 - 50.600176f * x1x1 - 23.421736f * x1 - 63.892864f,
                                149.24717f * x1x1 - 1.6880261f * x1 - 206.49895f * x1x1x1 + 26.661677f };
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