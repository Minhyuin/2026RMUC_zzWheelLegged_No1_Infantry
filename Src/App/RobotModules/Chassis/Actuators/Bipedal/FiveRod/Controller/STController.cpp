#include "STController.hpp"

#include <cmath>

using namespace CHASSIS;

STController::STController() = default;

Matrix<1, 1> STController::solve(const Matrix<4, 1> &_errX, float _l, bool _isFloating)
{
    const float &x = _l;
    float pow2f = x * x;
    float pow3f = pow2f * x;
    float arrayK[1 * 4] = {
        (121.435796f * powf(x, 2)) - (37.38054996f * x) - (128.1384775f * powf(x, 3)) - (13.86213762f),
        (497.2703763f * x) - (1294.546407f * powf(x, 2)) + (1361.131721f * powf(x, 3)) + (306.6259867f),
        (248.5920119f * powf(x, 2)) - (81.35609083f * x) - (261.7798166f * powf(x, 3)) - (28.33483983f),
        (91.47644933f * x) - (89.03366037f * powf(x, 2)) + (101.159991f * powf(x, 3)) + (11.3614279f)
    };

    Matrix<1, 4> k(arrayK);
    Matrix<1, 1> u;
    // if (!_isFloating)
    //     u = k * _errX;
    // else
    //     u(0, 0) = 0;
    u = k * _errX;
    return u;
}