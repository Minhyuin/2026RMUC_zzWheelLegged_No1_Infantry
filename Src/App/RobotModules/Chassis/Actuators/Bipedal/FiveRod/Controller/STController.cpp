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
    float arrayK[1 * 4] = {
        (367.7578972f * powf(x, 2)) - (119.0991449f * x) -
                (372.0232056f * powf(x, 3)) - (15.8426773f),
        (776.212958f * x) - (1995.719121f * powf(x, 2)) +
                (2017.854273f * powf(x, 3)) + (235.5366741f),
        (509.2616503f * powf(x, 2)) - (174.1514167f * x) -
                (514.2014853f * powf(x, 3)) - (21.93905547f),
        (94.55311758f * x) - (81.23798886f * powf(x, 2)) +
                (95.91579686f * powf(x, 3)) + (4.453837164f)
    };
    Matrix<1, 4> k(arrayK);
    Matrix<1, 1> u;
    if (_isFloating)
        u = k * _errX;
    else
        u[0][0] = 0;
    return u;
}