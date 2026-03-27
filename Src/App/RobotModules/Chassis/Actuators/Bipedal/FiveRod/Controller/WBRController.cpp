#include "WBRController.hpp"

#include <cmath>

using namespace CHASSIS;

WBRController::WBRController() = default;

Matrix<4, 1> WBRController::solve(const Matrix<10, 1> &_err, float _lL, float _lR, bool _isFloatingL, bool _isFloatingR)
{
    const float &x1 = _lL;
    const float &x2 = _lR;
    const float x1x1 = x1 * x1;
    const float x2x2 = x2 * x2;
    const float x1x1x1 = x1x1 * x1;
    const float x2x2x2 = x2x2 * x2;
    const float x1x2 = x1 * x2;
    const float x1x2x2 = x1x2 * x2;
    const float x1x1x2 = x1x1 * x2;
    // NOLINTBEGIN(readability-math-missing-parentheses, readability-identifier-naming)
    /*
            obj.Q = diag([ ...
                3000, 1000, ...       % 轮位置
                13000, 3000, ...       % 底盘偏航角
                1000, 100, ...       % 左腿倾角
                1000, 100, ...       % 右腿倾角
                30000, 1e-6]);           % 底盘俯仰角

            obj.R = diag([ ...
                1e2, ...    % 左轮能量惩罚
                1e2, ...    % 右轮能量惩罚
                2e1, ...    % 左腿能量惩罚
                2e1]);      % 右腿能量惩罚
    */
    float LQR_K_DATA[4 * 10] = {
    -3.0027077f,     -3.2887208f,     -7.0138275f,     -3.4480602f,     -9.7587388f,     -1.4368852f,     -6.9644579f,     -0.95121538f,     -3.6970547f,     -0.64710058f,
    -3.0027077f,     -3.2887208f,     7.0138275f,     3.4480602f,     -6.9644579f,     -0.95121538f,     -9.7587388f,     -1.4368852f,     -3.6970547f,     -0.64710058f,
    6.8722156f,     6.6563234f,     -10.98987f,     -5.9994497f,     26.238698f,     3.7505607f,     -9.5332077f,     -0.45606155f,     -30.64492f,     -1.9994378f,
    6.8722156f,     6.6563234f,     10.98987f,     5.9994497f,     -9.5332077f,     -0.45606155f,     26.238698f,     3.7505607f,     -30.64492f,     -1.9994378f 
};
    // NOLINTEND(readability-math-missing-parentheses, readability-identifier-naming)
    Matrix<4, 10> k(LQR_K_DATA);
    Matrix<4, 1> u = k * _err;

    if (_isFloatingL) {
        u(0, 0) = 0;
        u(2, 0) = _err(4, 0) * k(2, 4) + _err(5, 0) * k(2, 5) + _err(6, 0) * k(2, 6) + _err(7, 0) * k(2, 7);
    }
    if (_isFloatingR) {
        u(1, 0) = 0;
        u(3, 0) = _err(4, 0) * k(3, 4) + _err(5, 0) * k(3, 5) + _err(6, 0) * k(3, 6) + _err(7, 0) * k(3, 7);
    }

    return u;
}
