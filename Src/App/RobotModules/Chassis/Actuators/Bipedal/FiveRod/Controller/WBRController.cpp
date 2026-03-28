#include "WBRController.hpp"

#include <cmath>

#include "StmLog.hpp"

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
                6000, 7000, ...       % 轮位置
                20000, 5000, ...       % 底盘偏航角
                800, 80, ...       % 左腿倾角
                800, 80, ...       % 右腿倾角
                35000, 60]);           % 底盘俯仰角

            obj.R = diag([ ...
                350, ...    % 左轮能量惩罚
                350, ...    % 右轮能量惩罚
                50, ...    % 左腿能量惩罚
                50]);      % 右腿能量惩罚
    */
float LQR_K_DATA[4 * 10] = {
    -1.6754091f,     -2.1257854f,     -2.4841219f,     -1.1959498f,     -8.7002299f,     -1.0618055f,     -3.1883275f,     -0.49867702f,     -5.8818418f,     -1.7522596f,
    -1.6754091f,     -2.1257854f,     2.4841219f,     1.1959498f,     -3.1883275f,     -0.49867702f,     -8.7002299f,     -1.0618055f,     -5.8818418f,     -1.7522596f,
    6.4595232f,     7.1785324f,     -13.38932f,     -8.5640929f,     24.062312f,     2.86892f,     -5.0647373f,     0.35064455f,     -17.179802f,     -2.5240569f,
    6.4595232f,     7.1785324f,     13.38932f,     8.5640929f,     -5.0647373f,     0.35064455f,     24.062312f,     2.86892f,     -17.179802f,     -2.5240569f 
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
