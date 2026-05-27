#include "STController.hpp"

#include <cmath>

using namespace CHASSIS;

STController::STController() = default;

Matrix<1, 4> STController::kComputeBalance(float _l)
{
    const float &x1 = _l;
    float x1x1 = x1 * x1;
    float x1x1x1 = x1x1 * x1;
    // NOLINTBEGIN(readability-math-missing-parentheses, readability-identifier-naming)
    float LQR_K_DATA[1 * 4] = { 55.741196f * x1x1x1 - 22.563974f * x1x1 - 13.20684f * x1 - 17.804148f,
                                252.34194f * x1 + 793.81345f * x1x1 - 1576.4058f * x1x1x1 + 489.25045f,
                                221.61918f * x1x1x1 - 93.882349f * x1x1 - 49.260291f * x1 - 67.663104f,
                                6.6000593f * x1 + 250.37172f * x1x1 - 364.61797f * x1x1x1 + 40.171802f };
    // NOLINTEND(readability-math-missing-parentheses, readability-identifier-naming)
    Matrix<1, 4> k(LQR_K_DATA);
    return k;
}