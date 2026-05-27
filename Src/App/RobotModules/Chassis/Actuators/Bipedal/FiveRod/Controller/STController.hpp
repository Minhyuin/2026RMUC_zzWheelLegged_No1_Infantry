#pragma once

#include "Matrix.hpp"

namespace CHASSIS {

class STController {
public:
    enum class STControlMode_e : uint8_t { BALANCE = 0u };
    struct States_s {
        float data[4];
        float &s = data[0];
        float &q = data[1];
        float &ds = data[2];
        float &dq = data[3];
    };

    struct Controls_s {
        float data[1];
        float &tw = data[0];
    };

    STController();

    template <STControlMode_e M> static auto gain(float _l)
    {
        auto k = Matrix<1, 4>::zeros();
        if constexpr (M == STControlMode_e::BALANCE) {
            k = kComputeBalance(_l);
        }
        return k;
    }

private:
    static Matrix<1, 4> kComputeBalance(float _l);
};

} // namespace CHASSIS
