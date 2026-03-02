#pragma once

namespace CHASSIS {

class GasSpring {
public:
    GasSpring(float _k, float _f);

    float computeForce(float _vel) const;

private:
    float k_; // static thrust
    float f_; // damping coefficient
};

} // namespace CHASSIS