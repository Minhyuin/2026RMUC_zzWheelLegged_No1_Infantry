#include "GasSpring.hpp"

using namespace CHASSIS;

GasSpring ::GasSpring(float _k, float _f) : k_(_k), f_(_f) {}

float GasSpring::computeForce(float _vel) const { return k_ - (f_ * _vel); }