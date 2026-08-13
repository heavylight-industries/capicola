#pragma once

#include <cmath>

namespace capicola {

struct Sin
{
    float operator()(float x) const
    {
        return sinf(x);
    }
};

struct Tanh
{
    float operator()(float x) const
    {
        return tanhf(x);
    }
};

struct Lanczos
{
    float operator()(float x) const
    {
        if (x == 0.0f) return 1.0f;
        return (sinf(M_PI * x) / (M_PI * x)) * (sinf(0.5f * M_PI * x) / (0.5f * M_PI * x));
    }
};

struct SoftSat
{
    float operator()(float x) const
    {
        return x / sqrtf(1.0f + x * x);
    }
};

struct SincShaper
{
    float operator()(float x, float mix) const
    {
        const float k     = Lanczos{}(x);
        const float morph = (1.0f - 2.0f * mix) * k + 0.5f;
        return 2.0f * SoftSat{}(x) * morph;
    }
};

struct Quake
{
    float operator()(float x) const
    {
        const float x2 = x * x;
        const float x4 = x2 * x2;
        return 1.0f - 1.0f / (1.0f + 20.0f * x4);
    }
};

} // namespace capicola
