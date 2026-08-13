#pragma once

#include "TabulatedFunction.h"
#include "Tabulator.h"
#include "functors.h"
#include <cmath>

namespace capicola {

class Shapers
{
public:
    static constexpr size_t kSize = 1024;

    Shapers() {}

    void Init()
    {
        Tabulator<float, kSize> tab;
        sin_   = tab.tabulate<Mirror::Odd,  Extend::Wrap >([](float x){ return sinf(x); },               0.0f, kPi);
        quake_ = tab.tabulate<Mirror::Odd,  Extend::Clamp>(Quake{},                                      0.0f, 4.0f);
        sinc_  = tab.tabulate<Mirror::Odd,  Extend::Clamp>([](float x){ return SincShaper{}(x, 0.25f); }, 0.0f, 8.0f);
    }

    inline float ReadSin(float x)   const { return sin_(x); }
    inline float ReadQuake(float x) const { return quake_(x); }
    inline float ReadSinc(float x)  const { return sinc_(x); }

private:
    static constexpr float kPi = 3.14159265358979323846f;

    TabulatedFunction<float, kSize, Mirror::Odd,  Extend::Wrap>  sin_;
    TabulatedFunction<float, kSize, Mirror::Odd,  Extend::Clamp> quake_;
    TabulatedFunction<float, kSize, Mirror::Odd,  Extend::Clamp> sinc_;
};

} // namespace capicola
