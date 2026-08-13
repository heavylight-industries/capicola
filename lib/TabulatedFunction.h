#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace capicola {

enum class Mirror : uint8_t {
    None,   // table covers [min, max] as-is
    Odd,    // table covers [0, max], f(-x) = -f(|x|)  (sin, saturators)
    Even,   // table covers [0, max], f(-x) =  f(|x|)  (kernels)
};

enum class Extend : uint8_t {
    Clamp,  // past table edge, return last sample      (saturators)
    Wrap,   // past table edge, fold input back in      (periodic functions)
};

// Mirror and Extend are non-type template parameters so each instance
// dead-code-eliminates the unused branches at compile time.  Per-call
// cost drops to exactly the work the policies require.
template <typename T, size_t n,
          Mirror M = Mirror::None,
          Extend E = Extend::Clamp>
class TabulatedFunction
{
public:
    TabulatedFunction() {}
    TabulatedFunction(std::array<T, n> table, T min, T max)
        : table_(table), min_(min), max_(max)
    {
        scale_     = static_cast<T>(n - 1) / (max - min);
        period_    = (M == Mirror::None) ? (max - min) : (T(2) * max);
        invPeriod_ = T(1) / period_;
    }

    inline T operator()(T x) const
    {
        T input = x;
        T sign  = T(1);

        if constexpr (M != Mirror::None) {
            if (input < T(0)) {
                input = -input;
                if constexpr (M == Mirror::Odd) sign = T(-1);
            }
        }

        if constexpr (E == Extend::Wrap) {
            // input is non-negative here (mirror folded any negatives), so
            // (int) truncation matches floor() for our use case.
            int periods = static_cast<int>(input * invPeriod_);
            input -= static_cast<T>(periods) * period_;
            if constexpr (M != Mirror::None) {
                if (input > max_) {
                    input -= max_;
                    if constexpr (M == Mirror::Odd) sign = -sign;
                }
            }
        }

        T idx;
        if constexpr (M == Mirror::None) {
            idx = (input - min_) * scale_;
        } else {
            // min_ is 0 for mirrored tables — skip the subtract.
            idx = input * scale_;
        }

        if (idx <= T(0))     return sign * table_[0];
        if (idx >= T(n - 1)) return sign * table_[n - 1];

        int i  = static_cast<int>(idx);
        T   mu = idx - static_cast<T>(i);
        T   a  = table_[i];
        T   b  = table_[i + 1];
        return sign * (a + (b - a) * mu);
    }

private:
    std::array<T, n> table_{};
    T                min_       = T(0);
    T                max_       = T(0);
    T                scale_     = T(0);
    T                period_    = T(0);
    T                invPeriod_ = T(0);
};

} // namespace capicola
