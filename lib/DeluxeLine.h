#pragma once

#include <array>
#include <cstddef>
#include <cmath>

namespace capicola {

struct CubicResult {
    float value;
    float d1;
    float d2;
    float tkeo;
};

template <typename T, int bufsz = 4>
class DeluxeLine {
private:
    inline T GetSample(int position) const noexcept {
        return buffer[position & (bufsz - 1)];
    }

    static constexpr bool IsPowerOfTwo(size_t x) {
        return x && ((x & (x - 1)) == 0);
    }

    std::array<T, bufsz> buffer;
    int writePos;

public:
    static_assert(bufsz >= 4 && IsPowerOfTwo(bufsz), "bufsz must be at least 4 and a power of 2");

    // Implicit trivial constructor on purpose (SDRAM-resident; see Init()).

    inline void Init() {
        Clear();
    }

    inline void Clear() {
        writePos = 0;
        buffer.fill(T(0));
    }

    void Write(const T sample) {
        buffer[writePos] = sample;
        writePos = (writePos + 1) & (bufsz - 1);
    }

    //Direct register access
    inline T Read(float delay = T(1.0)) const noexcept {
        const int delay_int = static_cast<int>(delay);
        return GetSample(writePos - delay_int);
    }

    //Linear interpolation
    inline T ReadLinear(float delay = T(1.0)) const noexcept {
        const int delay_int = static_cast<int>(delay);
        const T delay_frac = delay - delay_int;

        const T a = GetSample(writePos - delay_int);
        const T b = GetSample(writePos - delay_int - 1);

        return a + delay_frac * (b - a);
    }

    //Hermite interpolation
    inline T ReadHermite(float delay = T(2.0), int stride = 1) const noexcept {
        const int delay_int_ = static_cast<int>(delay);
        const T delay_frac_ = delay - delay_int_;

        const T x0 = GetSample(writePos - delay_int_ + stride);
        const T x1 = GetSample(writePos - delay_int_);
        const T x2 = GetSample(writePos - delay_int_ - stride);
        const T x3 = GetSample(writePos - delay_int_ - 2 * stride);

        static constexpr T half = T(0.5);
        const T c = (x2 - x0) * half;
        const T v = x1 - x2;
        const T w = c + v;
        const T a = w + v + (x3 - x1) * half;
        const T b_neg = w + a;

        return x1 + delay_frac_ * (c + delay_frac_ * (-b_neg + delay_frac_ * a));
    }

    //Hermite interpolation plus first derivative
    inline CubicResult ReadHermiteD1(float delay = T(2.0), int stride = 1) const noexcept {
        const float inv_stride = 1.0f / static_cast<float>(stride);
        const float delay_scaled_ = delay * static_cast<float>(stride);
        const int delay_int_ = static_cast<int>(delay_scaled_);
        const T delay_frac_ = delay_scaled_ - delay_int_;

        const T x0 = GetSample(writePos - delay_int_ + stride);
        const T x1 = GetSample(writePos - delay_int_);
        const T x2 = GetSample(writePos - delay_int_ - stride);
        const T x3 = GetSample(writePos - delay_int_ - 2 * stride);

        static constexpr T half = T(0.5);
        const T c = (x2 - x0) * half;
        const T v = x1 - x2;
        const T w = c + v;
        const T a = w + v + (x3 - x1) * half;
        const T b_neg = w + a;

        const T t = delay_frac_;

        CubicResult result;
        result.value = x1 + t * (c + t * (-b_neg + t * a));
        result.d1 = (c + t * (-2.0f * b_neg + t * 3.0f * a)) * inv_stride;

        return result;
    }

    //Hermite interpolation with derivatives and teager kaiser energy operator
    inline CubicResult ReadHermiteFull(T delay = T(2.0), int stride = 1) const noexcept {
        const float inv_stride = 1.0f / static_cast<float>(stride);
        const float delay_scaled_ = delay * static_cast<float>(stride);
        const int delay_int_ = static_cast<int>(delay_scaled_);
        const T delay_frac_ = delay_scaled_ - delay_int_;

        const T x0 = GetSample(writePos - delay_int_ + stride);
        const T x1 = GetSample(writePos - delay_int_);
        const T x2 = GetSample(writePos - delay_int_ - stride);
        const T x3 = GetSample(writePos - delay_int_ - 2 * stride);

        static constexpr T half = T(0.5);
        const T c = (x2 - x0) * half;
        const T v = x1 - x2;
        const T w = c + v;
        const T a = w + v + (x3 - x1) * half;
        const T b_neg = w + a;

        const T t = delay_frac_;

        CubicResult result;
        result.value = x1 + t * (c + t * (-b_neg + t * a));
        result.d1 = (c + t * (-2.0f * b_neg + t * 3.0f * a)) * inv_stride;
        result.d2 = (-2.0f * b_neg + 6.0f * a * t) * (inv_stride * inv_stride);
        result.tkeo = std::abs(result.d1 * result.d1 - result.value * result.d2);

        return result;
    }

    // B-spline interpolation, integer delay, value + first derivative only.
    // Discrete-only optimization of the detector front-end: 3 taps, no
    // fractional-delay basis-function math (no t^2/t^3), no stride scaling,
    // no d2/tkeo. The keyframe detector only needs value + sign(d1), so this
    // is the cheap path. Use ReadBSplineFull when you actually want d2/tkeo
    // or a fractional delay.
    inline CubicResult ReadBSplineD1Integer(int delay) const noexcept {
        const T x0 = GetSample(writePos - delay + 1);
        const T x1 = GetSample(writePos - delay);
        const T x2 = GetSample(writePos - delay - 1);
        static constexpr T inv6 = T(1.0/6.0);
        static constexpr T half = T(0.5);

        CubicResult result;
        result.value = (x0 + T(4) * x1 + x2) * inv6;
        result.d1    = (x2 - x0) * half;
        result.d2    = T(0);
        result.tkeo  = T(0);
        return result;
    }

    //B-spline interpolation
    inline T ReadBSpline(T delay = T(2.0)) const noexcept {
        const int delay_int_ = static_cast<int>(delay);
        const T delay_frac_ = delay - delay_int_;

        const T x0 = GetSample(writePos - delay_int_ + 1);
        const T x1 = GetSample(writePos - delay_int_);
        const T x2 = GetSample(writePos - delay_int_ - 1);
        const T x3 = GetSample(writePos - delay_int_ - 2);

        const T t2 = delay_frac_ * delay_frac_;
        const T t3 = t2 * delay_frac_;
        static constexpr T inv6 = T(1.0/6.0);

        const T B0_ = T(1) + delay_frac_ * (T(-3) + delay_frac_ * (T(3) - delay_frac_));
        const T B1_ = T(4) + t2 * (T(-6) + T(3) * delay_frac_);
        const T B2_ = T(1) + delay_frac_ * (T(3) + delay_frac_ * (T(3) - T(3) * delay_frac_));
        const T B3_ = t3;

        return (x0*B0_ + x1*B1_ + x2*B2_ + x3*B3_) * inv6;
    }

    // B-spline interpolation with derivatives and teager kaiser energy operator
    inline CubicResult ReadBSplineFull(T delay = T(2.0), int stride = 1) const noexcept {
        //separate the desired delay value into integer and fractional parts
        const float inv_stride = 1.0f / static_cast<float>(stride);
        const float delay_scaled_ = delay * static_cast<float>(stride);
        const int delay_int_ = static_cast<int>(delay_scaled_);
        const T delay_frac_ = delay_scaled_ - delay_int_;

        const T x0 = GetSample(writePos - delay_int_ + stride);
        const T x1 = GetSample(writePos - delay_int_);
        const T x2 = GetSample(writePos - delay_int_ - stride);
        const T x3 = GetSample(writePos - delay_int_ - (2 * stride));

        // Precompute powers of t once
        const T t2 = delay_frac_ * delay_frac_;
        const T t3 = t2 * delay_frac_;
        static constexpr T inv6 = T(1.0/6.0);

        // Precompute common terms used across all computations
        const T term1 = T(3) * delay_frac_;
        const T term2 = T(6) * delay_frac_;

        // Calculate basis functions with factored expressions to minimize operations
        const T B0_ = T(1) + delay_frac_ * (T(-3) + delay_frac_ * (T(3) - delay_frac_));
        const T B1_ = T(4) + t2 * (T(-6) + term1);
        const T B2_ = T(1) + delay_frac_ * (T(3) + delay_frac_ * (T(3) - term1));
        const T B3_ = t3;

        // First derivatives are more efficiently calculated from the original basis functions
        const T dB0 = T(-3) + delay_frac_ * (T(6) - term1);
        const T dB1 = delay_frac_ * (T(-12) + T(9) * delay_frac_);
        const T dB2 = T(3) + delay_frac_ * (T(6) - T(9) * delay_frac_);
        const T dB3 = term1 * delay_frac_;

        // Second derivatives can be further optimized
        const T d2B0 = T(6) - term2;
        const T d2B1 = T(-12) + T(18) * delay_frac_;
        const T d2B2 = T(6) - T(18) * delay_frac_;
        const T d2B3 = term2;

        // Calculate results
        CubicResult result;
        result.value = (x0*B0_ + x1*B1_ + x2*B2_ + x3*B3_) * inv6;
        result.d1 = (x0*dB0 + x1*dB1 + x2*dB2 + x3*dB3) * inv6 * inv_stride;
        result.d2 = (x0*d2B0 + x1*d2B1 + x2*d2B2 + x3*d2B3) * inv6 * inv_stride * inv_stride;
        result.tkeo = std::abs(result.d1 * result.d1 - result.value * result.d2);

        return result;
    }

    inline CubicResult ReadWaveletFull(float delay = 2.0f, int stride = 1) const noexcept {
        CubicResult wavelet;
        auto resultH = ReadHermiteFull(delay,stride);
        auto resultB = ReadBSplineFull(delay,stride);

        wavelet.value = resultH.value - resultB.value;
        wavelet.d1 = resultH.d1 - resultB.d1;
        wavelet.d2 = resultH.d2 - resultB.d2;
        wavelet.tkeo = resultH.tkeo - resultB.tkeo;

        return wavelet;
    }
};

} // namespace capicola
