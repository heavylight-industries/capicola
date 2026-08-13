#pragma once

#include <array>
#include <cmath>
#include <algorithm>

namespace capicola {

// 12-byte struct size needs the attribute to not waste SDRAM on padding.
struct __attribute__((packed, aligned(4))) Keyframe {
    float value;
    double time;
};
static_assert(sizeof(Keyframe) == 12, "");

// sparse: count of keyframes (monotonic index space); uniform: sample-time.
struct Distance {
    long long sparse;
    double    uniform;
};

struct Position {
    long long sparse;
    double    uniform;
};

inline Distance operator-(Position a, Position b) {
    return { a.sparse - b.sparse, a.uniform - b.uniform };
}

inline float SmoothStep(float p) { return p * p * (3.0f - 2.0f * p); }

// A window brackets a position between two adjacent keyframes. `index` is the
// MONOTONIC keyframe index of p1 (the later frame); p2 sits at index-1.
struct Window {
    Keyframe* p1;
    Keyframe* p2;
    long long index;
    float duration;
    float invDuration;
    double endTime;
};

enum class InterpMode {
    LINEAR,
    CATMULLROM,
};

template <int bufsz = 4>
class SparseLine {
private:
    std::array<Keyframe, bufsz> buffer;
    long long writePos;   // MONOTONIC: total keyframes ever written. Slot = & (bufsz-1).

    static constexpr bool IsPowerOfTwo(size_t x) { return x && ((x & (x - 1)) == 0); }

    inline float SolveHermiteFast(const Window& w, float t) const {
        float t2 = t * t;
        float h0 = (2.0f * t - 3.0f) * t2 + 1.0f;
        float h1 = (-2.0f * t + 3.0f) * t2;
        return h0 * w.p1->value + h1 * w.p2->value;
    }

    // Oldest keyframe index still live in the ring.
    inline long long Oldest() const { return (writePos > (long long)bufsz) ? (writePos - bufsz) : 0; }

public:
    static_assert(bufsz >= 4 && IsPowerOfTwo(bufsz), "bufsz must be at least 4 and a power of 2");
    // Implicit trivial constructor on purpose (SDRAM-resident; see Init()).

    // Fetch a keyframe by MONOTONIC index, clamped to [Oldest, newest].
    inline Keyframe* GetSample(long long index) {
        long long newest = writePos - 1;
        if (newest < 0) return &buffer[0];               // empty buffer
        long long oldest = Oldest();
        if      (index < oldest) index = oldest;
        else if (index > newest) index = newest;
        return &buffer[index & (bufsz - 1)];
    }

    inline void Init() {
        Clear();
    }

    inline void Clear() {
        writePos = 0;
        buffer.fill(Keyframe{0.0f, 1.0f});
    }

    void Write(const Keyframe sample) {
        buffer[writePos & (bufsz - 1)] = sample;
        writePos++;
    }

    Keyframe* GetLatest() {
        return GetSample(writePos - 1);
    }

    inline float GetWindowDuration(long long index) {
        return (float)(GetSample(index)->time - GetSample(index - 1)->time);
    }

    inline long long GetLatestIndex() { return writePos - 1; }
    inline long long GetOldestIndex() const { return Oldest(); }

    inline Window WindowAtLatest() {
        long long idx = writePos - 1;
        Keyframe* p1 = GetSample(idx);
        Keyframe* p2 = GetSample(idx - 1);
        float duration = (float)std::abs(p1->time - p2->time);
        float invDuration = (duration > 1e-5f) ? (1.0f / duration) : 0.0f;
        return { p1, p2, idx, duration, invDuration, p1->time };
    }

    inline Position GetWritePosition() {
        return { GetLatestIndex(), GetLatest()->time };
    }

    inline Position GetMinReadPosition(int blockSize) {
        double uniform = GetLatest()->time - (double)blockSize;
        Window w = WindowAtLatest();
        StepWindow(w, uniform);
        return { w.index - 1, uniform };
    }

    // Binary search for the window bracketing uniform time `index` — O(log bufsz).
    // For random access; sequential motion should use StepWindow.
    inline Window WindowAt(double index) {
        long long newest = writePos - 1;
        if (newest < 1) return WindowAtLatest();
        long long lo = Oldest() + 1, hi = newest;   // candidate p1 indices
        while (lo < hi) {
            long long mid = lo + ((hi - lo) >> 1);
            if (GetSample(mid)->time < index) lo = mid + 1; else hi = mid;
        }
        Keyframe* p1 = GetSample(lo);
        Keyframe* p2 = GetSample(lo - 1);
        float duration = (float)(p1->time - p2->time);
        float invDuration = (duration > 1e-5f) ? (1.0f / duration) : 0.0f;
        return { p1, p2, lo, duration, invDuration, p1->time };
    }

    // Walk a known-good window (either direction, wrap-correct) until it brackets
    // `index`. Only one loop runs. Anchor from a nearby window; use WindowAt for
    // far jumps.
    inline void StepWindow(Window& w, double index) {
        long long newest = writePos - 1;
        long long oldest = Oldest();
        while (index > w.endTime && w.index < newest) {          // cursor ahead: step forward
            w.index++;
            w.p2 = w.p1;
            w.p1 = &buffer[w.index & (bufsz - 1)];
            float duration = (float)(w.p1->time - w.p2->time);
            w.duration = duration;
            w.invDuration = (duration > 1e-5f) ? (1.0f / duration) : 0.0f;
            w.endTime = w.p1->time;
        }
        while (index < w.p2->time && w.index > oldest + 1) {     // cursor behind: step backward
            w.index--;
            w.p1 = w.p2;
            w.p2 = &buffer[(w.index - 1) & (bufsz - 1)];
            float duration = (float)(w.p1->time - w.p2->time);
            w.duration = duration;
            w.invDuration = (duration > 1e-5f) ? (1.0f / duration) : 0.0f;
            w.endTime = w.p1->time;
        }
    }

    inline float Read(const Window& w, float t, InterpMode mode) {
        if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;
        switch (mode) {
            case InterpMode::LINEAR:
                return w.p1->value + t * (w.p2->value - w.p1->value);
            case InterpMode::CATMULLROM:
                return SolveHermiteFast(w, t);
            default: break;
        }
        return 0.0f;
    }

    // Shaped read: the waveshaper hits the two KEYFRAMES, and the interpolation
    // runs over the shaped values. The nonlinearity never sees the sample rate,
    // so it cannot fold new partials back into the band.
    template <typename ShapeFn>
    inline float Read(const Window& w, float t, InterpMode mode, ShapeFn fn) {
        if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;
        const float v1 = fn(w.p1->value);
        const float v2 = fn(w.p2->value);
        switch (mode) {
            case InterpMode::LINEAR:
                return v1 + t * (v2 - v1);
            case InterpMode::CATMULLROM: {
                const float t2 = t * t;
                const float h0 = (2.0f * t - 3.0f) * t2 + 1.0f;
                const float h1 = (-2.0f * t + 3.0f) * t2;
                return h0 * v1 + h1 * v2;
            }
            default: break;
        }
        return 0.0f;
    }
};

} // namespace capicola
