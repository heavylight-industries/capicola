#pragma once

#include <cmath>
#include <DeluxeLine.h>

namespace capicola {

// The adaptive comb in the feedback path: the feedback goes into one DeluxeLine and comes back out of eight taps L
// samples back (or spread inside L), so with Feedback up it repeats every L: a comb. L comes from the keyframes:
//
//   Size (0..1)  one knob for all of it (heavylight, 2026-11-02). 0 = off: the feedback passes straight through,
//                exactly as without the comb. Just above 0 the comb fades in (kOnBand of the knob); from there,
//                together:
//                - L sweeps, in log, from one keyframe period (twice the latest keyframe spacing: a pitched comb on
//                  the material's tone) to the grain's length (Grain Size keyframes at that spacing: repeats at the
//                  grain's rate);
//                - the taps spread from all eight on L (one clean comb) to golden-ratio fractions of L (kFrac):
//                  irregular, so they can't line up into a shorter comb (evenly spaced, at L/8 .. L, they did: the
//                  ring went up in pitch instead of spreading), a cloud of repeats around L.
//                The taps' sum is scaled from 1/8 (together, in phase) to 1/sqrt(8) (spread, not): a plain 1/8 lost
//                ~9 dB as they spread.
//
// An L past the line's reach is folded down by octaves (halved until it fits), keeping its rhythm. L is smoothed in
// log (kBaseGlide: the spacing jumps from keyframe to keyframe); the taps' places are worked out once a block (Block)
// and each tap ramps there across the block at most kTapSlew samples a sample, so a retune bends briefly instead of
// clicking. Trivially constructible on purpose (a section of its own): call Init().
class AdaptiveComb {
  public:
    static constexpr int kTaps = 8;
    static constexpr int kSize = 8192;   // power of 2: L up to ~170 ms

    // Padded out to 256 bytes past a power of 2: a DeluxeLine keeps its write position after its buffer, and two of
    // them a power of 2 apart share a D-cache set (the diffuser's lesson).
    DeluxeLine<float, kSize> line;
    char pad[256 - sizeof(int)];

    void Init() {
        line.Init();
        sizeT = size = 0.0f;
        amt = 0.0f;
        norm = 1.0f / kTaps;
        logPeriodT = logGrainT = logBase = std::log(kBaseInit);
        base = kBaseInit;
        for (int j = 0; j < kTaps; j++) { tapD[j] = kBaseInit; tapStep[j] = 0.0f; }
    }

    void SetSize(float s)   { sizeT = Clamp01(s); }
    // Once a block, from the recorder: the latest keyframe spacing and the grain's length, samples (0 = unknown,
    // the last kept).
    void SetKeyframes(float spacing, float grain) {
        if (spacing > 0.0f) logPeriodT = std::log(Clamp(2.0f * spacing, kTapMin, 1e7f));
        if (grain > 0.0f)   logGrainT  = std::log(Clamp(grain, kTapMin, 1e7f));
    }
    float Amount() const { return amt; }
    float TapBase() const { return base; }   // L, folded to fit

    // Once a block, before its n samples.
    void Block(int n) {
        const float fn = (float)n;
        size += Slew(sizeT - size, kKnobSlew * fn);
        const float s = (size - kOnBand) / (1.0f - kOnBand);
        const float sw = s < 0.0f ? 0.0f : s;
        const float spread = sw;   // (one knob: the taps spread as L grows)
        const float logT = logPeriodT + (logGrainT - logPeriodT) * sw;
        logBase += (logT - logBase) * (1.0f - std::pow(1.0f - kBaseGlide, fn));
        base = std::exp(logBase);
        while (base > kTapMax) base *= 0.5f;   // (an octave down, the rhythm kept)
        norm = std::pow((float)kTaps, -(1.0f - 0.5f * spread));
        for (int j = 0; j < kTaps; j++) {
            const float r = 1.0f - spread * (1.0f - kFrac[j]);
            float target = base * r;
            if (target < kTapMin) target = kTapMin;
            tapStep[j] = Slew(target - tapD[j], kTapSlew * fn) / fn;
        }
    }

    // One sample: the feedback in, the feedback out (x itself while off, the taps' sum once on).
    inline float Process(float x) {
        line.Write(x);   // (always: the comb comes on with its history)
        const float on = (sizeT > 0.0f) ? (sizeT < kOnBand ? sizeT / kOnBand : 1.0f) : 0.0f;
        amt += (on - amt) * kGlide;
        for (int j = 0; j < kTaps; j++) tapD[j] += tapStep[j];
        if (on == 0.0f && amt < kIdleBelow) return x;

        float sum = 0.0f;
        for (int j = 0; j < kTaps; j++) sum += line.ReadLinear(tapD[j]);
        return x + amt * (sum * norm - x);
    }

  private:
    static constexpr float kOnBand    = 0.02f;    // the knob's first 2 %: the comb fades in
    // Where the taps spread to, as shares of L: one on L, the others the fractional parts of j x the golden ratio
    // (j = 1..7), irregular.
    static constexpr float kFrac[kTaps] = {1.0f, 0.618f, 0.236f, 0.854f, 0.472f, 0.090f, 0.708f, 0.326f};
    static constexpr float kTapMin    = 8.0f;     // samples
    static constexpr float kTapMax    = 8000.0f;  // (inside the line, with the linear read's second sample)
    static constexpr float kBaseInit  = 2400.0f;  // (50 ms, until the first keyframes)
    static constexpr float kBaseGlide = 0.00026f; // ~80 ms (a sample's share; Block applies a block's)
    static constexpr float kTapSlew   = 0.2f;     // a tap's read point, samples a sample (a retune's bend <= ~3 st)
    static constexpr float kKnobSlew  = 1.0f / 24000.0f;   // the knobs' glide: end to end in 0.5 s
    static constexpr float kGlide     = 0.0014f;  // on / off, ~15 ms
    static constexpr float kIdleBelow = 1e-4f;    // (-80 dB: the taps stop being read under it)

    static float Clamp01(float v) { return v < 0.0f ? 0.0f : v > 1.0f ? 1.0f : v; }
    static float Clamp(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }
    static float Slew(float d, float r) { return d > r ? r : d < -r ? -r : d; }

    float tapD[kTaps], tapStep[kTaps];
    float sizeT, size, amt, norm, logPeriodT, logGrainT, logBase, base;
};

} // namespace capicola
