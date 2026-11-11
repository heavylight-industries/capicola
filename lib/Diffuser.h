#pragma once

#include <cmath>
#include <DeluxeLine.h>

namespace capicola {

// The feedback diffuser: four all-pass stages in series, each with a second all-pass nested inside its loop
// (Gardner's nesting: the outer stage's delay is a delay line followed by an inner all-pass, so the whole stays an
// all-pass), on DeluxeLines. Taps off the outer loops are summed in beside the chain's output.
//
// One knob runs it (Diffusion, 0..1): at 0 the input passes untouched; turning it up fades the network in, grows
// the loops from short to a long plate, and raises the stage gains and the taps' level, up to a dense, reverby wash
// at 1. The loops hold still: nothing wobbles them.
//
// The taps follow the grain: the engine gives the grain's length in samples (SetGrain: Grain Size keyframes at the
// latest keyframe spacing), so they move with both the material and the Grain Size knob. A grain longer than the
// lines reach is folded down by octaves (halved until it fits): the taps keep the grain's rhythm. Spread (0..1) sets
// where they sit: at 0 all on one grain length back (one strong repeat), at 1 spread evenly across it, 1/8 .. 8/8
// of it (a flutter of repeats inside the grain).
//
// Cost (the unit's boot benchmark, feedback 0.9, Diffusion 1): the per-sample work is the stages (integer reads)
// and the taps (linear reads); everything that steers them runs once a block (Block): the grain's smoothing (in
// log, kBaseGlide: the spacing jumps from keyframe to keyframe), the taps' places and their slew, ramped across
// the block. A tap moves at most kTapSlew samples a sample, so a retune
// bends its pitch briefly instead of clicking or tearing. Trivially constructible on purpose (a section of its own):
// call Init().
class Diffuser {
  public:
    static constexpr int kStages = 4;
    static constexpr int kTaps = 8;           // two off each stage's outer loop
    // Sized for internal SRAM (D2, both channels ~164 KB): in SDRAM the four stages alone cost 28 % of a block.
    static constexpr int kOuterSize = 4096;   // power of 2 over the longest loop and the furthest tap (~85 ms)
    static constexpr int kInnerSize = 1024;

    // Each line padded out to 256 bytes past a power of 2: a DeluxeLine keeps its write position after its buffer,
    // every access reads it, and with the lines a power of 2 apart all 16 of them (8 a channel) sat in one D-cache
    // set (4 ways), each access a trip out (the unit: four stages 63 % of a block, padded 28 %, both in SDRAM).
    template <int N>
    struct Slot {
        DeluxeLine<float, N> l;
        char pad[256 - sizeof(int)];
    };
    Slot<kOuterSize> outer[kStages];
    Slot<kInnerSize> inner[kStages];

    void Init() {
        for (int s = 0; s < kStages; s++) {
            outer[s].l.Init();
            inner[s].l.Init();
        }
        amtT = amt = 0.0f;
        size = 0.0f;
        spreadT = spread = 0.0f;
        logBaseT = logBase = std::log(kBaseInit);
        base = kBaseInit;
        for (int j = 0; j < kTaps; j++) { tapD[j] = kBaseInit; tapStep[j] = 0.0f; }
        idle = true;
    }

    void SetAmount(float a) { amtT = Clamp01(a); }
    void SetSpread(float s) { spreadT = Clamp01(s); }
    // The grain's length in samples (KeyframeRecorder::GrainLength); 0 = unknown, the last kept.
    void SetGrain(float samples) {
        if (samples <= 0.0f) return;
        logBaseT = std::log(samples < kTapMin ? kTapMin : samples);
    }
    float Amount()  const { return amt; }
    float TapBase() const { return base; }   // the taps' grain length, folded to fit
    bool  Idle()   const { return idle; }

    // Once a block, before its n samples: the grain, Spread and the taps' places, each tap's move ramped over the
    // block (at most kTapSlew a sample).
    void Block(int n) {
        const float fn = (float)n;
        logBase += (logBaseT - logBase) * (1.0f - std::pow(1.0f - kBaseGlide, fn));
        base = std::exp(logBase);
        while (base > kTapMax) base *= 0.5f;   // (an octave down, the grain's rhythm kept)
        spread += Slew(spreadT - spread, kSlew * fn);
        for (int j = 0; j < kTaps; j++) {
            const float r = 1.0f - spread * (1.0f - (float)(j + 1) / (float)kTaps);
            float target = base * r;
            if (target < kTapMin) target = kTapMin;
            tapStep[j] = Slew(target - tapD[j], kTapSlew * fn) / fn;
        }
    }

    // One sample: x in, x blended with the diffused signal out.
    inline float Process(float x) {
        amt += (amtT - amt) * kGlide;
        size += Slew(amtT - size, kSlew);
        for (int j = 0; j < kTaps; j++) tapD[j] += tapStep[j];

        // Bypassed: nothing runs. The loops keep what they held and resume from it under a blend rising from 0.
        if (amtT == 0.0f && amt < kIdleBelow) { idle = true; return x; }
        idle = false;

        const float g    = kGMin  + (kGMax  - kGMin)  * size;
        const float gi   = kGiMin + (kGiMax - kGiMin) * size;
        const float wet  = (amt < 0.5f) ? 2.0f * amt : 1.0f;
        const float taps = kTapGain * amt;

        float y = x;
        float tapSum = 0.0f;
        for (int s = 0; s < kStages; s++) {
            // The outer loop's delay: its line, then the inner all-pass. Integer reads: the loops step a whole
            // sample at a time while Diffusion slews.
            const float d  = outer[s].l.Read(kOuterMin[s] + (kOuterMax[s] - kOuterMin[s]) * size);
            const float id = inner[s].l.Read(kInnerMin[s] + (kInnerMax[s] - kInnerMin[s]) * size);
            const float u  = d + gi * id;
            const float w  = id - gi * u;
            inner[s].l.Write(Flush(u));

            const float v = y + g * w;
            y = w - g * v;
            outer[s].l.Write(Flush(v));

            tapSum += outer[s].l.ReadLinear(tapD[2 * s]) + outer[s].l.ReadLinear(tapD[2 * s + 1]);
        }

        // The chain is an all-pass (power gain 1); the taps aren't, and in the feedback loop their extra power kept
        // the tail going at full feedback (host: +2 dB a trip). The wet side is scaled back to power 1: the taps
        // read v, whose power is 1 / (1 - g^2) of the input's; kTapNorm x 8 taps = 1 even when they line up.
        const float tp = taps * taps / (1.0f - g * g);
        const float k  = 1.0f / std::sqrt(1.0f + tp);
        return x + wet * (k * (y + taps * kTapNorm * tapSum) - x);
    }

  private:
    // Outer loops (samples): short at size 0 .. the long plate at 1; mutually prime-ish.
    static constexpr float kOuterMin[kStages] = {173.0f, 269.0f, 379.0f, 521.0f};
    static constexpr float kOuterMax[kStages] = {1327.0f, 2069.0f, 2909.0f, 4027.0f};
    // Inner loops, about a third of their outer ones.
    static constexpr float kInnerMin[kStages] = {59.0f, 89.0f, 127.0f, 173.0f};
    static constexpr float kInnerMax[kStages] = {421.0f, 613.0f, 829.0f, 1013.0f};

    static constexpr float kGMin = 0.5f,  kGMax = 0.75f;    // outer gains
    static constexpr float kGiMin = 0.3f, kGiMax = 0.65f;   // inner gains
    static constexpr float kTapGain = 1.0f;
    static constexpr float kTapNorm = 0.125f;               // 1/8 taps

    static constexpr float kTapMin      = 8.0f;     // samples
    static constexpr float kTapMax      = 4000.0f;  // (inside the outer line, with the linear read's second sample)
    static constexpr float kBaseInit    = 2400.0f;  // (50 ms, until the first keyframes)
    static constexpr float kBaseGlide   = 0.00026f; // ~80 ms (a sample's share; Block applies a block's)
    static constexpr float kTapSlew     = 0.2f;     // a tap's read point, samples a sample (a retune's bend <= ~3 st)

    static constexpr float kGlide = 0.0014f;          // the blend's glide, ~15 ms
    static constexpr float kSlew  = 0.1f / 3506.0f;   // 0.1 sample a sample of the longest loop's range (~0.7 s sweep)
    static constexpr float kIdleBelow = 1e-4f;        // (-80 dB: the network stops under it)

    static float Clamp01(float v) { return v < 0.0f ? 0.0f : v > 1.0f ? 1.0f : v; }
    static float Slew(float d, float r) { return d > r ? r : d < -r ? -r : d; }
    static float Flush(float v) { return (v < 1e-20f && v > -1e-20f) ? 0.0f : v; }   // all-pass loops ring forever

    float tapD[kTaps], tapStep[kTaps];
    float amtT, amt, size, spreadT, spread, logBaseT, logBase, base;
    bool  idle;
};

} // namespace capicola
