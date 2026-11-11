/**
 * @file jack_routing.h
 * @brief Which signal drives which CV jack — the pure decision, no hardware.
 *
 * Four jacks: Trig 1, Env 1, Trig 2, Env 2. Out of the box (heavylight, 1.3) all four are outputs: Trig 1 the one
 * trigger (every slice), Env 1 the input envelope (TKEO), Trig 2 the contour (CONT) and Env 2 the pulses (PULS), so
 * the three modulators are always there as CVs. Trig 2 and Env 2 each carry whichever signal the user picks, in
 * either mode. The Options can flip Trig 1 / Env 1 into CV inputs (CV 2 / CV 3), and can add the output follower's
 * signals to the mod sources (ModRouter). The 6pm pips never follow any of this — they always show the followers.
 *
 * A corrupt preset byte falls back to these defaults: outputs, which never fight a patched cable.
 *
 * main.cpp applies the Plan to the hardware; tests/host checks every
 * combination of it, so the logic that ships is the logic that is tested.
 */

#pragma once

#include <cstdint>

namespace jacks {

/* What a jack can carry. Byte values are stored in presets — append only. */
enum Signal : uint8_t
{
    kInTrig  = 0,   // a pulse on every slice the engine takes
    kOutTrig = 1,   // a pulse on each kept output-follower transient
    kInEnv   = 2,   // input follower envelope
    kOutEnv  = 3,   // output follower envelope
    kInCont  = 4,   // the input side's accent contour (CONT: a smoothstep from hit to hit, lib/Contour.h)
    kOutCont = 5,   // the output side's
    kInPuls  = 6,   // the input side's pulses (PULS: a Gaussian per hit, lib/Pulse.h)
    kOutPuls = 7,   // the output side's
    kNumSignals
};

/* Jack order, also the order of Plan's arrays. */
enum Jack : uint8_t { kTrig1 = 0, kEnv1 = 1, kTrig2 = 2, kEnv2 = 3, kNumJacks };

constexpr float kTrigVolts = 5.0f;   // a high trigger
constexpr float kEnvVolts  = 5.0f;   // envelope full scale (0..1 → 0..5 V)

struct Options
{
    bool    cvIn        = false;     // Trig 1 / Env 1 as CV inputs
    bool    outFollower = false;     // the output follower's signals among the mod sources
    uint8_t trig2       = kInCont;   // what Trig 2 carries
    uint8_t env2        = kInPuls;   // what Env 2 carries
};

/* The live follower signals, sampled at the control rate. */
struct Signals
{
    bool  inTrig  = false;   // slice pulse currently high
    bool  outTrig = false;   // output-follower pulse currently high
    float inEnv   = 0.0f;    // 0..1
    float outEnv  = 0.0f;    // 0..1
    float inCont  = 0.0f;    // 0..1
    float outCont = 0.0f;    // 0..1
    float inPuls  = 0.0f;    // 0..1
    float outPuls = 0.0f;    // 0..1
};

/* Per jack: driven as an output, and if so at what voltage. */
struct Plan
{
    bool  output[kNumJacks];
    float volts [kNumJacks];
};

inline float Volts(uint8_t sig, const Signals& s)
{
    auto env = [](float e) { return (e < 0.0f ? 0.0f : e > 1.0f ? 1.0f : e) * kEnvVolts; };
    switch (sig)
    {
        case kInTrig:  return s.inTrig  ? kTrigVolts : 0.0f;
        case kOutTrig: return s.outTrig ? kTrigVolts : 0.0f;
        case kInEnv:   return env(s.inEnv);
        case kOutEnv:  return env(s.outEnv);
        case kInCont:  return env(s.inCont);    // (a contour is a CV like an envelope: 0..5 V)
        case kOutCont: return env(s.outCont);
        case kInPuls:  return env(s.inPuls);
        case kOutPuls: return env(s.outPuls);
        default:       return 0.0f;
    }
}

inline Plan Route(const Options& o, const Signals& s)
{
    Plan p{};
    p.output[kTrig1] = !o.cvIn;
    p.output[kEnv1]  = !o.cvIn;
    p.output[kTrig2] = true;
    p.output[kEnv2]  = true;
    p.volts[kTrig1]  = o.cvIn ? 0.0f : Volts(kInTrig, s);
    p.volts[kEnv1]   = o.cvIn ? 0.0f : Volts(kInEnv,  s);
    p.volts[kTrig2]  = Volts(o.trig2, s);
    p.volts[kEnv2]   = Volts(o.env2,  s);
    return p;
}

/* Preset bytes: [0] the jacks' mode, [1] trig2, [2] env2. Byte 0 is 0 / 1 (cvIn) as 1.2 wrote it, plus 2 when the
 * output follower is among the mod sources (a bit of a byte that existed: a new byte would change the preset's size
 * and lose every save). Anything out of range — a corrupt or foreign record — falls back to the defaults. */
constexpr uint8_t kBytes = 3;

inline void Encode(const Options& o, uint8_t* out)
{
    out[0] = (uint8_t)((o.cvIn ? 1 : 0) | (o.outFollower ? 2 : 0));
    out[1] = o.trig2;
    out[2] = o.env2;
}

inline Options Decode(const uint8_t* in)
{
    Options o;
    if (in[0] <= 3) { o.cvIn = (in[0] & 1) != 0; o.outFollower = (in[0] & 2) != 0; }
    o.trig2 = (in[1] < kNumSignals) ? in[1] : Options{}.trig2;
    o.env2  = (in[2] < kNumSignals) ? in[2] : Options{}.env2;
    return o;
}

} // namespace jacks
