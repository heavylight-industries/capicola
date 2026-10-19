/** Independent J3..J8 assignments. This logic is shared by firmware and host
 * tests; function byte values are the programmer's enum indices. */
#pragma once

#include <cmath>
#include <cstdint>

namespace jacks {

enum Function : uint8_t
{
    kPitch = 0, kStretch, kThreshold, kGrainSize, kQuality, kFeedback,
    kSlice, kInputEnvelope, kOutputEnvelope, kSliceTrigger, kOutputTransient,
    kNumFunctions
};

enum Jack : uint8_t { kJ3 = 0, kJ4, kJ5, kJ6, kJ7, kJ8, kNumJacks };
constexpr uint8_t kNumDestinations = 6;
constexpr uint8_t kBytes = kNumJacks;
constexpr uint32_t kSchemaHash = 0x4a41434bu;
constexpr float kTrigVolts = 5.0f;
constexpr float kEnvVolts = 5.0f;

struct Options
{
    uint8_t function[kNumJacks] = {
        kSlice, kPitch, kStretch, kFeedback, kSliceTrigger, kInputEnvelope,
    };
};

inline bool operator==(const Options& a, const Options& b)
{
    for (uint8_t j = 0; j < kNumJacks; ++j)
        if (a.function[j] != b.function[j]) return false;
    return true;
}
inline bool operator!=(const Options& a, const Options& b) { return !(a == b); }

inline uint8_t FunctionAt(const Options& o, uint8_t jack)
{
    return o.function[jack] < kNumFunctions ? o.function[jack] : Options{}.function[jack];
}
inline bool IsCvInput(uint8_t f) { return f < kNumDestinations; }
inline bool IsOutput(uint8_t f) { return f >= kInputEnvelope && f < kNumFunctions; }

inline void Encode(const Options& o, uint8_t* out)
{
    for (uint8_t j = 0; j < kNumJacks; ++j) out[j] = FunctionAt(o, j);
}
inline Options Decode(const uint8_t* in)
{
    Options o;
    for (uint8_t j = 0; j < kNumJacks; ++j)
        if (in[j] < kNumFunctions) o.function[j] = in[j];
    return o;
}

struct Signals
{
    bool inTrig = false;
    bool outTrig = false;
    float inEnv = 0.0f;
    float outEnv = 0.0f;
};
struct Plan
{
    bool output[kNumJacks] = {};
    float volts[kNumJacks] = {};
};

inline float EnvelopeVolts(float e)
{
    if (!std::isfinite(e)) return 0.0f;
    return (e < 0.0f ? 0.0f : e > 1.0f ? 1.0f : e) * kEnvVolts;
}
inline Plan Route(const Options& o, const Signals& s)
{
    Plan p;
    for (uint8_t j = 0; j < kNumJacks; ++j)
    {
        const uint8_t f = FunctionAt(o, j);
        p.output[j] = IsOutput(f);
        switch (f)
        {
            case kInputEnvelope: p.volts[j] = EnvelopeVolts(s.inEnv); break;
            case kOutputEnvelope: p.volts[j] = EnvelopeVolts(s.outEnv); break;
            case kSliceTrigger: p.volts[j] = s.inTrig ? kTrigVolts : 0.0f; break;
            case kOutputTransient: p.volts[j] = s.outTrig ? kTrigVolts : 0.0f; break;
            default: break;
        }
    }
    return p;
}

/** Sum calibrated jack voltages by destination. Five volts spans the full
 * parameter range. Do not clip individual inputs before they can cancel. */
inline void CvInputs(const Options& o, const float* volts, float* destinations)
{
    for (uint8_t p = 0; p < kNumDestinations; ++p) destinations[p] = 0.0f;
    for (uint8_t j = 0; j < kNumJacks; ++j)
    {
        const uint8_t f = FunctionAt(o, j);
        if (!IsCvInput(f) || !std::isfinite(volts[j])) continue;
        destinations[f] += volts[j] / 5.0f;
    }
}

/** Cache only sampled signals. Depth and source are resolved when the knob is
 * read, after preset loads, pot catch and held gestures have updated them. */
class ModulationMixer
{
  public:
    void Update(const Options& o, const float* volts, const Signals& s)
    {
        CvInputs(o, volts, external_);
        envelopes_[0] = EnvelopeVolts(s.inEnv) / kEnvVolts;
        envelopes_[1] = EnvelopeVolts(s.outEnv) / kEnvVolts;
    }
    float Signal(uint8_t source) const { return envelopes_[source == 1 ? 1 : 0]; }
    float Delta(uint8_t pot, float depth, uint8_t source) const
    {
        if (pot >= kNumDestinations) return 0.0f;
        return external_[pot] + (std::isfinite(depth) ? depth : 0.0f) * Signal(source);
    }
  private:
    float external_[kNumDestinations] = {};
    float envelopes_[2] = {};
};

/** One-shot counterpart for callers that already have a control snapshot. */
inline void Modulation(const Options& o, const float* volts, const float* depth,
                       const uint8_t* source, const Signals& s, float* delta)
{
    ModulationMixer mixer;
    mixer.Update(o, volts, s);
    for (uint8_t p = 0; p < kNumDestinations; ++p)
        delta[p] = mixer.Delta(p, depth[p], source[p]);
}

/** A Schmitt edge detector for every slice input. Entering Slice waits for a
 * low level, so a held gate or the jack's previous output cannot fire it. */
class SliceInputs
{
  public:
    bool Poll(const Options& o, const float* volts, const bool* inputReady)
    {
        bool fired = false;
        for (uint8_t j = 0; j < kNumJacks; ++j)
        {
            const bool enabled = FunctionAt(o, j) == kSlice && inputReady[j]
                                 && std::isfinite(volts[j]);
            const float level = volts[j];
            if (!enabled || !active_[j])
                armed_[j] = enabled && level < 1.0f;
            else if (armed_[j] && level > 2.0f)
            {
                armed_[j] = false;
                fired = true;
            }
            else if (level < 1.0f)
                armed_[j] = true;
            active_[j] = enabled;
        }
        return fired;
    }
  private:
    bool active_[kNumJacks] = {};
    bool armed_[kNumJacks] = {};
};

/** Stage all six DAC targets, latch the shared DAC once, then connect new
 * outputs. Failed writes and direction changes are retried on the next poll.
 * Board is AlchemyLabV2 in firmware and a recording fake in host tests. */
class OutputDriver
{
  public:
    bool InputReady(uint8_t j) const { return input_ready_[j]; }

    template <typename Board>
    void ReadInputs(Board& hw, float* volts, bool* ready = nullptr) const
    {
        for (uint8_t j = 0; j < kNumJacks; ++j)
        {
            const bool input = input_ready_[j] && !hw.cv_jacks[j].IsCvOutput();
            // Value() is raw ADC normalization, not a calibrated voltage.
            volts[j] = input ? hw.cv_jacks[j].Volts() : 0.0f;
            if (ready) ready[j] = input;
        }
    }

    template <typename Board>
    void Apply(Board& hw, const Plan& p)
    {
        bool staged[kNumJacks] = {};
        for (uint8_t j = 0; j < kNumJacks; ++j)
        {
            auto& jack = hw.cv_jacks[j];
            if (!p.output[j])
            {
                valid_[j] = false;
                const bool was_output = jack.IsCvOutput();
                if (was_output) jack.DisableCvOutput();
                input_ready_[j] = !was_output && !jack.IsCvOutput();
                continue;
            }
            input_ready_[j] = false;
            if (valid_[j] && jack.IsCvOutput() && std::fabs(p.volts[j] - written_[j]) <= 0.02f)
                continue;
            // A failed shared-DAC latch leaves the staged shadow pending.
            // Force the next poll to replace it even if the target returns
            // to the last successfully written value.
            valid_[j] = false;
            staged[j] = jack.StageVolts(p.volts[j]);
        }
        if (!hw.FlushCvOutputs()) return;
        for (uint8_t j = 0; j < kNumJacks; ++j)
        {
            if (!staged[j]) continue;
            written_[j] = p.volts[j];
            valid_[j] = true;
            if (!hw.cv_jacks[j].IsCvOutput()) hw.cv_jacks[j].EnableCvOutput();
        }
    }
  private:
    float written_[kNumJacks] = {};
    bool valid_[kNumJacks] = {};
    bool input_ready_[kNumJacks] = {};
};

} // namespace jacks
