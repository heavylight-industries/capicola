/**
 * @file audio_engine.cpp
 * @brief Capicola audio engine — see audio_engine.h.
 */

#include "audio_engine.h"

#include <cmath>
#include "daisy_seed.h"
#include "KeyframeRecorder.h"
#include "Shapers.h"

namespace AlchemyLabAudio {

/* Two independent keyframe rings in external SDRAM (one per channel). 2^20
 * frames × 16 B ≈ 16 MB each → ~32 MB total, the largest power-of-two pair that
 * fits the 64 MB region with double-precision keyframe time. Globals because the
 * DSY_SDRAM_BSS section attribute cannot apply to a class member. */
static constexpr int kRingFrames = 1048576;   // 2^20 per channel
static KeyframeRecorder<kRingFrames> DSY_SDRAM_BSS sparse_l;
static KeyframeRecorder<kRingFrames> DSY_SDRAM_BSS sparse_r;

/* Waveshaper LUTs — 3 × 4096 floats. Kept in SRAM, not SDRAM: read every
 * keyframe. Shared by both channels and the feedback saturator. */
static Shapers shapers;

namespace {
/* OLA crossfade boot value (secondary P2 sets it live): the monitoring latency
 * AND the retrigger floor. */
constexpr float kFadeSamples = 960.0f;    // 20 ms @ 48 kHz

/* Stereo re-alignment guardrail. When exactly one channel auto-fires and the two
 * grids have drifted more than this, the quiet channel is forced to slice,
 * re-anchoring to the same delayed tap. Below this they stay independent. */
constexpr double kMaxDriftSamples = 48000.0;   // 1 s @ 48 kHz

/* Feedback bandpass centre (normalised fc). 0.02 ≈ 480 Hz. */
constexpr float kFbFilterFc = 0.02f;

/* Cancels the sinc shaper's 2x small-signal gain so the loop matches tanh. */
constexpr float kFbSatGain = 0.5f;

/* Follower SVF cutoff (envelope smoothing), shared by the input detectors and
 * the output follower. */
constexpr float kTkeoFc = 0.0014f;

/* Envelope expansion for the normalised follower outputs (UI pip, mod sources,
 * CV outs): the low-passed Teager energy is tiny, so norm = sqrt(clamp01(env ×
 * kEnvGain)). */
constexpr float kEnvGain = 200.0f;

inline float EnvNorm(float env)
{
    float e = env * kEnvGain;
    if (e < 0.0f) e = 0.0f; else if (e > 1.0f) e = 1.0f;
    return std::sqrt(e);
}
} // namespace

void AudioEngine::Init()
{
    /* CPU load meter for the audio callback (48 kHz, 64-sample blocks). */
    cpuMeter.Init(48000.0f, (int)kMaxBlock);

    /* Tabulate the waveshapers before the recorders bind to them. */
    shapers.Init();

    /* Start live capture + granular playback on both channels. */
    sparse_l.Init(&shapers);
    sparse_r.Init(&shapers);
    sparse_l.SetThreshold(0.001f);
    sparse_r.SetThreshold(0.001f);
    sparse_l.SetGrainPitch(1.0f);
    sparse_r.SetGrainPitch(1.0f);
    sparse_l.SetGrainLeash(128);
    sparse_r.SetGrainLeash(128);
    sparse_l.SetGrainFade(kFadeSamples);   // fixed fade = the module latency
    sparse_r.SetGrainFade(kFadeSamples);

    fbSvfL.Init();
    fbSvfR.Init();
    fbSvfL.SetControls(kFbFilterFc, 0.01f);
    fbSvfR.SetControls(kFbFilterFc, 0.01f);

    /* Output-side follower: same chain as the input detectors, run on the final
     * mono-summed output. Never punches — signal source only. */
    outDet.Init(48000.0f);
    SetTkeoCutoff(kTkeoFc);
    inGate     = outGate     = false;
    envNormIn  = envNormOut  = 0.0f;
    inPeak     = 0.0f;

    sparse_l.SubmitRequest(Request::LIVE_EFFECT);
    sparse_r.SubmitRequest(Request::LIVE_EFFECT);
}

void AudioEngine::ProcessBlock(const float* in_l, const float* in_r,
                               float*       out_l, float*       out_r,
                               std::size_t  n)
{
    cpuMeter.OnBlockStart();

    const std::size_t m = (n < kMaxBlock) ? n : kMaxBlock;

    /* Feedback: last block's wet output → tanh → SVF, 50/50 HP+BP mix, scaled and
     * summed into the recorder input. Hard ±1 clamp on the injection. */
    const float* src_l = in_l;
    const float* src_r = in_r;
    float fin_l[kMaxBlock], fin_r[kMaxBlock];
    if (fbAmt > 0.0f)
    {
        for (std::size_t i = 0; i < m; i++)
        {
            fbSvfL.Tick(kFbSatGain * shapers.ReadSinc(fbL[i]));
            fbSvfR.Tick(kFbSatGain * shapers.ReadSinc(fbR[i]));
            const float vl = fbSvfL.GetBandpass();
            const float vr = fbSvfR.GetBandpass();

            float inj_l = vl * fbAmt;
            float inj_r = vr * fbAmt;
            if (inj_l > 1.0f) inj_l = 1.0f; else if (inj_l < -1.0f) inj_l = -1.0f;
            if (inj_r > 1.0f) inj_r = 1.0f; else if (inj_r < -1.0f) inj_r = -1.0f;

            fin_l[i] = in_l[i] + inj_l;
            fin_r[i] = in_r[i] + inj_r;
        }
        src_l = fin_l;
        src_r = fin_r;
    }

    sparse_l.ProcessBlock(src_l, out_l, m);
    sparse_r.ProcessBlock(src_r, out_r, m);

    /* Stereo re-alignment guardrail (kMaxDriftSamples): only on the block where
     * exactly one channel auto-fired. If they've drifted past the guardrail,
     * force the quiet channel to slice — it re-anchors to the same delayed tap. */
    const bool fired_l = sparse_l.FiredThisBlock();
    const bool fired_r = sparse_r.FiredThisBlock();
    if (fired_l != fired_r)
    {
        const double lag_l = sparse_l.GridLag();
        const double lag_r = sparse_r.GridLag();
        const double drift = (lag_l > lag_r) ? (lag_l - lag_r) : (lag_r - lag_l);
        if (drift > kMaxDriftSamples)
            (fired_l ? sparse_r : sparse_l).SubmitRequest(Request::SLICE);
    }

    /* Stash this block's wet (pitched) output for next block's feedback, before
     * the dry/wet blend overwrites the out buffers. */
    for (std::size_t i = 0; i < m; i++) { fbL[i] = out_l[i]; fbR[i] = out_r[i]; }

    /* Dry/wet blend (P5): dry = live input, wet = stretched output (lagged). */
    const float wet = mix;
    const float dry = 1.0f - wet;
    for (std::size_t i = 0; i < m; i++)
    {
        out_l[i] = wet * out_l[i] + dry * in_l[i];
        out_r[i] = wet * out_r[i] + dry * in_r[i];
    }

    /* Output follower consumes the final (post-mix) mono sum. Fed at 2× (no 0.5
     * halving): TKEO is quadratic, so this lifts the envelope 4× to sit level
     * with the per-channel input followers. Gate behavior is unaffected — the
     * dynamic threshold is a scale-invariant ratio over its own average. */
    for (std::size_t i = 0; i < m; i++)
        outDet.Analyze(out_l[i] + out_r[i]);

    const float env_l = sparse_l.TkeoEnvelope();
    const float env_r = sparse_r.TkeoEnvelope();
    envNormIn  = EnvNorm((env_l > env_r) ? env_l : env_r);
    envNormOut = EnvNorm(outDet.Envelope());
    inGate     = sparse_l.DetectorGate() || sparse_r.DetectorGate();
    outGate    = outDet.Gate();

    /* Input peak meter (decaying abs max) — the UI clip indicator reads it. */
    constexpr float kPeakDecay = 0.95f;
    float pk = inPeak * kPeakDecay;
    for (std::size_t i = 0; i < m; i++)
    {
        const float al = std::fabs(in_l[i]);
        const float ar = std::fabs(in_r[i]);
        if (al > pk) pk = al;
        if (ar > pk) pk = ar;
    }
    inPeak = pk;

    cpuMeter.OnBlockEnd();
    cpuAvg = cpuMeter.GetAvgCpuLoad();
    cpuMax = cpuMeter.GetMaxCpuLoad();
}

void AudioEngine::SetSlicePitch(float semis)
{
    const float v = std::exp2(semis / 12.0f);
    sparse_l.SetGrainPitch(v);
    sparse_r.SetGrainPitch(v);
}

void AudioEngine::SetSliceStretch(float stretch)
{
    sparse_l.SetGrainStretch(stretch);
    sparse_r.SetGrainStretch(stretch);
}

void AudioEngine::SetGrainSize(float keyframes)
{
    const int d = (int)(keyframes + 0.5f);
    sparse_l.SetGrainLeash(d);
    sparse_r.SetGrainLeash(d);
}

void AudioEngine::SetQuality(float epsilon)
{
    sparse_l.SetThreshold(epsilon);
    sparse_r.SetThreshold(epsilon);
}

void AudioEngine::SetFeedback(float amt)
{
    fbAmt = (amt < 0.0f) ? 0.0f : amt;
}

void AudioEngine::SetFeedbackCutoff(float fc)
{
    /* P6·shift — open the feedback filter wide for more transient bite; Q stays
     * at the gentle 0.01 (see Init), only fc moves. */
    fbSvfL.SetControls(fc, 0.01f);
    fbSvfR.SetControls(fc, 0.01f);
}

void AudioEngine::SetMix(float wet01)
{
    mix = Clamp01(wet01);
}

void AudioEngine::SetTransientThreshold(float t)
{
    sparse_l.SetTransientThreshold(t);
    sparse_r.SetTransientThreshold(t);
    outDet.SetThreshold(t);        // followers share threshold
}

void AudioEngine::SetTkeoCutoff(float fc)
{
    sparse_l.SetTkeoCutoff(fc);
    sparse_r.SetTkeoCutoff(fc);
    outDet.SetCutoff(fc);          // followers share smoothing
}

void AudioEngine::SetTkeoResonance(float q)
{
    /* Finder SVF resonance, UNCAPPED (filter.h maps Q = 0.5 + 4q); large q rings
     * into self-oscillation. Input side only. */
    const float qq = (q < 0.0f) ? 0.0f : q;
    sparse_l.SetTkeoResonance(qq);
    sparse_r.SetTkeoResonance(qq);
}

void AudioEngine::SetFade(float samples)
{
    sparse_l.SetGrainFade(samples);
    sparse_r.SetGrainFade(samples);
}

void AudioEngine::SetDrive(float d)
{
    sparse_l.SetDistortDrive(d);
    sparse_r.SetDistortDrive(d);
}

void AudioEngine::SetDriveCharacter(float c)
{
    sparse_l.SetDistortCharacter(c);
    sparse_r.SetDistortCharacter(c);
}

void AudioEngine::TriggerSlice()
{
    if (sparse_l.GetState() == State::LIVE_EFFECT)
    {
        sparse_l.SubmitRequest(Request::SLICE);
        sparse_r.SubmitRequest(Request::SLICE);
    }
}

} // namespace AlchemyLabAudio
