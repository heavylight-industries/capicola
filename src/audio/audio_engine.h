/**
 * @file audio_engine.h
 * @brief Capicola audio engine — the module's audio-side top level.
 *
 * Owns the two per-channel KeyframeRecorder chains (live capture → B-spline
 * sparsifier → granular playback), the filtered feedback loop, the dry/wet mix,
 * and the output-side follower. The UI layer (src/main.cpp) maps knobs to
 * engineering units and pushes them through the setters below; this class
 * exposes the follower/event/envelope state the panel and CV outs mirror.
 */

#ifndef AUDIO_ENGINE_H
#define AUDIO_ENGINE_H

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <atomic>
#include "hardware.h"          // AUDIO_BLOCK_SAMPLES
#include "util/CpuLoadMeter.h"
#include "Detector.h"

namespace AlchemyLabAudio {

class AudioEngine
{
  public:
    void Init();

    /**
     * Process one stereo audio block: feedback injection → per-channel
     * recorder/granulator → dry/wet mix → output follower. L and R run fully
     * independent chains ("line 0" = L, "line 1" = R throughout).
     */
    void ProcessBlock(const float* in_l, const float* in_r,
                      float*       out_l, float*       out_r,
                      std::size_t  n);

    /* ── Parameter setters — ENGINEERING UNITS ───────────────────────────────
     * Knob→value mapping lives in the VirtualKnob declarations in main.cpp;
     * these take the mapped values directly. */
    void SetSlicePitch          (float speed);   // P1 — signed playback speed (1 = unity, <0 = reverse)
    void SetSliceStretch        (float stretch); // P2 — time stretch, 0.01..1
    void SetTransientThreshold  (float t);       // P3 — finder keep ratio (TKEO gate)
    void SetGrainSize           (float keyframes);// P4 — grain size (adaptive leash, keyframes)
    void SetMix                 (float wet01);   // P5 — dry/wet blend, 1 = full wet
    void SetFeedback            (float amt);     // P6 — TKEO-excited feedback gain
    void SetFeedbackCutoff      (float fc);      // feedback bandpass centre (normalised fc)
    void SetTkeoCutoff          (float fc);      // finder SVF cutoff (envelope smoothing)
    void SetTkeoResonance       (float q);       // finder SVF resonance, UNCAPPED (unbound)
    void SetFade                (float samples); // OLA crossfade = latency = retrigger floor
    void SetDrive               (float d);       // keyframe waveshaper drive
    void SetDriveCharacter      (float c);       // 0 = sinc, 0.5 = clean, 1 = sin
    void SetQuality             (float epsilon); // P5 — analyzer keyframe threshold ε

    /* ── Follower IO signals (input chain + output chain) ────────────────────
     * The OUTPUT follower is a second Detector on the final (post-mix) mono sum;
     * it shares threshold/smoothing with the input followers but never punches —
     * auto-slice fires only from the input side. The two counters are monotonic
     * EVENT counts, not gates: the panel turns each new count into a fixed-width
     * trigger. Both carry the detector's kMinGapSec hold-off, so consecutive
     * events on one counter are always >= 150 ms apart. EnvNorm* is the envelope
     * expanded to 0..1 (sqrt of gained value). */
    uint32_t SliceCount() const { return sliceCount; }   // slices the engine actually took
    uint32_t OutEventCount() const { return outDet.Count(); }  // kept output-follower events
    float EnvNormIn()  const { return envNormIn; }     // 0..1 (max of L/R)
    float EnvNormOut() const { return envNormOut; }    // 0..1

    /** Input peak (decaying abs max of L/R): the UI turns the in-env pip red
     *  when this hits the clip ceiling. */
    float InPeak() const { return inPeak; }

    void TriggerSlice();               // B1 — manual slice on both rings

    /** Audio-callback CPU load (0..1), refreshed every block. */
    float CpuAvg() const { return cpuAvg; }
    float CpuMax() const { return cpuMax; }

  private:
    static inline float Clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

    static_assert(std::atomic<bool>::is_always_lock_free,
                  "Slice handoff must not lock inside the audio interrupt");
    std::atomic<bool> sliceRequested{false};

    float mix = 1.0f;   // dry/wet, 1.0 = full wet (stretched)

    /* P6 feedback: last block's wet output → sinc saturator → SVF bandpass,
     * scaled by fbAmt into this block's input. Hard ±1 clamp guards runaway. */
    static constexpr std::size_t kMaxBlock = AUDIO_BLOCK_SAMPLES;
    float fbAmt = 0.0f;
    float fbL[kMaxBlock] = {0.0f};
    float fbR[kMaxBlock] = {0.0f};
    capicola::StateVariable fbSvfL;   // feedback-path filters (init'd in Init())
    capicola::StateVariable fbSvfR;

    /* Output-side follower (final mono-summed output) + cached IO signals. */
    capicola::Detector outDet;
    /* Slice events: one tick per audio block in which either channel actually
     * punched (post-refractory). L/R skew inside a block collapses here; skew
     * across blocks is absorbed by the panel's pulse generator. Forced slices
     * (B2 / TRIG IN) are counted by the caller, which owns that edge already. */
    uint32_t sliceCount = 0;
    float envNormIn  = 0.0f;
    float envNormOut = 0.0f;
    float inPeak     = 0.0f;

    /* Audio-callback CPU load meter (avg/max over a sliding window). */
    daisy::CpuLoadMeter cpuMeter;
    float               cpuAvg = 0.0f;
    float               cpuMax = 0.0f;
};

} // namespace AlchemyLabAudio

#endif /* AUDIO_ENGINE_H */
