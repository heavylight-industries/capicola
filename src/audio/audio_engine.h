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
#include "hardware.h"          // AUDIO_BLOCK_SAMPLES
#include "util/CpuLoadMeter.h"
#include "Detector.h"
#include "Contour.h"
#include "Pulse.h"
#ifdef CAPICOLA_PROFILE
#include "KeyframeRecorder.h"   // (RecorderStats)
#endif

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
    void SetPcm                 (bool on);       // P5's top: the heads read the input's PCM, not the keyframes
    bool PcmOn() const;
    void SetCombSize            (float size01);  // the feedback comb, 0 = off .. one keyframe period .. the grain (Comb page P1)

#ifdef CAPICOLA_PROFILE
    /* Profiling (the bench image's serial stream). Stage timings in core
     * cycles, accumulated by the audio callback; the main loop takes them. */
    enum Stage { kStFeedback, kStCapL, kStRenderL, kStCapR, kStRenderR,
                 kStAlign, kStPost, kStTotal, kStages };
    struct Profile {
        uint32_t blocks;
        uint64_t sum[kStages];
        uint32_t max[kStages];
        uint32_t nanFb;    // non-finite feedback samples (the diffuser's or the bandpass's), zeroed
        uint32_t nanOut;   // non-finite recorder output samples, zeroed
        uint32_t trips;    // the probe's breaker: 16 blocks running over 98 % of the budget turned the comb off
        uint32_t nested;   // audio callbacks begun while another was still running (counted in main.cpp)
    };
    struct Controls {   // the last values set: load against settings
        float pitch, stretch, thresh, grain, mix, feedback, fade, quality;
        float diffusion, spread;
    };
    float DiffAmount() const;            // the feedback comb's glided amount, 0..1
    float DiffPeriod(int ch) const;      // its L (folded to fit), samples
    void TakeProfile(Profile& out);   // copy and clear (interrupts off a moment)
    void CountNested() { prof.nested++; }
    capicola::RecorderStats StatsL() const;
    capicola::RecorderStats StatsR() const;
    const Controls& GetControls() const { return ctl; }
    static uint32_t BlockBudgetCycles();   // core cycles in one block's real time
#endif

    /* ── Follower IO signals (input chain + output chain) ────────────────────
     * The OUTPUT follower is a second Detector on the final (post-mix) mono sum;
     * it shares threshold/smoothing with the input followers but never punches —
     * auto-slice fires only from the input side. The two counters are monotonic
     * EVENT counts, not gates: the panel turns each new count into a fixed-width
     * trigger. Both carry the detector's kMinGapSec hold-off, so consecutive
     * events on a detector are always >= 23 ms apart. EnvNorm* is the envelope
     * expanded to 0..1 (sqrt of gained value). */
    uint32_t SliceCount() const { return sliceCount; }   // slices the engine actually took
    uint32_t OutEventCount() const { return outDet.Count(); }  // kept output-follower events
    /* CONT, the accent contours (lib/Contour.h), 0..1: from the slices the engine takes (input) and from the
     * output follower's kept hits (output), each hit's strength its envelope peak on a log scale. */
    float ContourIn()  const { return contourIn.Value(); }
    float ContourOut() const { return contourOut.Value(); }
    /* PULS (lib/Pulse.h), 0..1: a Gaussian per hit at the same points and strengths; the input side's peak on the
     * hit as heard (lead = the Fade), the output side's from its hit (lead 0). Width: the Comb page's P3. */
    float PulseIn()  const { return pulseIn.Value(); }
    float PulseOut() const { return pulseOut.Value(); }
    void  SetPulseWidth(float seconds);
    void  SetHitFold(float amount01);   // the hits' strengths through a sine folder, 0 = as they are (Comb page P4)
    void  SetModDelay(float seconds);   // CONT and PULS later than their hits (Comb page P5)
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

    float mix = 1.0f;   // dry/wet, 1.0 = full wet (stretched): the glided value (mixT the target)
    /* Every continuous control glides to what it is set to (heavylight, 2026-11-02: robust to any modulation): the
     * knobs and their modulation arrive once a UI frame (16 ms), and a step there clicked on fast sources. Rates
     * glide over ~8 ms a block; gains also ramp sample by sample through each block. A polarity flip still jumps. */
    float mixT = 1.0f;
    float fbAmtT = 0.0f;
    float pitchT = 1.0f, pitchS = 1.0f, stretchT = 1.0f, stretchS = 1.0f;
    void  GlideControls(std::size_t m, float& fbFrom, float& mixFrom);

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
    capicola::Contour  contourIn, contourOut;
    capicola::Pulse    pulseIn, pulseOut;
    float    fadeSec   = 0.02f; // the Fade (Secondary P2), seconds: the input pulses' lead
    float    hitFold   = 0.0f;  // SetHitFold
    float    modDelay  = 0.0f;  // SetModDelay, seconds
    /* The contours' hits waiting out the delay (QueueContour), oldest first. kPending: 500 ms of hits 23 ms apart. */
    struct Pending { double due; float s; int side; };
    static constexpr int kPending = 32;
    Pending  pending[kPending];
    int      nPending  = 0;
    void     QueueContour(int side, float s, double due);
    float    FoldHit(float s) const;
    uint32_t outSeen   = 0;     // outDet's kept count, last block
    uint64_t sampleNow = 0;     // samples processed (the contours' clock)
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
#ifdef CAPICOLA_PROFILE
    Profile  prof = {};
    Controls ctl  = {};
#endif
    float               cpuAvg = 0.0f;
    float               cpuMax = 0.0f;
};

} // namespace AlchemyLabAudio

#endif /* AUDIO_ENGINE_H */
