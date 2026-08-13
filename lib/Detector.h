#pragma once

#include <cmath>
#include <cstdint>
#include <DeluxeLine.h>
#include <SparseLine.h>
#include "filter.h"

namespace capicola {

// Transient detector: TKEO front end → SVF lowpass → d1 zero-crossing peak
// picker (rising→falling only, no troughs) over the B-spline-read envelope.
// EVERY peak above the silence floor is stored as a timestamped keyframe; the
// threshold decides only whether an event is KEPT (flagged), not whether it
// exists. Peak time/value are refined to sub-sample precision from the d1 zero
// crossing.
//
// The threshold is ADAPTIVE: a ratio over a slow one-pole average of the
// smoothed envelope STREAM (kAvgSec), not of the peak events. The stream
// average is time-indexed — silence pulls it down — so an isolated hit after a
// gap towers over the reference and passes. Event density is set upstream by
// the smoothing cutoff (raw TKEO = dense bursts; lower cutoff merges peaks). No
// hysteresis, no refractory: a peak picker is discrete, so nothing chatters.
class Detector {
  public:
    static constexpr int kMaxEvents = 2048;   // power of 2 (shared ring mask)

    SparseLine<kMaxEvents> events;   // EVERY detected peak (time in SECONDS)
    DeluxeLine<float, 8> raw;
    DeluxeLine<float, 8> env;        // filtered envelope — B-spline peak picking
    CubicResult prevNewer;           // last sample's lag-2 env read (see Analyze)
    StateVariable svf;
    OnePole avgLp;                   // envelope-stream average (kAvgSec)

    float    fs;
    float    threshold;    // KEEP ratio over the envelope average (avgLp)
    float    cutoff;       // SVF cutoff (normalised fc — see filter.h)
    float    resonance;    // SVF resonance, UNCAPPED (Q = 0.5 + 4*q); high = ringy
    double   clock;        // seconds
    uint32_t keptCount;    // monotonic count of KEPT events
    uint32_t totalCount;   // monotonic count of ALL stored events
    int      gateSamps;    // kept-event monostable countdown (gate/LED/CV)

    // Trivially constructible on purpose (SDRAM-resident) — see Init().
    bool keptFlags[kMaxEvents];   // slaved to events' monotonic index (& mask)

    void Init(float sample_rate = 48000.0f) {
        fs = sample_rate;
        events.Init();
        raw.Init();
        env.Init();
        svf.Init();
        avgLp.Init();
        // kAvgSec time constant → normalised fc: 1/(2π·τ) Hz over fs/2.
        avgLp.SetCutoff(1.0f / ((float)M_PI * kAvgSec * fs));
        cutoff     = 0.01f;
        resonance  = 0.0f;
        svf.SetControls(cutoff, resonance);
        threshold  = 1.0f;
        clock      = 0.0;
        keptCount  = 0;
        totalCount = 0;
        gateSamps  = 0;
        prevNewer  = {};   // matches a lag-3 read of the freshly zeroed ring
        for (int i = 0; i < kMaxEvents; i++) keptFlags[i] = false;
    }

    void SetCutoff(float fc)     { cutoff = fc; svf.SetControls(cutoff, resonance); }
    void SetResonance(float r)   { resonance = r; svf.SetControls(cutoff, resonance); }
    void SetThreshold(float t)   { threshold = t; }

    float    Envelope()   const { return svf.GetLowpass(); }    // smoothed |TKEO|
    float    Average()    const { return avgLp.GetLowpass(); }  // threshold reference
    uint32_t Count()      const { return keptCount; }
    uint32_t TotalCount() const { return totalCount; }
    bool     Gate()       const { return gateSamps > 0; }       // kept-event monostable

    // Was this stored event kept? `idx` is the events ring's MONOTONIC index;
    // callers must stay within the live window.
    bool EventKept(long long idx) const { return keptFlags[idx & (kMaxEvents - 1)]; }

    // kAvgSec: time constant of the envelope-stream average. Long enough that a
    // transient's own swell barely moves it, short enough to re-reference within
    // a second or so after a level change or silence.
    static constexpr float kAvgSec    = 0.1f;
    static constexpr float kAbsFloor  = 1e-5f;   // silence guard: sub-floor peaks aren't events
    static constexpr float kMuteAt    = 100.0f;  // threshold ≥ this = hard mute (UI sentinel)
    static constexpr float kGateSec   = 0.02f;   // kept-event gate width (CV gate outs + LEDs)

    // Consume one sample; returns true iff a KEPT event was stored this sample.
    inline bool Analyze(float x) {
        raw.Write(x);
        svf.Tick(raw.ReadBSplineFull(2.0f, 1).tkeo);
        const float e = svf.GetLowpass();
        env.Write(e);
        avgLp.Tick(e);

        // The lag-3 tap after this Write equals the lag-2 tap before it, so last
        // sample's `newer` is this sample's `older` — one spline eval per sample.
        const CubicResult newer = env.ReadBSplineFull(2.0f, 1);
        const CubicResult older = prevNewer;
        prevNewer = newer;

        bool fired = false;
        // Peak only. d1 is d/d(delay); the delay axis points backward in time, so
        // a time-domain peak is rising at the older tap (d1 < 0) and falling at
        // the newer (d1 >= 0).
        if (older.d1 < 0.0f && newer.d1 >= 0.0f) {
            // Sub-sample peak: interp the d1 zero between lag 3 and lag 2, then
            // one fractional B-spline read for the value there.
            const float dd    = older.d1 - newer.d1;   // < 0 by the test above
            const float frac  = (dd < -1e-20f) ? (older.d1 / dd) : 0.5f;
            const float lag   = 3.0f - frac;
            const float value = env.ReadBSplineFull(lag, 1).value;

            if (value > kAbsFloor) {
                const bool kept = (threshold < kMuteAt)
                               && (value > threshold * avgLp.GetLowpass());
                events.Write({value, clock - (double)lag / (double)fs});
                keptFlags[events.GetLatestIndex() & (kMaxEvents - 1)] = kept;
                totalCount++;
                if (kept) {
                    keptCount++;
                    gateSamps = (int)(kGateSec * fs);
                    fired     = true;
                }
            }
        }
        if (gateSamps > 0) gateSamps--;
        clock += 1.0 / fs;
        return fired;
    }
};

} // namespace capicola
