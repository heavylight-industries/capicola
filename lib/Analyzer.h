#pragma once

#include <DeluxeLine.h>   // raw input ring + B-spline front end (CubicResult)
#include <SparseLine.h>   // shared sparse keyframe buffer (Keyframe/Window)
#include <cmath>
#include <cstdint>

namespace capicola {

// The keyframe writer. Owns the raw input ring and the write head (rawCount, in
// raw samples), consumes one sample per Analyze(), and sparsifies the stream
// into a borrowed SparseLine — the medium it shares with the Granule reader.
//
// Timestamps in SAMPLES (rawCount, the reader's native unit); carries fs so a
// seconds view is one divide away. Transport policy (stop-when-full, closing
// keyframe) is surfaced as plain queries/commands for the owner to drive.
template <int bufsz = 1024>
class Analyzer {
private:
    SparseLine<bufsz>*  sparse;      // borrowed; owner keeps the storage alive
    DeluxeLine<float,8> raw;         // raw input ring for the B-spline front end
    CubicResult         mostRecent;  // last front-end read (value + d1)

    // 64-bit: both are monotonic for the life of the session and nothing
    // re-Init()s the analyzer, so 32 bits wrapped after 12 h 25 min of uptime.
    int64_t rawCount;                // write head, in raw samples
    int64_t sparseCount;             // keyframes written
    float  threshold;                // value-diff gate for storing a peak
    float  fs;                       // sample rate
    bool   firstAnalysis;            // seed the very first keyframe unconditionally
    bool   armFinal;                 // force one closing keyframe on next Analyze

    inline void Push(const Keyframe& kf) {
        sparse->Write(kf);
        sparseCount++;
        rawCount++;
    }

public:
    // Implicit trivial constructor on purpose (SDRAM-resident; see Init()).

    // Bind the shared buffer and reset the write head. Buffer must outlive this.
    void Init(SparseLine<bufsz>& s, float sampleRate = 48000.0f) {
        sparse        = &s;
        fs            = sampleRate;
        raw.Init();
        rawCount      = 0;
        sparseCount   = 0;
        threshold     = 0.001f;
        firstAnalysis = true;
        armFinal      = false;
        mostRecent    = {};
    }

    void SetThreshold(float t) { threshold = t; }
    void ArmFinalFrame()       { armFinal = true; }   // force a closing keyframe

    int64_t WriteHead()     const { return rawCount; }
    int64_t KeyframeCount() const { return sparseCount; }
    bool   IsEmpty()       const { return sparseCount == 0; }
    bool   IsFull()        const { return sparseCount >= bufsz - 1; }
    float  SampleRate()    const { return fs; }
    double Seconds()       const { return (double)rawCount / fs; }
    Keyframe* GetLatest()        { return sparse->GetLatest(); }

    // Furthest raw-sample time a reader may safely reach: last written sample
    // minus one block. The only wire from writer to reader.
    double LiveEdge(int blockSize) const { return (double)(rawCount - 1 - blockSize); }

    // Local record-edge bandwidth: 1 / raw-sample interval between the two
    // latest keyframes (x fs for Hz). 0 when there is no interval yet.
    float LatestInvDuration() const {
        float d = sparse->GetWindowDuration(sparse->GetLatestIndex());
        return (d > 1e-5f) ? (1.0f / d) : 0.0f;
    }

    // Consume one sample; store a keyframe at each significant peak. Returns true
    // iff a keyframe was written. Advances the write head by 1.
    bool Analyze(float input) {
        raw.Write(input);

        // The lag-3 tap after this Write equals the lag-2 tap before it (the ring
        // advanced by one), so last sample's mostRecent is this sample's older
        // read — one spline eval per sample, not two.
        const CubicResult older = mostRecent;
        mostRecent = raw.ReadBSplineD1Integer(2);

        bool crossedD1 = ((older.d1 > 0.0f && mostRecent.d1 <= 0.0f) ||
                          (older.d1 < 0.0f && mostRecent.d1 >= 0.0f));

        Keyframe* lastFrame = sparse->GetLatest();
        Keyframe kf;
        kf.value = mostRecent.value;
        kf.time  = (rawCount > 0) ? (double)(rawCount - 1) : 0.0;

        float valueDiff = std::abs(mostRecent.value - lastFrame->value);

        if (firstAnalysis) {
            Push(kf);
            firstAnalysis = false;
            return true;
        }
        if (armFinal) {
            Push(kf);
            armFinal = false;
            return true;
        }
        if (crossedD1 && valueDiff > threshold) {
            // Fractional peak location between the two front-end taps.
            float  alpha = std::abs(older.d1) / (std::abs(older.d1) + std::abs(mostRecent.d1));
            double index = (double)(rawCount - 2) + alpha;
            if (std::abs(index - lastFrame->time) > 1.0) {
                kf.time  = index;
                kf.value = raw.ReadBSpline(3.0f - alpha);
                Push(kf);
                return true;
            }
        }

        rawCount++;
        return false;
    }

    // Gap-gated boundary keyframe (call once per block; invariant math at the
    // KeyframeRecorder call site). Writes a keyframe at the current write
    // position only if the newest stored keyframe has fallen more than `maxGap`
    // raw samples behind — i.e. the signal has been extremum-quiet long enough
    // that the reader's coverage needs topping up. Replaces the old
    // unconditional per-block frame, which landed mid-slope and injected
    // block-rate reconstruction error. Does NOT advance the write head. Returns
    // true iff a frame was written.
    bool GuardKeyframe(double maxGap) {
        if ((double)rawCount - sparse->GetLatest()->time <= maxGap)
            return false;
        Keyframe kf;
        kf.value = mostRecent.value;
        kf.time  = (rawCount > 0) ? (double)(rawCount - 1) : 0.0;
        sparse->Write(kf);
        sparseCount++;
        return true;
    }
};

} // namespace capicola
