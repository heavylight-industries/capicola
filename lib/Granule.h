#pragma once

#include <SparseLine.h>

namespace capicola {

// A reading cursor over the sparse buffer: the window it sits in, its uniform
// (sample-time) index, sub-window phase `tau` in [0,1], advance rate, and last
// value read. The `ideal` cursor (the stretch grid) never populates `result` —
// it only marks metric position; `actual`/`temp` do the reading.
struct Playhead {
    Window w;
    double index;    // uniform position, in samples
    float  tau;      // phase within [p2,p1]; 0 at p1, 1 at p2
    float  speed;    // advance per sample (stretch for `ideal`, pitch for the rest)
    float  result;   // last sampled value (unused for `ideal`)
};

template <int bufsz = 1024>
class Granule {
private:
    SparseLine<bufsz>* sparse_;

    Playhead ideal, actual, temp;

    // Critical positions, kept live as convenient tuples.
    Position writePos;   // the write head              (fence, from SparseLine)
    Position liveEdge;   // furthest safe read          (fence, from SparseLine)
    Position master;     // writePos - fade, clamped    (the delayed tap; fixed-fade target)
    Window   masterW;    // window bracketing `master`, cached per block for re-anchors

    // Live distances, refreshed per sample for triggers + introspection.
    Distance distance;   // actual - ideal(grid)  : the head's lead over the stretch grid
    Distance runway;     // liveEdge - actual     : room ahead of the head
    Distance backlog;    // writePos - ideal(grid): how close the writer is to lapping the grid

    // Controls.
    float pitch;         // rate of `actual` (playback speed / transpose)
    float stretch;       // rate of `ideal` (the stretch grid)
    int   leash;         // adaptive grain size, in keyframes

    // Crossfade state. fixed* is a control (SetFade); adaptive* is computed
    // per-splice. Only one fade runs at a time; `adaptive` selects the increment.
    float fixedDuration;
    float fixedIncrement;
    float adaptiveDuration;
    float adaptiveIncrement;
    float splicePhase;
    bool  splicing;
    bool  adaptive;

    int   blockSize;

    // Re-anchor when the writer has lapped `ideal` to within this many keyframes
    // of capacity. Coarse back-stop; the front guard is the exact one.
    static constexpr int kSafetyKeyframes = bufsz - (bufsz >> 3);  // ~87.5% full

    // Package a cursor from a window that already brackets `pos`.
    static Playhead Head(const Window& w, double pos, float rate) {
        return { w, pos, 0.0f, rate, 0.0f };
    }

    // Sample a cursor at its current index (does not advance it).
    float Sample(Playhead& ph) {
        ph.tau    = (float)(ph.w.endTime - ph.index) * ph.w.invDuration;
        ph.result = sparse_->Read(ph.w, ph.tau, InterpMode::CATMULLROM);
        return ph.result;
    }

    template <typename ShapeFn>
    float Sample(Playhead& ph, ShapeFn fn) {
        ph.tau    = (float)(ph.w.endTime - ph.index) * ph.w.invDuration;
        ph.result = sparse_->Read(ph.w, ph.tau, InterpMode::CATMULLROM, fn);
        return ph.result;
    }

public:
    // Implicit trivial constructor on purpose (SDRAM-resident; see Init()).

    void Init(SparseLine<bufsz>& sparse, double startPos) {
        sparse_ = &sparse;
        pitch   = 1.0f;
        stretch = 1.0f;
        leash   = 128;
        SetFade(2400.0f);   // 50 ms @ 48 kHz
        adaptive  = false;
        blockSize = 0;
        writePos = sparse_->GetWritePosition();
        liveEdge = writePos;
        masterW  = sparse_->WindowAtLatest();
        Spawn(startPos);
    }

    void SetPitch(float s) { pitch = s; actual.speed = s; temp.speed = s; }
    void SetStretch(float t) { stretch = t; ideal.speed = t; }
    void SetLeash(int k)   { leash = k; }
    void SetFade(float f)  { fixedDuration = (f < 1.0f) ? 1.0f : f;
                             fixedIncrement = 1.0f / fixedDuration; }

    float  GetPitch()   const { return pitch; }
    float  GetStretch() const { return stretch; }
    int    GetLeash()   const { return leash; }
    Position GetIdeal() const { return { ideal.w.index, ideal.index }; }

    // Uniform samples the stretch grid sits behind the writer (the read lag);
    // the engine diffs L vs R for the stereo guardrail.
    double   GridLag()   const { return backlog.uniform; }
    Position GetActual() const { return { actual.w.index, actual.index }; }

    Position GetMaster()   const { return master; }
    Position GetLiveEdge() const { return liveEdge; }
    Distance GetRunway()   const { return runway; }
    Distance GetBacklog()  const { return backlog; }
    Distance GetDistance() const { return distance; }
    bool     IsSplicing()  const { return splicing; }
    bool     IsAdaptive()  const { return adaptive; }

    // (Re)seat both cursors at `pos` and clear any in-flight crossfade.
    void Spawn(double pos) {
        Window w = sparse_->WindowAtLatest();
        sparse_->StepWindow(w, pos);
        ideal    = Head(w, pos, stretch);
        actual   = Head(w, pos, pitch);
        splicing    = false;
        splicePhase = 0.0f;
    }

    // Recompute the delayed tap: master = writePos - fade, clamped to
    // [0, liveEdge]. The fade length is the minimum lead, so a full crossfade
    // fits in front of an anchor before the fence. `masterW` steps incrementally
    // (the tap advances ~one block per block); re-seed from the latest window
    // only if the writer lapped the cached window off the ring.
    void UpdateMaster() {
        double m = writePos.uniform - (double)fixedDuration;
        if (m > liveEdge.uniform) m = liveEdge.uniform;
        if (m < 0.0)              m = 0.0;
        master.uniform = m;
        if (masterW.index <= sparse_->GetOldestIndex())
            masterW = sparse_->WindowAtLatest();
        sparse_->StepWindow(masterW, m);
        master.sparse = masterW.index;
    }

    // Snapshot the fences once per block and recompute the derived landmarks.
    void BeginBlock(int size) {
        blockSize = size;
        writePos  = sparse_->GetWritePosition();
        liveEdge  = sparse_->GetMinReadPosition(size);
        UpdateMaster();
    }

    // Begin a FIXED-duration crossfade re-anchoring to the delayed tap. Reuses
    // the cached `masterW` — no walk, no search.
    void ReanchorMaster() {
        temp        = Head(masterW, master.uniform, pitch);
        splicePhase = 0.0f;
        splicing    = true;
        adaptive    = false;
    }

    // Begin a FIXED-duration crossfade back onto the stretch grid, leaving the
    // grid's own time intact. The front guard uses this: when the head races the
    // live edge, only the HEAD needs rescuing — hauling the grid to the tap
    // destroys the stretch timer. At stretch = 1 grid ≈ tap, so this matches the
    // classic OLA delay-span loop.
    void ReanchorGrid() {
        temp        = Head(ideal.w, ideal.index, pitch);
        splicePhase = 0.0f;
        splicing    = true;
        adaptive    = false;
    }

    // Hard-seat both cursors at the delayed tap with NO internal fade — for grain
    // trading, where the caller crossfades between two Granules one layer up (see
    // KeyframeRecorder). BeginBlock has already refreshed master this block; any
    // in-flight splice is simply abandoned, since this grain enters at zero gain.
    void Retrigger() {
        ideal  = Head(masterW, master.uniform, stretch);
        actual = Head(masterW, master.uniform, pitch);
        splicing    = false;
        splicePhase = 0.0f;
    }

    // Produce one output sample, unshaped. Kept as the generic entry point for
    // ports that don't want the waveshaper; delegates through an identity
    // functor so there is only one implementation.
    float Read() { return Read([](float v) { return v; }); }

    // Produce one output sample, shaping each keyframe on the way out.
    template <typename ShapeFn>
    float Read(ShapeFn fn) {
        // Keep every cursor's keyframe bracket ahead of its index.
        if (ideal.index  > ideal.w.endTime)          sparse_->StepWindow(ideal.w,  ideal.index);
        if (actual.index > actual.w.endTime)         sparse_->StepWindow(actual.w, actual.index);
        if (splicing && temp.index > temp.w.endTime) sparse_->StepWindow(temp.w,   temp.index);

        // Refresh live distances. Indices are monotonic, so these are plain
        // subtractions — signed and correct across any number of ring wraps.
        // distance > 0: actual leads the grid; < 0: it lags (pitch down).
        distance.sparse  = actual.w.index - ideal.w.index;
        distance.uniform = actual.index    - ideal.index;
        runway.sparse    = liveEdge.sparse - actual.w.index;
        runway.uniform   = liveEdge.uniform - actual.index;
        backlog.sparse   = writePos.sparse - ideal.w.index;
        backlog.uniform  = writePos.uniform - ideal.index;

        // ── boundary guards (fixed fade → master) ────────────────────────────
        // Safety outranks the adaptive splice: a guard may preempt an adaptive
        // fade, never a fixed one. The front guard only arms when the head is
        // closing on the fence (pitch > 1); it fires when the runway left is less
        // than the fade will consume. One block of slack covers the per-block
        // fence snapshot.
        if (!splicing || adaptive) {
            bool frontBreach = false;
            if (pitch > 1.0f) {
                // In-flight adaptive fade: compare the runway against what the
                // REMAINDER of that fade consumes; idle, against a full fixed fade.
                double closing = splicing
                    ? (double)((1.0f - splicePhase) * adaptiveDuration)
                          * (1.0 - 1.0 / (double)pitch)
                    : (double)fixedDuration * (1.0 - 1.0 / (double)pitch);
                frontBreach = runway.uniform <= closing + (double)blockSize;
            }
            bool backBreach = backlog.sparse >= kSafetyKeyframes;   // writer lapping grid
            if (backBreach) {
                // The GRID is about to fall off the ring: full reset to the tap.
                ReanchorMaster();
                ideal = Head(masterW, master.uniform, stretch);
            } else if (frontBreach) {
                // Only the HEAD is in danger: fade it back onto the stretch grid,
                // leaving the grid's time (the stretch timer) alone.
                ReanchorGrid();
            }
        }

        // ── adaptive splice (adaptive fade → ideal grid) ─────────────────────
        // The head has drifted a full grain off the grid: crossfade back onto it.
        // Symmetric — a lead (pitch up) or lag (pitch down) both trigger. Lowest
        // priority; never interrupts a fade in progress.
        long long lead = distance.sparse < 0 ? -distance.sparse : distance.sparse;
        if (!splicing && lead > leash) {
            temp.index = ideal.index;    // drop the splice cursor on the grid
            temp.speed = pitch;
            temp.w     = ideal.w;

            // Budget = raw-sample span of `leash` keyframes, measured in the
            // direction the splice crosses. A lagging head (pitch down) fades
            // across the keyframes BEHIND the grid; a leading head (pitch up)
            // fades across the span ahead, then CLAMPS to 90% of the runway so
            // the fade lands with room to spare.
            long long sparseIndex = ideal.w.index;
            if (distance.sparse < 0)   // head lags the grid: budget behind
                adaptiveDuration = (float)(sparse_->GetSample(sparseIndex - 1)->time
                                         - sparse_->GetSample(sparseIndex - leash)->time);
            else                       // head leads the grid: budget ahead
                adaptiveDuration = (float)(sparse_->GetSample(sparseIndex + leash - 1)->time
                                         - sparse_->GetSample(sparseIndex - 1)->time);
            float limit = 0.9f * (float)runway.uniform;
            if (adaptiveDuration > limit) adaptiveDuration = limit;
            if (adaptiveDuration < 48.0f) adaptiveDuration = 48.0f;   // 1 ms floor — never a click

            adaptiveIncrement = 1.0f / adaptiveDuration;
            splicePhase       = 0.0f;
            splicing          = true;
            adaptive          = true;
        }

        // Advance the Now marker (non-reading).
        ideal.index += ideal.speed;

        // ── crossfade in progress ────────────────────────────────────────────
        if (splicing && splicePhase <= 1.0f) {
            temp.speed = pitch;                    // stay locked to live pitch
            float in  = Sample(temp, fn);
            float out = Sample(actual, fn);
            temp.index   += temp.speed;
            actual.index += actual.speed;

            const float w = SmoothStep(splicePhase);
            float mixed = (in * w) + (out * (1.0f - w));
            splicePhase += (adaptive ? adaptiveIncrement : fixedIncrement) * pitch;
            return mixed;
        }
        // ── crossfade complete: adopt the incoming cursor ────────────────────
        else if (splicing && splicePhase > 1.0f) {
            actual      = temp;
            splicePhase = 0.0f;
            splicing    = false;
        }

        // ── steady read ──────────────────────────────────────────────────────
        float out = Sample(actual, fn);
        actual.index += actual.speed;
        return out;
    }
};

} // namespace capicola
