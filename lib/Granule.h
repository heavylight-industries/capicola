#pragma once

#include <SparseLine.h>
#include <PcmRing.h>

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
    Playhead prev;       // a preempted fade's incoming grain, kept in the outgoing blend (Preempt)

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
    bool  blendOut;      // the outgoing side is actual and prev at prevW (a guard preempted a fade in flight)
    float prevW;

    int   blockSize;
    int   hold;            // output samples left with no adaptive splice (an attack plays: RetriggerAt)

    // The PCM the heads read at the top of Quality (KeyframeRecorder::SetPcm): its share of each read, 0 = the
    // keyframes only (and the PCM never touched), 1 = the PCM only. A head past what the ring holds reads the
    // keyframes whatever the share.
    const PcmRing* pcm_;
    float pcmW;

public:
    // Profiling counts (monotonic since Init): leash splices, front-guard
    // rescues, back-guard re-anchors.
    uint32_t nSplice, nFront, nBack;
private:

    // Re-anchor when the writer has lapped `ideal` to within this many keyframes
    // of capacity. Coarse back-stop; the front guard is the exact one.
    static constexpr int kSafetyKeyframes = bufsz - (bufsz >> 3);  // ~87.5% full

    // The leash in sparse material (the dt2-keyframer's KF_DIST_PER_K). Where the
    // keyframes between the heads lie further apart than c on average, the
    // leash holds at a distance of K c^2 / spacing, not K keyframes: the
    // sparser, the shorter the grain (a kick's body, a bass note), where K
    // keyframes alone would let the heads drift seconds apart. Below c the
    // keyframe count alone rules. Never closer than kSparseMin (silence).
    static constexpr double kDistPerK  = 48.0;    // c, samples (1 ms)
    static constexpr double kSparseMin = 516.0;   // samples
    // The shortest splice the sparse leash's cap may leave (it splices only past
    // kSparseMin): a splice is a crossfade, never a click.
    static constexpr float kSparseFadeMin = 256.0f;

    // The grain SIZE spans here, in samples: `leash` keyframes at their spacing
    // `span / leash`, or in sparse material the sparse leash's K c^2 / spacing.
    float Grain(double span) const {
        const double kc = (double)leash * kDistPerK;
        return (float)((span > kc) ? kc * kc / span : span);
    }

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

    // From the PCM, by its share (SetPcm). The shaper meets the keyframes only (SparseLine::Read: it shapes the two
    // keyframes, never the sample rate's samples); the PCM is read as it is: shaping it would alias (heavylight,
    // 2026-11-02: no distortion there). A head past what the ring holds reads the keyframes.
    template <typename ShapeFn>
    float SamplePcm(Playhead& ph, ShapeFn fn) {
        if (!pcm_->Holds(ph.index)) return Sample(ph, fn);
        if (pcmW >= 1.0f) return ph.result = pcm_->Read(ph.index);
        Sample(ph, fn);
        ph.result += pcmW * (pcm_->Read(ph.index) - ph.result);
        return ph.result;
    }

    // A head's sample. Which path is decided once a block, not per read (KeyframeRecorder: Read<true> only while
    // the PCM has a share), as the dt2-keyframer's kf_run hoists its checks out of the sample loop.
    template <bool kPcm, typename ShapeFn>
    float Get(Playhead& ph, ShapeFn fn) {
        if constexpr (kPcm) return SamplePcm(ph, fn);
        else return Sample(ph, fn);
    }

public:
    // Implicit trivial constructor on purpose (SDRAM-resident; see Init()).

    void Init(SparseLine<bufsz>& sparse, double startPos) {
        sparse_ = &sparse;
        pcm_    = nullptr;
        pcmW    = 0.0f;
        pitch   = 1.0f;
        stretch = 1.0f;
        leash   = 128;
        SetFade(2400.0f);   // 50 ms @ 48 kHz
        adaptive  = false;
        blockSize = 0;
        hold      = 0;
        nSplice = nFront = nBack = 0;
        writePos = sparse_->GetWritePosition();
        liveEdge = writePos;
        masterW  = sparse_->WindowAtLatest();
        Spawn(startPos);
    }

    void SetPcm(const PcmRing* ring, float share) { pcm_ = ring; pcmW = ring ? share : 0.0f; }
    void SetPitch(float s) { pitch = s; actual.speed = s; temp.speed = s; prev.speed = s; }
    void SetStretch(float t) { stretch = t; ideal.speed = t; }
    void SetLeash(int k)   { leash = k; }
    // The grain's length in samples for keyframes `spacing` apart (Grain: the leash's span, or the sparse leash's).
    float GrainLength(float spacing) const { return Grain((double)leash * spacing); }
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
        blendOut    = false;
        splicePhase = 0.0f;
        hold        = 0;
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
    // A guard preempting an adaptive fade in flight: the outgoing side keeps both of its grains at the blend they
    // had. Restarting the fade from the head alone dropped the incoming grain's share at once: with STR reversed
    // the grid meets the oldest frame every few seconds, and a leash fade running then jumped by up to half scale
    // (host, 1.2.0 too). The guard's fade then starts from exactly what was playing, at its full length.
    void Preempt() {
        blendOut = false;
        if (!splicing) return;
        if (splicePhase > 1.0f) { actual = temp; return; }   // (landed, not yet adopted)
        if (splicePhase > 0.0f) { prev = temp; prevW = SmoothStep(splicePhase); blendOut = true; }
    }

    void ReanchorMaster() {
        Preempt();
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
        Preempt();
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
        blendOut    = false;
        splicePhase = 0.0f;
        hold        = 0;
    }

    // Retrigger at `pos` (at or behind master: the recorder seats a punch so its
    // ramp ends on a transient's onset) and hold off the adaptive splice for
    // `holdSamps` output samples, so the attack plays before the leash can take
    // the head off it.
    void RetriggerAt(double pos, int holdSamps) {
        Window w = masterW;
        sparse_->StepWindow(w, pos);
        ideal  = Head(w, pos, stretch);
        actual = Head(w, pos, pitch);
        splicing    = false;
        blendOut    = false;
        splicePhase = 0.0f;
        hold        = holdSamps;
    }

    // Seat at the stereo partner's grid and head (the same sample times, read
    // from this channel's keyframes), holding off the adaptive splice for
    // `holdSamps` output samples: the two channels realigned, their grains
    // still their own.
    void RetriggerTo(double gridPos, double headPos, int holdSamps) {
        ideal  = Head(sparse_->WindowAt(gridPos), gridPos, stretch);
        actual = Head(sparse_->WindowAt(headPos), headPos, pitch);
        splicing    = false;
        blendOut    = false;
        splicePhase = 0.0f;
        hold        = holdSamps;
    }

    // Produce one output sample, unshaped. Kept as the generic entry point for
    // ports that don't want the waveshaper; delegates through an identity
    // functor so there is only one implementation.
    float Read() { return Read([](float v) { return v; }); }

    // Produce one output sample, shaping each keyframe on the way out.
    template <bool kPcm = false, typename ShapeFn>
    float Read(ShapeFn fn) {
        // Keep every cursor's keyframe bracket AROUND its index. Two-sided: at
        // negative stretch the grid walks backward out of its window, which a
        // past-the-end test never notices, and the window then stays stale for
        // the rest of the pass. StepWindow always handled both directions; only
        // the call was one-sided.
        if (ideal.index  > ideal.w.endTime  || ideal.index  < ideal.w.p2->time)
            sparse_->StepWindow(ideal.w,  ideal.index);
        if (actual.index > actual.w.endTime || actual.index < actual.w.p2->time)
            sparse_->StepWindow(actual.w, actual.index);
        if (splicing && (temp.index > temp.w.endTime || temp.index < temp.w.p2->time))
            sparse_->StepWindow(temp.w,   temp.index);
        if (blendOut && (prev.index > prev.w.endTime || prev.index < prev.w.p2->time))
            sparse_->StepWindow(prev.w,   prev.index);

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
            // Back edge, from either side: the writer lapping the grid, or the
            // grid reversing onto the oldest frame still live. Both mean the
            // same thing — there is nothing behind us — and both loop to the
            // delayed tap over a full fixed fade.
            bool backBreach = backlog.sparse >= kSafetyKeyframes
                           || ideal.w.index <= sparse_->GetOldestIndex() + 1;
            if (backBreach) {
                // The GRID is about to fall off the ring: full reset to the tap.
                nBack++;
                ReanchorMaster();
                ideal = Head(masterW, master.uniform, stretch);
            } else if (frontBreach) {
                // Only the HEAD is in danger: fade it back onto the stretch grid,
                // leaving the grid's time (the stretch timer) alone.
                nFront++;
                ReanchorGrid();
            }
        }

        // ── adaptive splice (adaptive fade → ideal grid) ─────────────────────
        // The head has drifted a full grain off the grid: crossfade back onto it.
        // Symmetric — a lead (pitch up) or lag (pitch down) both trigger. Lowest
        // priority; never interrupts a fade in progress.
        // The sparse leash too: the heads further apart than K c^2 / spacing,
        // i.e. distance^2 > K c^2 keyframes. Neither while an attack holds.
        long long lead = distance.sparse < 0 ? -distance.sparse : distance.sparse;
        bool go = lead > leash;
        const double ds = distance.uniform < 0.0 ? -distance.uniform : distance.uniform;
        if (ds > kSparseMin
            && ds * ds > (double)leash * kDistPerK * kDistPerK * (double)(lead < 1 ? 1 : lead))
            go = true;
        if (hold > 0) { hold--; go = false; }
        if (!splicing && go) {
            nSplice++;
            temp.index = ideal.index;    // drop the splice cursor on the grid
            temp.speed = pitch;
            temp.w     = ideal.w;

            // Budget = raw-sample span of `leash` keyframes, measured in the
            // direction the splice crosses. A lagging head (pitch down) fades
            // across the keyframes BEHIND the grid; a leading head (pitch up)
            // fades across the span ahead, then CLAMPS to 90% of the runway so
            // the fade lands with room to spare.
            long long sparseIndex = ideal.w.index;
            // Budget behind when the head lags the grid, and also whenever the
            // grid itself is reversing — that is the side it is heading into.
            // The sign of `stretch` already says so; no extra state for it.
            if (distance.sparse < 0 || stretch < 0.0f)
                adaptiveDuration = (float)(sparse_->GetSample(sparseIndex - 1)->time
                                         - sparse_->GetSample(sparseIndex - leash)->time);
            else                       // head leads the grid: budget ahead
                adaptiveDuration = (float)(sparse_->GetSample(sparseIndex + leash - 1)->time
                                         - sparse_->GetSample(sparseIndex - 1)->time);
            {   // (sparse: K c^2 / spacing, never under kSparseFadeMin unless the span is)
                const float span = adaptiveDuration;
                adaptiveDuration = Grain((double)span);
                const float lo = (span < kSparseFadeMin) ? span : kSparseFadeMin;
                if (adaptiveDuration < lo) adaptiveDuration = lo;
            }
            float limit = 0.9f * (float)runway.uniform;
            if (adaptiveDuration > limit) adaptiveDuration = limit;
            if (adaptiveDuration < 48.0f) adaptiveDuration = 48.0f;   // 1 ms floor — never a click

            // A head running against the grid (reverse pitch on a forward grid, or a head far slower than the
            // grid): started on the grid, the incoming head parts from it over the fade by
            // |pitch - stretch| / |pitch| x the fade, more than the leash it was spliced for (host: landing two
            // leashes off, straying to four, splicing ~100% of the time). It starts instead where it meets the
            // grid as the fade lands (the dt2-keyframer's reverse-loop fix, which shortens the fade; here the
            // fade stays whole), inside the live ring.
            {
                const float sa  = pitch < 0.0f ? -pitch : pitch;
                const float rel = (pitch - stretch < 0.0f) ? stretch - pitch : pitch - stretch;
                if (rel > sa) {
                    double off = (double)(stretch - pitch) * (double)adaptiveDuration / (double)sa;
                    const double ahead  = liveEdge.uniform - ideal.index - (double)blockSize;
                    const double behind = ideal.index
                                        - sparse_->GetSample(sparse_->GetOldestIndex() + 2)->time;
                    if (off > ahead)   off = ahead > 0.0 ? ahead : 0.0;
                    if (off < -behind) off = behind > 0.0 ? -behind : 0.0;
                    if (off != 0.0) {
                        temp.index = ideal.index + off;
                        temp.w     = sparse_->WindowAt(temp.index);
                    }
                }
            }

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
            float in  = Get<kPcm>(temp, fn);
            float out = Get<kPcm>(actual, fn);
            if (blendOut) {   // (Preempt: the outgoing side the blend the guard found)
                prev.speed = pitch;
                out += (Get<kPcm>(prev, fn) - out) * prevW;
                prev.index += prev.speed;
            }
            temp.index   += temp.speed;
            actual.index += actual.speed;

            const float w = SmoothStep(splicePhase);
            float mixed = (in * w) + (out * (1.0f - w));
            // |pitch|: the fade spans buffer samples, crossed in either direction.
            splicePhase += (adaptive ? adaptiveIncrement : fixedIncrement)
                         * (pitch < 0.0f ? -pitch : pitch);
            return mixed;
        }
        // ── crossfade complete: adopt the incoming cursor ────────────────────
        else if (splicing && splicePhase > 1.0f) {
            actual      = temp;
            splicePhase = 0.0f;
            splicing    = false;
            blendOut    = false;
        }

        // ── steady read ──────────────────────────────────────────────────────
        float out = Get<kPcm>(actual, fn);
        actual.index += actual.speed;
        return out;
    }
};

} // namespace capicola
