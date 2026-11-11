#pragma once

#include <Analyzer.h>    // keyframe writer (raw ring + sparsifier)
#include <Granule.h>     // self-contained grain reader (own fences, self-healing)
#include <Detector.h>    // transient finder (TKEO + SVF + peak keyframer)
#include <Shapers.h>     // tabulated waveshapers (optional; Init(nullptr) = clean)
#include <cstddef>
#include <cstdint>

// Profiling (the bench image): a cycle counter the firmware provides, so each
// stage of a block can be timed. Elsewhere the stamps compile to nothing.
#ifdef CAPICOLA_PROFILE
extern "C" uint32_t capicola_cycles(void);
#define CAPICOLA_CYC() capicola_cycles()
#else
#define CAPICOLA_CYC() 0u
#endif

namespace capicola {

// What a recorder has done (profiling): monotonic counts, and the last block's
// stage timings in the firmware's cycle units (0 without CAPICOLA_PROFILE).
struct RecorderStats {
    uint64_t keyframes;     // written by the analyzer
    uint32_t detected;      // detector peaks stored
    uint32_t kept;          // ... kept
    uint32_t punches;       // hits punched (after the refractory)
    uint32_t follows;       // stereo follows taken
    uint32_t splices;       // leash splices, both grains
    uint32_t fronts;        // front-guard rescues
    uint32_t backs;         // back-guard re-anchors
    bool     silent;        // the analyzer is inside a silence
    uint32_t capCycles;     // last block: analyzer + detector
    uint32_t renderCycles;  // last block: punches + grain reads + guard frame
};

// Transport state of the recorder.
enum class State : uint8_t {
    STOPPED,
    RECORDING,
    PLAYBACK,
    LIVE_EFFECT,
};

// Change requests into the state machine.
enum class Request : uint8_t {
    NONE,
    ARM,
    RECORD,
    PLAY,
    RESTART_PLAYBACK,
    CLEAR,
    STOP_REC,
    STOP_PLAYING,
    LIVE_EFFECT,
    SLICE,
};

template <int bufsz = 1024>
class KeyframeRecorder {
private:
    SparseLine<bufsz>* sparse;   // the keyframe ring: storage the owner places (SDRAM: it is large)
    Analyzer<bufsz>   analyzer;

    // Two grains trading places over one gain ramp. A trigger always punches
    // through: the idle grain hard-seats at the delayed tap (Retrigger) and fades
    // in over curFade while the old grain finishes its in-flight splice at falling
    // gain. No internal fade is ever interrupted, so a punch can never click.
    Granule<bufsz> granule[2];
    int   liveG;     // index of the grain fading in / steady
    PcmRing* pcmRing;   // the input as PCM (optional: Init); null = keyframes only
    bool     pcmOn;     // SetPcm: play from the PCM
    float    pcmW;      // the PCM's share of every read, ramping to pcmOn over kPcmFade
    float    shapeNorm; // the shaped keyframes' level match (UpdateShapeNorm)
    bool     pcmBlock;  // this block reads the PCM (pcmOn, or its share still ramping out)
    float xGain;     // gain of granule[liveG]; the other grain gets 1-xGain
    float xStep;     // per-sample ramp, 1/curFade

    Detector detector;   // transient finder over the recorded input
    uint32_t transientCount;              // monotonic; the UI polls it for edges

    State   currentState;
    Request currentRequest;

    // Current control snapshot, re-applied whenever the reader is re-Init'd.
    float curPitch;
    float curStretch;
    int   curLeash;
    float curThreshold;
    float curFade;

    bool  pendingSeed;    // spawn a fresh voice on the next render block
    bool  liveSeeded;     // LIVE: the first grain is seated (see ProcessBlock)
    float seedFade;       // ... and fading in from silence, 0..1
    bool  pendingHit;     // ... for a detected hit (its attack holds the leash off)
    bool  firedThisBlock; // detector auto-fired this block (stereo-guard edge; NOT forced slices)
    int64_t sinceFire;    // samples since this channel last punched on a hit (own or the partner's)
    uint32_t followCount;    // stereo follows taken (profiling)
    uint32_t gSplice, gFront, gBack;   // grain counts folded in at each ResetReader (profiling)
    uint32_t capCycles, renderCycles;  // last block's stage timings (profiling)
    bool  pendingFollow;  // seat on the partner's grid and head next block (Follow)
    double followGrid, followHead;
    int   followSteps;    // its ramp, output samples
    int   followHold;
    int   lastBlockSize;

    // LIVE trigger path: a kept detector event punches the block it lands in,
    // through a non-retriggerable refractory (trigGate). The refractory length is
    // curFade — a punch physically lasts one crossfade, so that's the retrigger
    // floor; dense kept bursts retrigger every fade instead of latching. B1
    // (SLICE) bypasses this.
    float trigGate;   // refractory countdown; doubles as the B1 LED state

    // Waveshaper. Applied to KEYFRAMES on the way out, never to the interpolated
    // stream — the nonlinearity never sees the sample rate, so it cannot alias.
    const Shapers* shapers;   // null = clean path, no shaping at all
    float drive;
    float character;          // 0 = quake, 0.5 = bypass, 1 = sinc

    static constexpr float kBypassEps = 1e-4f;
    // The keyframes <-> PCM crossfade at the top of Quality: 20 ms, run once when the knob crosses the boundary.
    static constexpr float kPcmFade = 960.0f;
    static constexpr float kPcmStep = 1.0f / kPcmFade;
    // After a hit's ramp, how long no leash splice may take the head off the
    // attack, output samples (the dt2-keyframer's KF_ATTACK_HOLD).
    static constexpr int   kAttackHold = 1024;
    // A seat stays this far behind the live edge.
    static constexpr float kEdgeMargin = 64.0f;

    inline float Shape(float x) const {
        if (!shapers) return x;
        const float d = character - 0.5f;
        if (d > -kBypassEps && d < kBypassEps) return x;
        const float xd = x * drive;
        if (d < 0.0f) {
            const float t = character * 2.0f;
            return (1.0f - t) * shapers->ReadQuake(xd) + t * x;
        }
        const float t = d * 2.0f;
        return (1.0f - t) * x + t * shapers->ReadSinc(xd);
    }

    // The keyframe path's level against Drive and Character (heavylight, 2026-11-02: more distorted, not louder, so
    // it sits at the PCM top's level): the shaped keyframes are scaled so a sine at kShapeRef of full scale (about
    // -10 dBFS) comes out at its own RMS. (Not by the shaper's slope at 0: the Quake side's is 0.) Worked out when
    // either knob moves, kept within kShapeNormMin .. kShapeNormMax.
    static constexpr float kShapeRef     = 0.3f;
    static constexpr float kShapeNormMin = 0.25f;
    static constexpr float kShapeNormMax = 4.0f;
    void UpdateShapeNorm() {
        double in = 0.0, out = 0.0;
        for (int i = 0; i < 64; i++) {
            const float x = kShapeRef * std::sin(6.2831853f * ((float)i + 0.5f) / 64.0f);
            const float y = Shape(x);
            in += (double)x * x;
            out += (double)y * y;
        }
        float k = (out > 1e-12) ? (float)std::sqrt(in / out) : kShapeNormMax;
        shapeNorm = k < kShapeNormMin ? kShapeNormMin : k > kShapeNormMax ? kShapeNormMax : k;
    }

    void ApplyParams() {
        for (Granule<bufsz>& g : granule) {
            g.SetPitch(curPitch);
            g.SetStretch(curStretch);
            g.SetLeash(curLeash);
            g.SetFade(curFade);
        }
        analyzer.SetThreshold(curThreshold);
    }

    // Bind fresh readers over the (already Init'd) buffer and re-apply controls.
    void ResetReader() {
        for (const Granule<bufsz>& g : granule) {   // (keep the profiling counts monotonic)
            gSplice += g.nSplice; gFront += g.nFront; gBack += g.nBack;
        }
        granule[0].Init(*sparse, 0.0);
        granule[1].Init(*sparse, 0.0);
        liveG = 0;
        xGain = 1.0f;
        xStep = 0.0f;
        ApplyParams();
        pendingFollow = false;
    }

    // Unconditional punch-through (button / transient). Swaps to the idle grain,
    // seats it at the tap, and starts the gain ramp over the full curFade (a
    // punch never clicks). The tap is the write head less the fade, not master
    // (the newest keyframe less the fade, which trails the writer by up to a
    // keyframe gap), so every punch leaves at the same latency; never past the
    // live edge. A hit's attack then holds the leash off (kAttackHold). Never
    // mid-ramp in LIVE (ProcessBlock waits for the running ramp to land).
    void Punch(bool hit = false) {
        liveG ^= 1;
        Granule<bufsz>& g = granule[liveG];
        double tap = (double)analyzer.WriteHead() - (double)curFade;
        const double edge = g.GetLiveEdge().uniform - (double)kEdgeMargin;
        if (tap > edge) tap = edge;
        const float ramp = (curFade < 1.0f) ? 1.0f : curFade;
        g.RetriggerAt(tap, hit ? (int)ramp + kAttackHold : 0);
        g.SetPitch(curPitch);
        xGain = 0.0f;
        xStep = 1.0f / ramp;
    }

    // A stereo follow (see Follow): the idle grain at the partner's grid and
    // head, over the same ramp as a punch.
    void FollowPunch() {
        liveG ^= 1;
        granule[liveG].RetriggerTo(followGrid, followHead, followHold);
        granule[liveG].SetPitch(curPitch);
        xGain = 0.0f;
        xStep = 1.0f / (float)followSteps;
    }

    // Direction change. Reversing a moving head in place is a corner in the
    // waveform, so the idle grain takes a copy of the live one — same cursors,
    // same in-flight splice — and plays on in the new direction while the old
    // one fades out in the old. Same ramp as a punch, and the same refractory,
    // so a transient can't punch into it.
    void Flip() {
        Granule<bufsz>& idle = granule[liveG ^ 1];
        const uint32_t ns = idle.nSplice, nf = idle.nFront, nb = idle.nBack;
        idle = granule[liveG];
        idle.nSplice = ns;   // (each grain keeps its own profiling counts: the
        idle.nFront  = nf;   //  copy took the live one's, and the sum ran back)
        idle.nBack   = nb;
        liveG ^= 1;
        xGain    = 0.0f;
        xStep    = 1.0f / ((curFade < 1.0f) ? 1.0f : curFade);
        trigGate = curFade;
    }

    // Once per block, after any punch: only the live grain follows the knob;
    // the outgoing one finishes its ramp at the rate it had. A sign change
    // trades grains, and waits for a running ramp to land rather than reuse a
    // grain that is still audible.
    void TrackPitch() {
        if ((curPitch < 0.0f) != (granule[liveG].GetPitch() < 0.0f)) {
            if (xGain < 1.0f) return;
            Flip();
        }
        granule[liveG].SetPitch(curPitch);
    }

    // Mixed read of the trading pair; the retired grain is skipped once the ramp
    // completes.
    float ReadPair() { return pcmBlock ? ReadPairT<true>() : ReadPairT<false>(); }

    // kPcm: the PCM has a share this block (pcmBlock, set once a block): its ramp and its reads. Without it the
    // keyframes' path runs as it did before the PCM.
    template <bool kPcm>
    float ReadPairT() {
        auto shape = [this](float v) { return Shape(v) * shapeNorm; };
        if constexpr (kPcm) {   // (the PCM's share: to its target in kPcmFade samples, once, at the boundary)
            const float tgt = pcmOn ? 1.0f : 0.0f;
            if (pcmW != tgt) {
                pcmW += (tgt > pcmW) ? kPcmStep : -kPcmStep;
                if ((tgt - pcmW) * (pcmOn ? 1.0f : -1.0f) <= 0.0f) pcmW = tgt;
                granule[0].SetPcm(pcmRing, pcmW);
                granule[1].SetPcm(pcmRing, pcmW);
            }
        }
        if (xGain >= 1.0f) return granule[liveG].template Read<kPcm>(shape);
        const float in  = granule[liveG].template Read<kPcm>(shape);
        const float out = granule[liveG ^ 1].template Read<kPcm>(shape);
        const float w   = SmoothStep(xGain);
        const float y   = in * w + out * (1.0f - w);
        xGain += xStep;
        if (xGain > 1.0f) xGain = 1.0f;
        return y;
    }

public:
    // Trivially constructible on purpose: instances are DSY_SDRAM_BSS globals, so
    // any ctor code would run during static init — before main() configures the
    // SDRAM controller — and bus-fault on the first write. Everything initializes
    // in Init(), called from AudioEngine::Init() after hw.Init().

    // `ring` holds the keyframes and must outlive the recorder. Apart from it a
    // recorder is small (its grains, analyzer, detector), so it can live in
    // fast internal RAM while the ring sits in SDRAM.
    // `pcm` (optional) is the input as PCM, written in lockstep with the Analyzer: SetPcm plays from it.
    void Init(SparseLine<bufsz>& ring, const Shapers* sh = nullptr, PcmRing* pcm = nullptr) {
        sparse    = &ring;
        shapers   = sh;
        pcmRing   = pcm;
        if (pcmRing) pcmRing->Reset();
        pcmOn = false;
        pcmW  = 0.0f;
        pcmBlock = false;
        drive     = 1.0f;
        character = 0.5f;   // bypass
        shapeNorm = 1.0f;
        sparse->Init();
        analyzer.Init(*sparse);
        granule[0].Init(*sparse, 0.0);
        granule[1].Init(*sparse, 0.0);
        liveG = 0;
        xGain = 1.0f;
        xStep = 0.0f;
        followCount = 0;
        gSplice = gFront = gBack = 0;
        capCycles = renderCycles = 0;
        detector.Init(48000.0f);
        transientCount = 0;

        curPitch     = 1.0f;
        curStretch   = 1.0f;
        curLeash     = 128;
        curThreshold = 0.001f;
        curFade      = 2400.0f;   // 50 ms @ 48 kHz
        ApplyParams();

        currentState   = State::STOPPED;
        currentRequest = Request::NONE;
        pendingSeed    = false;
        liveSeeded     = false;
        seedFade       = 1.0f;
        pendingHit     = false;
        sinceFire      = 1 << 30;
        pendingFollow  = false;
        followGrid = followHead = 0.0;
        followSteps = 1;
        followHold  = 0;
        firedThisBlock = false;
        lastBlockSize  = 0;
        trigGate       = 0.0f;
    }

    void SetGrainPitch(float s)   { curPitch = s; }   // signed; applied per block (TrackPitch)
    void SetGrainStretch(float t) { curStretch = t; for (auto& g : granule) g.SetStretch(t); }
    void SetGrainLeash(int k)     { curLeash = k; for (auto& g : granule) g.SetLeash(k); }
    void SetThreshold(float v)    { curThreshold = v; analyzer.SetThreshold(v); }
    void SetGrainFade(float f)    { curFade = f; for (auto& g : granule) g.SetFade(f); }

    void SetTransientThreshold(float t) { detector.SetThreshold(t); }
    void SetTkeoCutoff(float fc)        { detector.SetCutoff(fc); }
    void SetTkeoResonance(float r)      { detector.SetResonance(r); }
    float ShapedForTest(float v) const  { return Shape(v) * shapeNorm; }   // (the host tests: the keyframes' shaping)
    void SetDistortDrive(float d)       { drive = d; UpdateShapeNorm(); }
    void SetDistortCharacter(float c)   { character = c; UpdateShapeNorm(); }

    State GetState() { return currentState; }

    // The top of Quality: play from the PCM (true) or the keyframes. Crossing over crossfades the two once, over
    // kPcmFade; past it only the one runs.
    void SetPcm(bool on) { pcmOn = on && pcmRing; }
    bool PcmOn() const { return pcmOn; }

    uint32_t TransientCount() const { return transientCount; }
    float    PairGain()       const { return xGain; }             // takeover ramp state
    float    TkeoEnvelope()   const { return detector.Envelope(); }
    float    LastHitValue()   const { return detector.lastKept; }   // the latest kept hit's envelope peak
    // Raw samples between the two latest keyframes (an extremum each: about half the input's period); 0 before the
    // first interval.
    float    KeyframeSpacing() const {
        const float inv = analyzer.LatestInvDuration();
        return (inv > 0.0f) ? 1.0f / inv : 0.0f;
    }
    // The grain's length in samples at the latest keyframe spacing (Grain Size keyframes, or the sparse leash's
    // span); 0 before the first interval. The feedback diffuser's taps follow it.
    float    GrainLength() const {
        const float sp = KeyframeSpacing();
        return (sp > 0.0f) ? granule[0].GrainLength(sp) : 0.0f;
    }

    // Stereo re-alignment guardrail (see AudioEngine::ProcessBlock): did the
    // detector auto-fire this block (forced slices excluded), and how far behind
    // the writer the live grid sits right now.
    bool     FiredThisBlock() const { return firedThisBlock; }
    double   GridLag()        const { return granule[liveG].GridLag(); }

    // Stereo coincidence (lib/StereoAlign.h): samples since this channel last
    // punched on a hit, and whether its own envelope is near its keep level.
    int64_t  SinceFire()      const { return sinceFire; }
    bool     NearHit(float share) const { return detector.Near(share); }

    // Realign onto the partner: on the next block the idle grain takes the
    // leader's grid and head positions and fades in over a punch's ramp; on a
    // hit the attack holds the leash off, as a punch's does. Counts as this
    // channel's punch for the refractory, so its own detection of the same hit
    // cannot punch again.
    void Follow(const KeyframeRecorder& leader, bool hit) {
        if (currentState != State::LIVE_EFFECT || !liveSeeded || !leader.liveSeeded) return;
        const Granule<bufsz>& g = leader.granule[leader.liveG];
        followGrid = g.GetIdeal().uniform;
        followHead = g.GetActual().uniform;
        // Never past this channel's own live edge: its newest keyframe can
        // trail the leader's by up to a guard gap (a pitched-up leader reads
        // close to its own edge).
        const double edge = granule[liveG].GetLiveEdge().uniform - (double)kEdgeMargin;
        if (followGrid > edge) followGrid = edge;
        if (followHead > edge) followHead = edge;
        followSteps   = (curFade < 1.0f) ? 1 : (int)curFade;
        followHold    = hit ? followSteps + kAttackHold : 0;
        pendingFollow = true;
        followCount++;
        trigGate      = curFade;
        if (hit) sinceFire = 0;
    }

    void SubmitRequest(Request r) { currentRequest = r; }

    RecorderStats Stats() const {
        RecorderStats s;
        s.keyframes = (uint64_t)analyzer.KeyframeCount();
        s.detected  = detector.TotalCount();
        s.kept      = detector.Count();
        s.punches   = transientCount;
        s.follows   = followCount;
        s.splices   = gSplice + granule[0].nSplice + granule[1].nSplice;
        s.fronts    = gFront  + granule[0].nFront  + granule[1].nFront;
        s.backs     = gBack   + granule[0].nBack   + granule[1].nBack;
        s.silent    = analyzer.InSilence();
        s.capCycles    = capCycles;
        s.renderCycles = renderCycles;
        return s;
    }

    void HandleRequest() {
        switch (currentRequest) {
            case Request::RECORD:
                sparse->Init();
                analyzer.Init(*sparse);
                if (pcmRing) pcmRing->Reset();
                ResetReader();
                currentState = State::RECORDING;
                break;

            case Request::LIVE_EFFECT:
                sparse->Init();
                analyzer.Init(*sparse);
                if (pcmRing) pcmRing->Reset();
                ResetReader();
                pendingSeed  = false;   // (the first grain waits for material: liveSeeded)
                liveSeeded   = false;
                seedFade     = 0.0f;
                currentState = State::LIVE_EFFECT;
                break;

            case Request::SLICE:
                if (!analyzer.IsEmpty()) pendingSeed = true;
                break;

            case Request::STOP_REC:
                analyzer.ArmFinalFrame();
                ResetReader();
                currentState = State::STOPPED;
                break;

            case Request::PLAY:
            case Request::RESTART_PLAYBACK:
                ResetReader();
                pendingSeed  = true;
                currentState = State::PLAYBACK;
                break;

            case Request::STOP_PLAYING:
                currentState = State::STOPPED;
                break;

            case Request::CLEAR:
                sparse->Init();
                analyzer.Init(*sparse);
                if (pcmRing) pcmRing->Reset();
                ResetReader();
                currentState = State::STOPPED;
                break;

            default:
                break;
        }
        currentRequest = Request::NONE;
    }

    void ProcessBlock(const float* in, float* out, size_t size) {
        lastBlockSize  = (int)size;
        pcmBlock       = pcmRing && (pcmOn || pcmW > 0.0f);
        firedThisBlock = false;
        if (currentRequest != Request::NONE) HandleRequest();

        switch (currentState) {
            case State::STOPPED: {
                for (size_t i = 0; i < size; i++) out[i] = in[i];
                break;
            }

            case State::RECORDING: {
                for (size_t i = 0; i < size; i++) {
                    if (analyzer.IsFull()) { SubmitRequest(Request::STOP_REC); out[i] = in[i]; continue; }
                    if (pcmRing) pcmRing->Write(in[i]);
                    analyzer.Analyze(in[i]);
                    out[i] = in[i];
                }
                break;
            }

            case State::PLAYBACK: {
                granule[0].BeginBlock((int)size);
                granule[1].BeginBlock((int)size);
                if (pendingSeed) { Punch(); pendingSeed = false; }
                TrackPitch();
                for (size_t i = 0; i < size; i++) out[i] = ReadPair();
                break;
            }

            case State::LIVE_EFFECT: {
                // Capture the whole block first so keyframes exist ahead of the
                // reader, then render behind them. A kept detector event fires
                // live: with delay pinned at 0 a punch seats its grain one fade
                // behind the transient, so the fade-in completes exactly as the
                // transient reaches the playhead — full gain at the hit.
                const uint32_t c0 = CAPICOLA_CYC();
                bool fire = false;
                for (size_t i = 0; i < size; i++) {
                    if (pcmRing) pcmRing->Write(in[i]);   // (in lockstep with the Analyzer: one sample each)
                    analyzer.Analyze(in[i]);
                    if (detector.Analyze(in[i])) fire = true;
                }

                const uint32_t c1 = CAPICOLA_CYC();
                capCycles = c1 - c0;
                sinceFire += (int64_t)size;
                // Refractory: expire first, then punch only from an expired state,
                // never re-arm on top of a running hold (else dense bursts latch).
                if (trigGate > 0.0f) trigGate -= (float)size;
                if (fire && trigGate <= 0.0f) {
                    transientCount++;
                    pendingSeed    = true;
                    pendingHit     = true;
                    sinceFire      = 0;
                    firedThisBlock = true;   // stereo-guard edge (natural fire only)
                    trigGate       = curFade;
                }

                granule[0].BeginBlock((int)size);
                granule[1].BeginBlock((int)size);
                // A new ramp only once the running one has landed: reusing a
                // half-faded grain would jump its gain (a click). A hit's own
                // punch is spaced by the refractory; a follow or a button
                // slice waits here instead.
                // The first grain waits until the ring holds a fade of input:
                // seated earlier, its tap (write head less the fade) clamps to
                // the start and it reads at the write head, past the newest
                // keyframe, garbage until the first hit re-seats it (1.2.0
                // too: a drone from power-on played full-scale jumps). Silence
                // until then, then the grain at the tap, faded in.
                if (!liveSeeded) {
                    pendingSeed = pendingHit = pendingFollow = false;
                    if (analyzer.WriteHead() >= (int64_t)curFade + 2 * (int64_t)size) {
                        granule[liveG].Retrigger();
                        granule[liveG].SetPitch(curPitch);
                        xGain      = 1.0f;
                        seedFade   = 0.0f;
                        liveSeeded = true;
                    }
                }
                if (xGain >= 1.0f) {
                    if (pendingFollow) {   // (the partner's grid: no punch of our own on top)
                        FollowPunch();
                        pendingFollow = pendingSeed = pendingHit = false;
                    } else if (pendingSeed) {
                        Punch(pendingHit);
                        pendingSeed = pendingHit = false;
                    }
                }
                TrackPitch();
                if (!liveSeeded) {
                    for (size_t i = 0; i < size; i++) out[i] = 0.0f;
                } else if (seedFade < 1.0f) {
                    const float step = 1.0f / ((curFade < 1.0f) ? 1.0f : curFade);
                    for (size_t i = 0; i < size; i++) {
                        out[i] = ReadPair() * SmoothStep(seedFade);
                        seedFade += step;
                        if (seedFade > 1.0f) seedFade = 1.0f;
                    }
                } else {
                    for (size_t i = 0; i < size; i++) out[i] = ReadPair();
                }

                // Gap-gated boundary keyframe. Safe because every reader fence
                // derives from the newest keyframe's time, and read heads sit
                // >= curFade behind it — so bounding coverage staleness by `gap`
                // keeps every legal read >= (curFade - gap) clear of the newest
                // frame. Half the fade is a comfortable gap; clamp so it degrades
                // to per-block cadence for degenerate tiny fades.
                float gap = 0.5f * curFade;
                const float cap = curFade - 2.0f * (float)size;
                if (gap > cap) gap = cap;
                if (gap < 1.0f) gap = 1.0f;
                analyzer.GuardKeyframe((double)gap);
                renderCycles = CAPICOLA_CYC() - c1;
                break;
            }
        }
    }
};

} // namespace capicola
