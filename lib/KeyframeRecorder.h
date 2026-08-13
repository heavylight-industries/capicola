#pragma once

#include <Analyzer.h>    // keyframe writer (raw ring + sparsifier)
#include <Granule.h>     // self-contained grain reader (own fences, self-healing)
#include <Detector.h>    // transient finder (TKEO + SVF + peak keyframer)
#include <Shapers.h>     // tabulated waveshapers (optional; Init(nullptr) = clean)
#include <cstddef>
#include <cstdint>

namespace capicola {

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
    SparseLine<bufsz> sparse;
    Analyzer<bufsz>   analyzer;

    // Two grains trading places over one gain ramp. A trigger always punches
    // through: the idle grain hard-seats at the delayed tap (Retrigger) and fades
    // in over curFade while the old grain finishes its in-flight splice at falling
    // gain. No internal fade is ever interrupted, so a punch can never click.
    Granule<bufsz> granule[2];
    int   liveG;     // index of the grain fading in / steady
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
    bool  firedThisBlock; // detector auto-fired this block (stereo-guard edge; NOT forced slices)
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
        granule[0].Init(sparse, 0.0);
        granule[1].Init(sparse, 0.0);
        liveG = 0;
        xGain = 1.0f;
        xStep = 0.0f;
        ApplyParams();
    }

    // Unconditional punch-through (button / transient). Swaps to the idle grain,
    // seats it at the tap, and starts the gain ramp. If a punch lands mid-ramp
    // the half-faded grain is reused and its gain jumps — rare given the fire-side
    // hold (>=50 ms) and curFade-length ramps.
    void Punch() {
        liveG ^= 1;
        granule[liveG].Retrigger();
        xGain = 0.0f;
        xStep = 1.0f / ((curFade < 1.0f) ? 1.0f : curFade);
    }

    // Mixed read of the trading pair; the retired grain is skipped once the ramp
    // completes.
    inline float ReadPair() {
        auto shape = [this](float v) { return Shape(v); };
        if (xGain >= 1.0f) return granule[liveG].Read(shape);
        const float in  = granule[liveG].Read(shape);
        const float out = granule[liveG ^ 1].Read(shape);
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

    void Init(const Shapers* sh = nullptr) {
        shapers   = sh;
        drive     = 1.0f;
        character = 0.5f;   // bypass
        sparse.Init();
        analyzer.Init(sparse);
        granule[0].Init(sparse, 0.0);
        granule[1].Init(sparse, 0.0);
        liveG = 0;
        xGain = 1.0f;
        xStep = 0.0f;
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
        firedThisBlock = false;
        lastBlockSize  = 0;
        trigGate       = 0.0f;
    }

    void SetGrainPitch(float s)   { curPitch = s; for (auto& g : granule) g.SetPitch(s); }
    void SetGrainStretch(float t) { curStretch = t; for (auto& g : granule) g.SetStretch(t); }
    void SetGrainLeash(int k)     { curLeash = k; for (auto& g : granule) g.SetLeash(k); }
    void SetThreshold(float v)    { curThreshold = v; analyzer.SetThreshold(v); }
    void SetGrainFade(float f)    { curFade = f; for (auto& g : granule) g.SetFade(f); }

    void SetTransientThreshold(float t) { detector.SetThreshold(t); }
    void SetTkeoCutoff(float fc)        { detector.SetCutoff(fc); }
    void SetTkeoResonance(float r)      { detector.SetResonance(r); }
    void SetDistortDrive(float d)       { drive = d; }
    void SetDistortCharacter(float c)   { character = c; }

    State GetState() { return currentState; }

    uint32_t TransientCount() const { return transientCount; }
    float    PairGain()       const { return xGain; }             // takeover ramp state
    float    TkeoEnvelope()   const { return detector.Envelope(); }
    bool     DetectorGate()   const { return detector.Gate(); }   // LIVE kept-event monostable

    // UI mirror of the LIVE trigger gate — the exact signal that punches the
    // engine, held for the refractory period.
    bool     GateActive()     const { return trigGate > 0.0f; }

    // Stereo re-alignment guardrail (see AudioEngine::ProcessBlock): did the
    // detector auto-fire this block (forced slices excluded), and how far behind
    // the writer the live grid sits right now.
    bool     FiredThisBlock() const { return firedThisBlock; }
    double   GridLag()        const { return granule[liveG].GridLag(); }

    void SubmitRequest(Request r) { currentRequest = r; }

    void HandleRequest() {
        switch (currentRequest) {
            case Request::RECORD:
                sparse.Init();
                analyzer.Init(sparse);
                ResetReader();
                currentState = State::RECORDING;
                break;

            case Request::LIVE_EFFECT:
                sparse.Init();
                analyzer.Init(sparse);
                ResetReader();
                pendingSeed  = true;
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
                sparse.Init();
                analyzer.Init(sparse);
                ResetReader();
                currentState = State::STOPPED;
                break;

            default:
                break;
        }
        currentRequest = Request::NONE;
    }

    inline void ProcessBlock(const float* in, float* out, size_t size) {
        lastBlockSize  = (int)size;
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
                    analyzer.Analyze(in[i]);
                    out[i] = in[i];
                }
                break;
            }

            case State::PLAYBACK: {
                granule[0].BeginBlock((int)size);
                granule[1].BeginBlock((int)size);
                if (pendingSeed) { Punch(); pendingSeed = false; }
                for (size_t i = 0; i < size; i++) out[i] = ReadPair();
                break;
            }

            case State::LIVE_EFFECT: {
                // Capture the whole block first so keyframes exist ahead of the
                // reader, then render behind them. A kept detector event fires
                // live: with delay pinned at 0 a punch seats its grain one fade
                // behind the transient, so the fade-in completes exactly as the
                // transient reaches the playhead — full gain at the hit.
                bool fire = false;
                for (size_t i = 0; i < size; i++) {
                    analyzer.Analyze(in[i]);
                    if (detector.Analyze(in[i])) fire = true;
                }

                // Refractory: expire first, then punch only from an expired state,
                // never re-arm on top of a running hold (else dense bursts latch).
                if (trigGate > 0.0f) trigGate -= (float)size;
                if (fire && trigGate <= 0.0f) {
                    transientCount++;
                    pendingSeed    = true;
                    firedThisBlock = true;   // stereo-guard edge (natural fire only)
                    trigGate       = curFade;
                }

                granule[0].BeginBlock((int)size);
                granule[1].BeginBlock((int)size);
                if (pendingSeed) { Punch(); pendingSeed = false; }
                for (size_t i = 0; i < size; i++) out[i] = ReadPair();

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
                break;
            }
        }
    }
};

} // namespace capicola
