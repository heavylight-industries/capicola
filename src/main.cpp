/**
 * @file main.cpp
 * @brief Capicola firmware — composition site on the alchemy-sdk surface layer.
 *
 * Perf-page knobs (ranges are the spec; the map lives in the VirtualKnob decls):
 *
 *   P1 Pitch      playback speed, bipolar ±2x; stop at noon, detents at ±1x
 *   P2 Stretch    bipolar +-1x; freeze at noon, |u|^2.5 taper each side
 *   P3 Threshold  keep ratio 0..8 over the envelope average (90% = 4, top = mute)
 *   P4 Grain Size 32 .. 4096 keyframes
 *   P5 Quality    analyzer ε 0.03 (crunchy) .. 0.001 (max)
 *   P6 Feedback   0 .. 1.5 — wet → sinc sat → SVF bandpass → back into the recorder
 *
 * Buttons (panel-final): B1 = PAGE, B2 = SLICE, B3 = PAGE.
 *
 * Two page groups, each rotated by a single press of its own button and each
 * remembering where it was left. All three buttons wear the visible page's hue;
 * B2 flashes OFF on a trigger.
 *
 *   B1  perf (orange) → mod depth (pink)
 *   B3  CV routing (blue) → secondary settings (teal)
 *
 * Pressing the other group's button jumps to that group's remembered page; the
 * next press of the same button advances within the group.
 *
 *   Depth:     pot i = bipolar depth (−1..+1) for pot i's modulation.
 *   Routing:   pot i = 3-zone source selector — input follower (orange, default)
 *              / output follower (blue) / CV IN (violet), zone-granular catch.
 *   Secondary: envelope smoothing, fade time, keyframe drive, drive
 *              character, MIX dry/wet, feedback bandpass — each with its own
 *              pot-catch.
 *
 * Routing zones and secondary norms both ride the preset store.
 *
 * B1+B2 held 0.33 s saves everything as the boot state (QSPI slot 0); B1+B3 held
 * 0.5 s resets the CURRENT view to factory defaults. Both confirm with a triple
 * dark blink. B2 alone is the ping-pong gesture, so the two never collide.
 *
 * B2 held is the PING-PONG gesture on P1/P2: the rate pins where it is and the
 * knob becomes a direction control — nudge it either way to flip polarity at
 * that same rate. Release and the knob is page-switched, inert behind a catch
 * pip until the pot is swept through it. The press still slices.
 *
 * B1+B3 held 0.5 s resets the CURRENT view to factory defaults, confirmed with
 * a triple dark blink.
 *
 *
 * Panel jacks (2×5, panel-final names, code indices in parens):
 *   INL   TRIG IN (cv[0])   TRIG 1·slice (jacks[2])   TRIG 2·out-event (jacks[4])   OUTL
 *   INR   CV IN   (cv[1])   ENV 1·in-env   (jacks[3])   ENV 2·out-env   (jacks[5])   OUTR
 * TRIG IN = external slice trigger (rising edge, always accepted). CV IN = the
 * single external mod CV (eurorack-bipolar). Trig outs are 10 ms 0/5 V pulses,
 * one per event, never closer than the detector's 150 ms hold-off; envelopes
 * are 0..1 → 0..5 V. Same physical channels as the old CV1..CV6 map — only the
 * roles/names changed.
 *
 * Fade (= latency = retrigger floor) is the secondary page's P2, boot 20 ms.
 * The SDK's ControlLoop owns polling, pot-catch (Pager), CV dispatch, and ring
 * rendering (each knob's declarative Ring).
 */

#include <cstring>

#include "alchemy/control/pot_catch.h"       /* PotState, InitCatch, UpdateCatch */
#include "alchemy/led/anims/fill.h"
#include "alchemy/led/ring_frame.h"
#include "alchemy/host_link/component_writer.h"
#include "alchemy/host_link/host.h"
#include "alchemy/hw/alchemy_lab_v2.h"
#include "alchemy/surface/control_loop.h"
#include "alchemy/surface/cv_source.h"
#include "alchemy/surface/page.h"
#include "alchemy/surface/pager.h"
#include "alchemy/surface/presets.h"
#include "alchemy/surface/virtual_knob.h"

#include "hardware.h"
#include "audio_engine.h"

using namespace alchemy;

#ifdef CAPICOLA_BENCH_LOG
/* Bench image: the 1 Hz CPU/CV serial log rides the front-panel USB
 * (LOGGER_EXTERNAL) INSTEAD of HostLink — the two share that CDC port.
 * (The Seed's onboard connector is buried once the module is racked.) */
using SerialLog = daisy::Logger<daisy::LOGGER_EXTERNAL>;
#endif

/* ── Palette ─────────────────────────────────────────────────────────── */

static constexpr LedPanel::Rgb kBlue   = {30, 110, 255};   // routing page / out-follower
static constexpr LedPanel::Rgb kIce    = {150, 210, 255};  // perf bipolar negative
static constexpr LedPanel::Rgb kOrange = {255, 110, 15};   // perf page / in-follower
static constexpr LedPanel::Rgb kViolet = {160, 40, 255};   // depth positive / ext CV
static constexpr LedPanel::Rgb kCenter = {90, 90, 90};     // bipolar zero
static constexpr LedPanel::Rgb kWhite  = {255, 255, 255};
static constexpr LedPanel::Rgb kPink   = {255, 60, 150};   // depth page / positive depth
static constexpr LedPanel::Rgb kTeal   = {0, 220, 200};    // secondary page

/* Mode color: worn by all three buttons, one hue per page. Activity flashes OFF.
 * B1 pages are warm, B3 pages cool. */
static constexpr LedPanel::Rgb kPageButton[2] = {
    kOrange,           // B1 page 0 — perf
    kPink,             // B1 page 1 — depth
};

static constexpr uint8_t kB3Routing   = 0;
static constexpr uint8_t kB3Secondary = 1;

static constexpr LedPanel::Rgb kB3PageButton[2] = {
    kBlue,             // B3 page 0 — CV routing
    kTeal,             // B3 page 1 — secondary settings
};

/* CV out levels. */
static constexpr float kTrigVolts   = 5.0f;
static constexpr float kEnvOutVolts = 5.0f;

/* Trigger outs. The jack gets a standard Eurorack trigger; the pip is held
 * longer so a hit always survives a 16 ms render frame. Both come off the same
 * edge, so the panel and the jack can never disagree about what fired. */
static constexpr uint32_t kTrigPulseMs = 10;
static constexpr uint32_t kTrigLedMs   = 60;

/* One trigger out + its 6pm pip. Fires either from a monotonic engine event
 * counter (Follow) or from a control-side event (Fire, for the forced slices).
 * A pulse is never retriggered while it is high: L and R punch off their own
 * detectors, so one stereo hit can straddle two audio blocks, and the second
 * edge belongs to the pulse already running rather than to a runt second
 * trigger. Consecutive events on one counter are >= 150 ms apart (the
 * detector's hold-off), so nothing musical is lost to that rule. */
class TrigOut
{
  public:
    /* Engine counter, sampled at 1 ms — far faster than the 150 ms hold-off, so
     * an edge can never be missed between polls. */
    void Follow(uint32_t count, uint32_t t_ms)
    {
        if (count == lastCount) return;
        lastCount = count;
        Fire(t_ms);
    }

    void Fire(uint32_t t_ms)
    {
        if (!High(t_ms)) startMs = t_ms ? t_ms : 1u;   // 0 is the "never fired" sentinel
    }

    /* 1 ms poll. The trig outs sit on the I2C MCP4728, so write only on edges. */
    void Poll(uint32_t t_ms, CvJack& jack)
    {
        const bool hi = High(t_ms);
        if (hi == wasHigh) return;
        wasHigh = hi;
        jack.SetVolts(hi ? kTrigVolts : 0.0f);
    }

    bool Lit(uint32_t t_ms) const
    {
        return startMs != 0u && (t_ms - startMs) < kTrigLedMs;
    }

  private:
    bool High(uint32_t t_ms) const
    {
        return startMs != 0u && (t_ms - startMs) < kTrigPulseMs;
    }

    uint32_t lastCount = 0;
    uint32_t startMs   = 0;   // 0 = never fired
    bool     wasHigh   = false;
};

static TrigOut trig1;   // TRIG 1 — every slice the engine takes
static TrigOut trig2;   // TRIG 2 — kept output-follower transients

/* ── Hardware / engine ───────────────────────────────────────────────── */

static AlchemyLabV2                 hw;
static AlchemyLabAudio::AudioEngine engine;

/* ── Mod routing (replaces CvMatrix) ─────────────────────────────────────
 * Each perf pot picks its modulation source — input follower, output follower,
 * or the CV IN jack — and takes its bipolar depth from the depth view. A
 * CvSource because two of the three sources are internal engine signals. */
class ModRouter : public CvSource
{
  public:
    static constexpr uint8_t kSrcInEnv  = 0;
    static constexpr uint8_t kSrcOutEnv = 1;
    static constexpr uint8_t kSrcCv     = 2;   // panel CV IN (cv[1])

    void SetDepth (uint8_t pot, float d)   { if (pot < NUM_POTS) depth[pot] = d; }
    void SetSource(uint8_t pot, uint8_t s) { if (pot < NUM_POTS) src[pot]   = s; }

    float DeltaAtPage(uint8_t page, uint8_t pot) const override
    {
        return (page == 0 && pot < NUM_POTS) ? delta[pot] : 0.0f;
    }

    void Update(const float* cv, uint32_t) override
    {
        const float sigCv  = (cv[1] - 0.5f) * 2.0f;   // panel CV IN, bipolar
        const float sigIn  = engine.EnvNormIn();      // unipolar 0..1
        const float sigOut = engine.EnvNormOut();
        for (uint8_t p = 0; p < NUM_POTS; p++)
        {
            const float s = (src[p] == kSrcInEnv)  ? sigIn
                          : (src[p] == kSrcOutEnv) ? sigOut
                                                   : sigCv;
            delta[p] = depth[p] * s;
        }
    }

  private:
    float   depth[NUM_POTS] = {};
    uint8_t src  [NUM_POTS] = {};   // defaults to input follower
    float   delta[NUM_POTS] = {};
};

static ModRouter modRouter;

/* ── Page 0 — performance knobs ──────────────────────────────────────── */

static constexpr float kPitchMax    = 2.0f;    // speed at either stop
static constexpr float kPitchDetent = 0.03f;   // ±0.03x holds each unity

/* ── Ping-pong gesture, shared by Pitch and Stretch ───────────────────────
 * Hold B2 (or a high TRIG IN gate) and the knob stops being a value and becomes
 * a DIRECTION control: the magnitude pins where it was, and the MOMENTUM of the
 * knob sets polarity — nudge it toward negative and playback flips to reverse at
 * that same rate, nudge back toward positive and it flips forward. Release and
 * the knob is page-switched: inert, showing a catch pip, until the pot is swept
 * through the value the gesture left behind. Same PotState the Pager uses, so
 * the pip means exactly what it means everywhere else.
 *
 * Momentum is measured on the MODULATED value, so a CV riding the knob flips
 * polarity too — an LFO into P1 while B2 is held is automatic ping-pong at the
 * LFO's rate.
 *
 * B2 is the only hold source. A sustained TRIG IN gate was tried as a second
 * one and taken out: a long gate then made both knobs inert for value, which is
 * the wrong thing to happen to a patch that just uses long gates.
 *
 * One struct per knob; the magnitude map and its inverse are passed in, since
 * Pitch and Stretch taper differently. */
static constexpr float kGestureNudge = 0.015f;   // 1.5% of sweep counts as a move

struct GestureKnob
{
    PotState catchState;
    bool     b2Prev      = false;
    bool     latchRev    = false;   // polarity the momentum has latched
    float    ref         = 0.75f;   // pot position the nudge is measured from
    float    frozenMag   = 1.0f;
    float    liveMag     = 1.0f;    // magnitude to play this frame
    float    heldNorm    = 0.75f;   // live gesture value; becomes the catch target
    float    displayNorm = 0.75f;   // what the arc draws
};

static GestureKnob pitchG, stretchG;

/* Returns the effective norm to use this frame. `magFn` maps norm -> magnitude,
 * `normFn` inverts it back onto a chosen side. */
static float GestureNorm(GestureKnob& g, float n, float phys, bool held,
                         float (*magFn)(float), float (*normFn)(float, bool))
{
    float eff;
    if (held)
    {
        if (!g.b2Prev)                       // gesture opens: pin rate and side
        {
            /* From what is PLAYING, not from the knob. After a release while
             * flipped the pot still sits on the old side, so seeding from it
             * would snap polarity back the instant B2 is pressed again.
             * displayNorm is last frame's effective value — exactly right
             * whether the knob is caught or still decoupled. */
            const float cur = g.displayNorm;
            g.frozenMag = magFn(cur);
            g.latchRev  = (cur < 0.5f);
            g.ref       = n;                 // momentum is still measured on the knob
        }
        if (n < g.ref - kGestureNudge)        { g.latchRev = true;  g.ref = n; }
        else if (n > g.ref + kGestureNudge)   { g.latchRev = false; g.ref = n; }
        g.heldNorm = normFn(g.frozenMag, g.latchRev);
        g.liveMag  = g.frozenMag;        // exact — no norm round trip
        eff = g.heldNorm;
    }
    else
    {
        if (g.b2Prev)                       // just released: decouple
        {
            g.catchState.stored = g.heldNorm;
            InitCatch(g.catchState, phys);
        }
        UpdateCatch(g.catchState, phys);
        eff = g.catchState.stored + (n - phys);   // modulation still rides on top
        if (eff < 0.0f) eff = 0.0f; else if (eff > 1.0f) eff = 1.0f;
        g.liveMag = magFn(eff);
    }
    g.b2Prev      = held;
    g.displayNorm = eff;
    return eff;
}

/* Bipolar ring drawn from the gesture's value rather than the pot, so a latched
 * knob shows what it is actually playing: the arc keeps its length and jumps
 * across noon the instant the pot crosses. Everything else is the SDK's own
 * rendering — FillMode::Center is the same bipolar fill ArcStyle::Bipolar uses
 * (including its LED-step snapping), and the Emit overload paints the standard
 * catch pip on top, so a decoupled P1/P2 reads exactly like a decoupled knob
 * anywhere else. PerfRenderer skips Custom rings entirely, which is why the pip
 * has to be asked for here. */
static void DrawGestureRing(LedPanel& panel, uint8_t pot, const ArcGeometry& arc,
                            float, uint32_t t_ms, void* ctx)
{
    const GestureKnob& g = *static_cast<const GestureKnob*>(ctx);

    /* PerfRenderer dims an uncaught arc to 30%; a Custom ring has to do that
     * itself or a decoupled P1/P2 would look live when every other knob dims. */
    const float k = g.catchState.caught ? 1.0f : 0.30f;

    FillDesc desc;
    desc.mode         = FillMode::Center;
    desc.color        = LedPanel::Scale(kOrange, k);   // forward
    desc.neg_color    = LedPanel::Scale(kBlue,   k);   // reverse
    desc.center_color = LedPanel::Scale(kCenter, k);

    RingFrame f;
    f.Begin(arc);
    f.Base(desc, g.displayNorm, t_ms);
    f.Emit(panel, pot, g.catchState);
}

VirtualKnob pitch = VirtualKnob(0, "Pitch")
    .Ident("perf.pitch")
    .Unit("x")
    .Linear(-kPitchMax, +kPitchMax)
    .Ring(Custom(DrawGestureRing, &pitchG));   // gesture-aware bipolar arc

/* Pitch is playback speed, linear: noon stops the head, the stops are ±2x,
 * unity sits at ¼ and ¾. Below noon the grains play backward. A detent holds
 * each unity; noon has none — near-zero is just slow. */
static float PitchMag(float n)
{
    if (n < 0.0f) n = 0.0f; else if (n > 1.0f) n = 1.0f;
    const float m = std::fabs(2.0f * n - 1.0f) * kPitchMax;
    return (std::fabs(m - 1.0f) < kPitchDetent) ? 1.0f : m;
}

static float PitchNormFor(float mag, bool rev)
{
    float a = mag / kPitchMax;
    if (a > 1.0f) a = 1.0f;
    return rev ? (0.5f - 0.5f * a) : (0.5f + 0.5f * a);
}

VirtualKnob stretch = VirtualKnob(1, "Stretch")
    .Ident("perf.stretch")
    .Linear(0.0f, 1.0f)
    .Ring(Custom(DrawGestureRing, &stretchG));  // gesture-aware bipolar arc

/* Stretch taper, bipolar: a TRUE FREEZE at noon, realtime forward at the CW
 * stop, realtime reverse at CCW. |u|^2.5 keeps the fine resolution around
 * freeze that the old unipolar (1-n)^2.5 gave, mirrored onto both sides — so
 * n = 0.75 still lands ~5.7x, exactly where noon used to sit.
 *
 * No detent: 1% off noon is already 1e-5, half a sample per second. Freeze is
 * an asymptote, not a hard target. */
static float StretchMag(float n)
{
    if (n < 0.0f) n = 0.0f; else if (n > 1.0f) n = 1.0f;
    const float u = 2.0f * n - 1.0f;                // -1 .. +1, zero at noon
    return std::pow(std::fabs(u), 2.5f);            // 0 at noon, 1 at the stops
}

/* Inverse of StretchMag onto a chosen side. The magnitude legitimately reaches
 * zero (a true freeze), and pow handles it. */
static float StretchNormFor(float mag, bool rev)
{
    float a = std::pow(mag, 1.0f / 2.5f);
    if (a < 0.0f) a = 0.0f; else if (a > 1.0f) a = 1.0f;
    return rev ? (0.5f - 0.5f * a) : (0.5f + 0.5f * a);
}

VirtualKnob threshold = VirtualKnob(2, "Threshold")
    .Ident("perf.threshold")
    .Linear(0.0f, 1.0f)
    .Ring(Level(kOrange));

VirtualKnob grainSize = VirtualKnob(3, "Grain Size")
    .Ident("perf.grain")
    .Linear(32.f, 4096.f)
    .Ring(Level(kOrange));

VirtualKnob quality = VirtualKnob(4, "Quality")
    .Ident("perf.quality")
    /* ε 0.03..0.001. The old CCW end was 0.1, which measured ~2.5 dB
     * reconstruction SNR on speech — past crunchy into broken, and because the
     * map is linear in ε it ate half the sweep. 0.03 measures ~13.3 dB against
     * 17.1 at the CW end, so the whole travel is now usable and reads roughly
     * evenly in dB. */
    .Linear(0.03f, 0.001f)  // analyzer ε — CW = max fidelity, CCW = crunchy
    /* The host would otherwise render the raw epsilon as a backwards
     * "0.1 .. 0.001" sweep; fidelity percent is what the knob means. */
    .Disp("{\"kind\":\"norm\"}")
    .Ring(Level(kOrange));

VirtualKnob feedback = VirtualKnob(5, "Feedback")
    .Ident("perf.feedback")
    .Linear(0.f, 1.5f)
    .Ring(Level(kOrange, FillAnim::Pulse));

/* ── Page 1 — mod depth view (B2) ────────────────────────────────────── */

/* At noon the bipolar arcs read empty — a faint page-hued glow keeps the page
 * identity visible. Skips the center LED so the zero pip stays crisp. */
static void DrawDepthGlow(LedPanel& panel, uint8_t pot, const ArcGeometry&,
                          float norm, uint32_t, void*)
{
    if (norm < 0.44f || norm > 0.56f)
        return;
    for (int s = 0; s <= 12; s++)
    {
        if (s == 6) continue;
        panel.SetRingByHour(pot, 7.5f + 0.75f * (float)s,
            panel.ScaleGlobal(LedPanel::Scale(kPink, 0.10f)));
    }
}

VirtualKnob cvDepth1 = VirtualKnob(0, "Pitch Mod")
    .Ident("depth.p1")
    .Linear(-1.f, +1.f).Ring(Bipolar(kPink, kOrange, kCenter)).Overdraw(DrawDepthGlow);
VirtualKnob cvDepth2 = VirtualKnob(1, "Stretch Mod")
    .Ident("depth.p2")
    .Linear(-1.f, +1.f).Ring(Bipolar(kPink, kOrange, kCenter)).Overdraw(DrawDepthGlow);
VirtualKnob cvDepth3 = VirtualKnob(2, "Threshold Mod")
    .Ident("depth.p3")
    .Linear(-1.f, +1.f).Ring(Bipolar(kPink, kOrange, kCenter)).Overdraw(DrawDepthGlow);
VirtualKnob cvDepth4 = VirtualKnob(3, "Grain Size Mod")
    .Ident("depth.p4")
    .Linear(-1.f, +1.f).Ring(Bipolar(kPink, kOrange, kCenter)).Overdraw(DrawDepthGlow);
VirtualKnob cvDepth5 = VirtualKnob(4, "Quality Mod")
    .Ident("depth.p5")
    .Linear(-1.f, +1.f).Ring(Bipolar(kPink, kOrange, kCenter)).Overdraw(DrawDepthGlow);
VirtualKnob cvDepth6 = VirtualKnob(5, "Feedback Mod")
    .Ident("depth.p6")
    .Linear(-1.f, +1.f).Ring(Bipolar(kPink, kOrange, kCenter)).Overdraw(DrawDepthGlow);

/* ── Mod routing view (B3 double-tap; not a pager page) ──────────────────
 * Three-zone source selector per pot. The ring shows all three source colors
 * as thirds (input follower / output follower / CV IN, CCW→CW) with the
 * selected third at full intensity; the dim thirds double as the position
 * map. No catch pip — but catch is zone-granular: a pot decouples on entry
 * and re-locks the moment it sits inside its stored zone. */

static constexpr uint8_t kNumSrcZones = 3;
static constexpr LedPanel::Rgb kSrcColors[kNumSrcZones] = {kOrange, kBlue,
                                                           kViolet};

static int ZoneOf(float v)
{
    const int z = (int)(v * (float)kNumSrcZones);
    return (z > kNumSrcZones - 1) ? kNumSrcZones - 1 : (z < 0) ? 0 : z;
}

#ifndef CAPICOLA_BENCH_LOG
/* Routing field identity and prose — defined in capicola_manual.cpp so every
 * manual string lives in one file. */
extern const char* const kRouteFieldId  [NUM_POTS];
extern const char* const kRouteFieldName[NUM_POTS];
extern const char* const kRouteFieldHelp[NUM_POTS];
extern const VirtualKnob* const kRouteSeeKnob[NUM_POTS];
extern const char* const kRouteDispJson;
#endif

/* Persisted zone per perf pot — rides the preset store as its own component
 * now that routing left the Pager. */
class RoutingStore : public Serializable
{
  public:
    uint8_t zone[NUM_POTS] = {0, 0, 0, 0, 0, 0};   // 0 = input follower (factory default)

    size_t SerializedSize() const override { return NUM_POTS; }
    void   Serialize(uint8_t* out) const override
    {
        for (uint8_t p = 0; p < NUM_POTS; p++) out[p] = zone[p];
    }
    bool Deserialize(const uint8_t* in) override
    {
        for (uint8_t p = 0; p < NUM_POTS; p++)
            zone[p] = (in[p] < kNumSrcZones) ? in[p]
                                             : (uint8_t)(kNumSrcZones - 1);
        return true;
    }
    uint32_t SchemaHash() const override
    {
        return 0x43563034u ^ (uint32_t)NUM_POTS
                           ^ ((uint32_t)kNumSrcZones << 8);
    }

#ifndef CAPICOLA_BENCH_LOG
    /* Self-describe to HostLink so the routing page is editable and
     * documented on the web programmer instead of an opaque blob. One byte
     * per pot maps straight onto an enum field — no storage change, so the
     * schema hash (and every saved preset) is untouched. Prose and labels
     * live in capicola_manual.cpp with the rest of the manual. */
    bool Describe(hostlink::ComponentWriter& w) const override
    {
        static const char* const kRoutingPage[1] = {"Mod Routing"};
        w.Ident("routing").Kind("settings").Label("Mod Routing").Grid(1, NUM_POTS);
        w.PageMeta(kRoutingPage, nullptr, 1);
        for (uint8_t p = 0; p < NUM_POTS; p++)
        {
            const SeeRef see[1] = {SeeRef(*kRouteSeeKnob[p])};
            if (!w.Field(kRouteFieldId[p], kRouteFieldName[p], p,
                         hostlink::FieldType::Enum, 0.0f /* in follower */,
                         kRouteDispJson, kNumSrcZones, 0, (int16_t)p,
                         kRouteFieldHelp[p], see, 1u))
                return false;
        }
        return true;
    }
#endif
};

static RoutingStore routing;
static bool         routeCaught[NUM_POTS] = {};

/* B3 page group: latched, remembers its page independently of B1. */
static bool    b3Active = false;
static uint8_t b3Page   = kB3Routing;

/* B1 page held frozen while a B3 page is up. */
static uint8_t underPage = 0;
static float   underNorm[NUM_POTS] = {};

static inline bool RoutingOpen()   { return b3Active && b3Page == kB3Routing; }
static inline bool SecondaryOpen() { return b3Active && b3Page == kB3Secondary; }

static void DrawRouteRing(uint8_t pot, uint8_t sel)
{
    /* Orange gets a higher dim floor: its low G/B channels quantize away and it
     * reads dimmer than its neighbors at equal scale. */
    static constexpr float kZoneDim[3] = {0.12f, 0.06f, 0.06f};
    for (int s = 0; s <= 12; s++)
    {
        const int   zone = (s < 4) ? 0 : (s < 9) ? 1 : 2;   // 4|5|4 LEDs
        const float k    = (zone == sel) ? 1.0f : kZoneDim[zone];
        hw.leds.SetRingByHour(pot, 7.5f + 0.75f * (float)s,
            hw.leds.ScaleGlobal(LedPanel::Scale(kSrcColors[zone], k)));
    }
}

Page perfPage  = Page(0).Name("Primary").Knobs(pitch, stretch, threshold,
                                      grainSize, quality, feedback);
Page depthPage = Page(1).Name("Depth").Knobs(cvDepth1, cvDepth2, cvDepth3,
                                      cvDepth4, cvDepth5, cvDepth6);

/* ── Surfaces ────────────────────────────────────────────────────────── */

static ControlLoop loop  (hw);
static Pager       pager (hw.buttons[0], 2, NUM_POTS);   // B1: perf ↔ depth

/* Startup settings = preset slot 0. The Pager is the only managed component —
 * its stored knob values are the module's settings. BootLoad() at startup;
 * hold B1+B2 kSaveHoldMs to overwrite (see OnPoll). */
static Presets presets(hw.seed.qspi);

#ifndef CAPICOLA_BENCH_LOG
/* Browser/CLI preset editor over the front-panel USB CDC (SDK default
 * transport). Identity strings are literals — kept by reference. */
hostlink::Host host(presets, "capicola", "Capicola",
                           "1.0.0", CAPICOLA_GIT_HASH);

/* src/capicola_manual.cpp — attaches the firmware-authored manual (prose,
 * jacks, buttons) to the descriptor. Called before BootLoad() brings USB up. */
void DescribeManual(hostlink::Host& host);
#endif

static constexpr uint32_t kSaveHoldMs  = 330;
static constexpr uint32_t kResetHoldMs = 500;    // B1+B3: reset current page
static constexpr uint32_t kSaveFlashMs  = 600;   // 3 dark pulses at kSaveBlinkMs
static constexpr uint32_t kSaveBlinkMs  = 100;
static uint32_t saveFlashMs = 0;   // 0 = never; set on save/reset commit


/* Factory defaults (stored norms) — seeded at boot AND restored per-page by the
 * B1+B3 hold. One table so the two can never drift apart. (Routing zones default
 * to the input follower in RoutingStore's own initializer.) */
static constexpr float kPageDefaults[2][NUM_POTS] = {
    /* perf: pitch unity forward, stretch ~6x forward, threshold mid, grain,
     * quality max, feedback off. */
    {0.75f, 0.75f, 0.5f, 0.3f, 1.0f, 0.0f},
    /* depths: noon = off */
    {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f},
};

/* ── Secondary settings (B3 page 1) ──────────────────────────────────────
 *   P1  Smoothing  envelope fc  Exp 5e-5..0.125                    default 0.0014
 *   P2  Fade       OLA crossfade  Exp 10..250 ms                   default 20 ms
 *   P3  Drive      keyframe shaper gain   Linear 0.5..4.0         default 1.0
 *   P4  Character  quake ← clean → sinc   Linear 0..1             default 1 (sinc)
 *   P5  Mix        dry/wet      Linear 0..1                        default 1 (wet)
 *   P6  FB band    feedback bandpass fc   Exp 2e-3..0.9           default 0.02
 */
static PotState secState[NUM_POTS];

static constexpr bool kSecEnabled[NUM_POTS] = {true, true, true,
                                               true, true, true};

/* Default secondary norms (each engine boot value expressed on its sweep). */
static constexpr float kSecDefault[NUM_POTS] = {
    0.4259f,   // P1 smoothing → fc 0.0014
    0.2153f,   // P2 fade      → 960 samples (20 ms)
    0.1429f,   // P3 drive     → 1.0
    1.0f,      // P4 character → full sinc
    1.0f,      // P5 mix       → full wet (classic Capicola)
    0.3769f,   // P6 fb band   → fc 0.02
};

/* Secondary engineering value from a caught norm. */
static float SecValue(uint8_t pot, float n)
{
    switch (pot)
    {
        case 0: return 5.0e-5f * std::pow(0.125f / 5.0e-5f, n);  // smoothing fc (Exp)
        case 1: return 480.0f * std::pow(12000.0f / 480.0f, n);  // fade samples (Exp)
        case 2: return 0.5f + 3.5f * n;                          // drive (Linear)
        case 3: return n;                                       // character (Linear)
        case 4: return n;                                       // mix (Linear)
        case 5: return 2.0e-3f * std::pow(0.9f / 2.0e-3f, n);   // fb band fc (Exp)
        default: return 0.0f;
    }
}

/* Push a caught secondary value into the engine. */
static void SecApply(uint8_t pot, float v)
{
    switch (pot)
    {
        case 0: engine.SetTkeoCutoff(v);     break;
        case 1: engine.SetFade(v);           break;
        case 2: engine.SetDrive(v);          break;
        case 3: engine.SetDriveCharacter(v); break;
        case 4: engine.SetMix(v);            break;
        case 5: engine.SetFeedbackCutoff(v); break;
        default: break;
    }
}

#ifndef CAPICOLA_BENCH_LOG
/* Secondary field identity and prose — defined in capicola_manual.cpp. */
extern const char* const kSecFieldId  [NUM_POTS];
extern const char* const kSecFieldName[NUM_POTS];
extern const char* const kSecFieldHelp[NUM_POTS];
extern const char* const kSecFieldDisp[NUM_POTS];
#endif

/* Secondary norms ride the preset store.
 *
 * Stored as pot-normalized f32 — the one type HostLink's descriptor can
 * address (protocol §5: "f32" is a 4-byte LE float, natural range [0,1]).
 * The earlier u16 packing halved the bytes but made the whole page opaque to
 * the web programmer; twelve extra preset bytes buys six live, documented
 * controls. */
class SecondaryStore : public Serializable
{
  public:
    size_t SerializedSize() const override { return NUM_POTS * sizeof(float); }
    void   Serialize(uint8_t* out) const override
    {
        for (uint8_t p = 0; p < NUM_POTS; p++)
        {
            float n = secState[p].stored;
            if (n < 0.0f) n = 0.0f; else if (n > 1.0f) n = 1.0f;
            std::memcpy(out + p * sizeof(float), &n, sizeof(float));
        }
    }
    bool Deserialize(const uint8_t* in) override
    {
        for (uint8_t p = 0; p < NUM_POTS; p++)
        {
            float n;
            std::memcpy(&n, in + p * sizeof(float), sizeof(float));
            /* Written by anyone on the wire — clamp, and let NaN fall to 0
             * (the !(n >= 0) test catches it; n > 1 never fires on NaN). */
            if (!(n >= 0.0f))     n = 0.0f;
            else if (n > 1.0f)    n = 1.0f;
            secState[p].stored = n;
        }
        return true;
    }
    uint32_t SchemaHash() const override
    {
        /* Bumped from ...35: the payload changed from u16 to f32. */
        return 0x53454336u ^ (uint32_t)NUM_POTS;
    }

#ifndef CAPICOLA_BENCH_LOG
    /* Self-describe so the secondary page is editable and documented on the
     * web programmer. OnFrame pushes any changed norm into the engine, so a
     * live write from the host takes effect on the next frame. */
    bool Describe(hostlink::ComponentWriter& w) const override
    {
        static const char* const kSecondaryPage[1] = {"Secondary"};
        w.Ident("secondary").Kind("settings").Label("Secondary").Grid(1, NUM_POTS);
        w.PageMeta(kSecondaryPage, nullptr, 1);
        for (uint8_t p = 0; p < NUM_POTS; p++)
            if (!w.Field(kSecFieldId[p], kSecFieldName[p],
                         p * (uint32_t)sizeof(float),
                         hostlink::FieldType::F32, kSecDefault[p],
                         kSecFieldDisp[p], 0u, 0, (int16_t)p,
                         kSecFieldHelp[p]))
                return false;
        return true;
    }
#endif
};

static SecondaryStore secondary;

/* Secondary ring: a level arc (dim while uncaught) plus a bright catch pip. */
static void DrawShiftArc(uint8_t pot, float level, bool caught, uint32_t t_ms)
{
    FillDesc d;
    d.mode          = FillMode::Edge;
    d.color         = LedPanel::Scale(kTeal, caught ? 0.9f : 0.4f);
    d.passive_color = LedPanel::Scale(kTeal, 0.04f);
    DrawFill(hw.leds, pot, 7.5f, 0.75f, 13, level, d, t_ms);
    if (!caught)
        hw.leds.SetRingByHour(pot, 7.5f + 9.0f * level,
            hw.leds.ScaleGlobal(LedPanel::Mix(kTeal, kWhite, 0.7f)));
}

/* Every ring on the secondary page, so it reads the same over either B1 page.
 * Unwired pots sit dark. */
static void DrawSecondaryRings(uint32_t t_ms)
{
    const LedPanel::Rgb off = hw.leds.ScaleGlobal(LedPanel::Scale(kTeal, 0.04f));
    for (uint8_t p = 0; p < NUM_POTS; p++)
    {
        if (kSecEnabled[p])
            DrawShiftArc(p, secState[p].stored, secState[p].caught, t_ms);
        else
            for (int s = 0; s <= 12; s++)
                hw.leds.SetRingByHour(p, 7.5f + 0.75f * (float)s, off);
    }
}

/* ── Frame hooks ─────────────────────────────────────────────────────── */

/* B3 press bookkeeping, written by OnPoll at 1 ms cadence and read by OnFrame /
 * OnRender. */
static bool b1Chorded = false;   // another button joined while down → not a page press
static bool b3Chorded = false;

/* Knob values → engine, routing zones → mod sources. */
static void OnFrame()
{
    /* B3 view changes: arm the incoming page's catch, re-arm B1's on the way out. */
    {
        static bool    prevActive = false;
        static uint8_t prevPage   = kB3Routing;
        const float*   ph         = loop.Phys();

        /* Pager::Update runs before this hook and tracks the caught pot, so the
         * B1 page underneath follows the knob unless we hold it. Snapshot on
         * entry, re-assert every frame; the pot locks out once it moves past
         * kCatchTolerance. */
        if (b3Active && !prevActive)
        {
            underPage = pager.Page();
            for (uint8_t p = 0; p < NUM_POTS; p++)
                underNorm[p] = pager.Stored(underPage, p);
        }
        if (b3Active)
            for (uint8_t p = 0; p < NUM_POTS; p++)
                if (pager.Stored(underPage, p) != underNorm[p])
                    pager.SetStored(underPage, p, underNorm[p], ph);

        if (b3Active != prevActive || (b3Active && b3Page != prevPage))
        {
            if (RoutingOpen())
                for (uint8_t p = 0; p < NUM_POTS; p++)
                    routeCaught[p] = false;
            else if (SecondaryOpen())
            {
                for (uint8_t p = 0; p < NUM_POTS; p++)
                    if (kSecEnabled[p]) InitCatch(secState[p], ph[p]);
            }
            else
                pager.LockPage(pager.Page(), ph);
        }

        if (SecondaryOpen())
            for (uint8_t p = 0; p < NUM_POTS; p++)
                if (kSecEnabled[p]) UpdateCatch(secState[p], ph[p]);

        /* Push every changed secondary, whoever moved it — a caught pot, a
         * preset load, the B1+B3 reset, or a HostLink live write. Gating this
         * on SecondaryOpen() used to strand anything the panel didn't move.
         * Seeded out of range so the first frame always pushes. */
        static float applied[NUM_POTS] = {-1.f, -1.f, -1.f, -1.f, -1.f, -1.f};
        for (uint8_t p = 0; p < NUM_POTS; p++)
        {
            if (!kSecEnabled[p] || secState[p].stored == applied[p]) continue;
            applied[p] = secState[p].stored;
            SecApply(p, SecValue(p, secState[p].stored));
        }

        prevActive = b3Active;
        prevPage   = b3Page;
    }

    /* Pitch carries the same B2 gesture as Stretch. Value() is the modulated
     * speed; the gesture works in norms, and the map is linear, so convert. */
    {
        const bool  held = hw.buttons[1].Pressed() && !hw.buttons[0].Pressed();
        const float n    = (pitch.Value() + kPitchMax) / (2.0f * kPitchMax);
        const float eff  = GestureNorm(pitchG, n, loop.Phys()[0],
                                       held, PitchMag, PitchNormFor);
        engine.SetSlicePitch((eff < 0.5f) ? -pitchG.liveMag : pitchG.liveMag);
    }

    /* Secondary page open: skip the primaries so the two layers don't fight. */
    if (!SecondaryOpen())
    {
        /* P2 carries the same B2 gesture as P1: hold to flip the grid's
         * direction at a constant stretch, release and the knob is page-
         * switched until its catch pip is consumed. */
        {
            const bool  held = hw.buttons[1].Pressed() && !hw.buttons[0].Pressed();
            const float eff  = GestureNorm(stretchG, stretch.Value(), loop.Phys()[1],
                                           held, StretchMag, StretchNormFor);
            engine.SetSliceStretch((eff < 0.5f) ? -stretchG.liveMag : stretchG.liveMag);
        }
        /* P3 piecewise: 0..90% of the sweep is ratio 0..4 over the envelope
         * average, the last 10% climbs 4..8; the very top hard-mutes auto
         * triggers (sentinel trips Detector::kMuteAt). B1 still works. */
        const float tn    = threshold.Value();
        const float ratio = (tn <= 0.9f) ? (tn * (4.0f / 0.9f))
                                         : (4.0f + (tn - 0.9f) * 40.0f);
        engine.SetTransientThreshold(tn > 0.99f ? 1.0e9f : ratio);
        engine.SetGrainSize(grainSize.Value());   // keyframes
        engine.SetQuality  (quality.Value());      // analyzer ε
        engine.SetFeedback (feedback.Value());     // filtered feedback
    }

    modRouter.SetDepth(0, cvDepth1.Value());
    modRouter.SetDepth(1, cvDepth2.Value());
    modRouter.SetDepth(2, cvDepth3.Value());
    modRouter.SetDepth(3, cvDepth4.Value());
    modRouter.SetDepth(4, cvDepth5.Value());
    modRouter.SetDepth(5, cvDepth6.Value());

    /* Routing view open: pots edit the stored zones with zone-granular catch —
     * a pot decouples on entry and re-locks the moment it sits inside its
     * stored zone. The pager page underneath is re-locked every frame so those
     * primaries can't catch while their pots are steering zones. */
    if (RoutingOpen())
    {
        const float* ph = loop.Phys();
        for (uint8_t p = 0; p < NUM_POTS; p++)
        {
            const uint8_t z = (uint8_t)ZoneOf(ph[p]);
            if (!routeCaught[p] && z == routing.zone[p])
                routeCaught[p] = true;
            if (routeCaught[p])
                routing.zone[p] = z;
        }
    }

    for (uint8_t p = 0; p < NUM_POTS; p++)
        modRouter.SetSource(p, routing.zone[p]);

#ifdef CAPICOLA_BENCH_LOG
    /* ~1 Hz audio-CPU + raw-CV report over USB serial (cv in milliunits, 500 ≈
     * 0 V — bench probe for the jack map). Bench image only; the deploy image
     * gives this port to HostLink. */
    static uint32_t lastCpuLogMs = 0;
    const uint32_t  t = daisy::System::GetNow();
    if (t - lastCpuLogMs >= 1000)
    {
        lastCpuLogMs = t;
        const int avg = (int)(engine.CpuAvg() * 1000.0f + 0.5f);
        const int mx  = (int)(engine.CpuMax() * 1000.0f + 0.5f);
        const float* cvb = loop.Cv();
        SerialLog::PrintLine("CPU avg=%d.%d%% max=%d.%d%%  cv %d %d %d %d %d %d",
                          avg / 10, avg % 10, mx / 10, mx % 10,
                          (int)(cvb[0] * 1000), (int)(cvb[1] * 1000),
                          (int)(cvb[2] * 1000), (int)(cvb[3] * 1000),
                          (int)(cvb[4] * 1000), (int)(cvb[5] * 1000));
    }
#endif
}

/* 1 ms poll: B2 force trigger, B3 tap/hold tracking + the four CV outs. Trig
 * outs sit on the MCP4728 (I²C) so they write only on pulse edges; envelope
 * outs sit on the STM DAC and stream every poll. */
static uint32_t sliceFlashMs = 0;   // last button-originated trigger

static void OnPoll(uint32_t t_ms)
{
    /* B1+B2 held kSaveHoldMs = save everything (all four pages) as the boot
     * state, QSPI slot 0. Mirrors B1+B3 = reset, and leaves B2 alone to the
     * ping-pong gesture — the two are mutually exclusive, so a save can never
     * flip a polarity on the way past. B2's press still slices going down. */
    static uint32_t saveHoldMs = 0;
    if (hw.buttons[0].Pressed() && hw.buttons[1].Pressed()
        && !hw.buttons[2].Pressed())
    {
        pager.ConsumeButton();   // B1 is the pager now — eat its release
        if (++saveHoldMs == kSaveHoldMs)
        {
            presets.Save(0);
            saveFlashMs = t_ms;   // LED confirm (see OnRender)
        }
    }
    else
        saveHoldMs = 0;

    /* B1+B3 held kResetHoldMs = reset the CURRENT view's knobs to factory
     * defaults (routing zones, if the routing view is open; stored values
     * only — the QSPI save is untouched). Same triple blink confirms. */
    static uint32_t resetHoldMs = 0;
    if (hw.buttons[0].Pressed() && hw.buttons[2].Pressed()
        && !hw.buttons[1].Pressed())
    {
        pager.ConsumeButton();   // B1 is the pager now — eat its release
        if (++resetHoldMs == kResetHoldMs)
        {
            if (RoutingOpen())
            {
                for (uint8_t p = 0; p < NUM_POTS; p++)
                {
                    routing.zone[p] = 0;       // input follower
                    routeCaught[p]  = false;   // re-catch against the new zone
                }
            }
            else if (SecondaryOpen())
            {
                for (uint8_t p = 0; p < NUM_POTS; p++)
                {
                    secState[p].stored = kSecDefault[p];
                    secState[p].caught = false;
                    if (kSecEnabled[p]) SecApply(p, SecValue(p, secState[p].stored));
                }
            }
            else
            {
                const uint8_t pg = pager.Page();
                for (uint8_t p = 0; p < NUM_POTS; p++)
                    pager.SetStored(pg, p, kPageDefaults[pg][p], loop.Phys());
            }
            saveFlashMs = t_ms;   // LED confirm (see OnRender)
        }
    }
    else
        resetHoldMs = 0;

    /* B2 = force trigger, on every press. A forced slice is a real slice, so it
     * pulses TRIG 1 exactly like an auto-detected one. */
    if (hw.buttons[1].RisingEdge())
    {
        engine.TriggerSlice();
        trig1.Fire(t_ms);
        sliceFlashMs = t_ms;
    }

    /* TRIG IN (cv[0]) = external slice trigger, always accepted. Rising edge
     * with hysteresis: fire above ~+2 V, re-arm below ~+1 V. Shares the B2
     * dark-flash so the panel confirms external hits too. */
    {
        static bool trigArmed = true;
        const float trig = (loop.Cv()[0] - 0.5f) * 2.0f;   // bipolar ±1 ≈ ±5 V
        if (trigArmed && trig > 0.4f)
        {
            trigArmed = false;
            engine.TriggerSlice();
            trig1.Fire(t_ms);
            sliceFlashMs = t_ms;
        }
        else if (!trigArmed && trig < 0.2f)
            trigArmed = true;
    }

    /* B3 rotates its own page group: first press opens on the remembered page,
     * each press after that advances. A chorded press is never a page press. */
    if (hw.buttons[2].RisingEdge())
        b3Chorded = hw.buttons[0].Pressed() || hw.buttons[1].Pressed();
    if (hw.buttons[2].Pressed()
        && (hw.buttons[0].Pressed() || hw.buttons[1].Pressed()))
        b3Chorded = true;
    if (hw.buttons[2].FallingEdge())
    {
        if (!b3Chorded)
        {
            if (b3Active) b3Page ^= 1;
            else          b3Active = true;
        }
        b3Chorded = false;
    }

    /* B1 while a B3 page is up returns to B1's remembered page instead of
     * advancing it — the pager never sees the release. */
    if (hw.buttons[0].RisingEdge())
        b1Chorded = hw.buttons[1].Pressed() || hw.buttons[2].Pressed();
    if (hw.buttons[0].Pressed())
    {
        if (hw.buttons[1].Pressed() || hw.buttons[2].Pressed())
            b1Chorded = true;
        if (b3Active)
            pager.ConsumeButton();
    }
    if (hw.buttons[0].FallingEdge())
    {
        if (b3Active && !b1Chorded) b3Active = false;
        b1Chorded = false;
    }

    /* TRIG 1 = every slice the engine actually took (auto-detected, B2 and TRIG
     * IN alike); TRIG 2 = the output follower's kept transients. */
    trig1.Follow(engine.SliceCount(),    t_ms);
    trig2.Follow(engine.OutEventCount(), t_ms);
    trig1.Poll(t_ms, hw.cv_jacks[2]);   // panel CV2
    trig2.Poll(t_ms, hw.cv_jacks[4]);   // panel CV3

    /* in-env sits on the I²C MCP — write only on meaningful change. */
    static float inEnvLast = -1.0f;
    const float ie = engine.EnvNormIn() * kEnvOutVolts;
    if (std::fabs(ie - inEnvLast) > 0.02f)
    {
        inEnvLast = ie;
        hw.cv_jacks[3].SetVolts(ie);                               // panel CV5
    }
    hw.cv_jacks[5].SetVolts(engine.EnvNormOut() * kEnvOutVolts);   // panel CV6
}

/* Always-on layer, drawn last every frame regardless of page:
 *   B1/B2/B3   wear the mode color (blue while the routing view is open);
 *              activity flashes the LED OFF.
 *   P3..P6 6pm the four output jacks (TRIG 1, TRIG 2, ENV 1, ENV 2). The trig
 *              pips run off the same events as the jacks, held kTrigLedMs so a
 *              10 ms pulse is visible. The ENV 1 pip turns red on input clip. */
static void OnRender(uint32_t t_ms)
{
    constexpr uint32_t kSliceFlashVisMs = 60;
    constexpr LedPanel::Rgb kOff = {0, 0, 0};
    const bool lit = (t_ms - sliceFlashMs) < kSliceFlashVisMs;

    /* Mode color: all three buttons wear the visible page's hue. B2 dark-flashes
     * on a slice. */
    const LedPanel::Rgb pc = hw.leds.ScaleGlobal(
        b3Active ? kB3PageButton[b3Page] : kPageButton[pager.Page()]);
    hw.leds.SetButtonPair(0, pc);
    hw.leds.SetButtonPair(1, lit ? kOff : pc);
    hw.leds.SetButtonPair(2, pc);

    /* Routing view: repaint every ring with its zone selector. */
    if (RoutingOpen())
        for (uint8_t p = 0; p < NUM_POTS; p++)
            DrawRouteRing(p, routing.zone[p]);

    constexpr LedPanel::Rgb kCvNeg = {255, 10, 0};    // red

    const LedPanel::Rgb w = hw.leds.ScaleGlobal(kWhite);
    if (trig1.Lit(t_ms)) hw.leds.SetRingByHour(2, 6.0f, w);
    if (trig2.Lit(t_ms)) hw.leds.SetRingByHour(3, 6.0f, w);

    /* in-env pip: white by envelope, red when the input clips. */
    constexpr float kClipAt = 0.98f;
    const LedPanel::Rgb inEnv = (engine.InPeak() >= kClipAt)
        ? kCvNeg
        : LedPanel::Scale(kWhite, engine.EnvNormIn());
    hw.leds.SetRingByHour(4, 6.0f, hw.leds.ScaleGlobal(inEnv));
    hw.leds.SetRingByHour(5, 6.0f,
        hw.leds.ScaleGlobal(LedPanel::Scale(kWhite, engine.EnvNormOut())));

    /* Secondary page: repaint every ring over the B1 page underneath. The 6pm CV
     * pips above are untouched (these arcs span 7:30→4:30, never 6:00). */
    if (SecondaryOpen())
        DrawSecondaryRings(t_ms);

    /* Settings-saved confirm: all three buttons blink DARK three times. Painted
     * last so it overrides the normal button layer. */
    const uint32_t flashAge = t_ms - saveFlashMs;
    if (saveFlashMs != 0 && flashAge < kSaveFlashMs
        && ((flashAge / kSaveBlinkMs) & 1u) == 0u)
    {
        hw.leds.SetButtonPair(0, kOff);
        hw.leds.SetButtonPair(1, kOff);
        hw.leds.SetButtonPair(2, kOff);
    }
}

/* ── Audio callback ──────────────────────────────────────────────────── */

static void AudioCallback(daisy::AudioHandle::InputBuffer  in,
                          daisy::AudioHandle::OutputBuffer out,
                          size_t                           size)
{
    engine.ProcessBlock(in[0], in[1], out[0], out[1], size);
}

/* ── Main ────────────────────────────────────────────────────────────── */

int main(void) {
    hw.Init(daisy::SaiHandle::Config::SampleRate::SAI_48KHZ, AUDIO_BLOCK_SAMPLES);
    engine.Init();

    /* CV IO: left column = inputs (trig top, mod bottom); trigger outs across the
     * top row, envelope outs across the bottom. */
    hw.cv_jacks[2].EnableCvOutput();   // panel CV2 — slice trigger
    hw.cv_jacks[4].EnableCvOutput();   // panel CV3 — output-follower trigger
    hw.cv_jacks[3].EnableCvOutput();   // panel CV5 — input-follower envelope
    hw.cv_jacks[5].EnableCvOutput();   // panel CV6 — output-follower envelope

    /* Seed musical defaults (stored norms); pots stay uncaught until the knob
     * sweeps through the stored value. */
    hw.ProcessAllControls();
    float phys[NUM_POTS];
    for (uint8_t i = 0; i < NUM_POTS; i++)
        phys[i] = hw.pots[i].Value();

    for (uint8_t pg = 0; pg < 2; pg++)
        for (uint8_t p = 0; p < NUM_POTS; p++)
            pager.SetStored(pg, p, kPageDefaults[pg][p], phys);

    /* Secondary bank: park each at its default (= the engine's boot value),
     * caught, so the first visit to the page shows the live setting. */
    for (uint8_t p = 0; p < NUM_POTS; p++)
        secState[p].stored = kSecDefault[p];

    pager.SetPageColor(0, kOrange);
    pager.SetPageColor(1, kPink);

    presets.Manage(pager);
    presets.Manage(routing);     /* mod-source zones */
    presets.Manage(secondary);   /* secondary-settings norms */
    presets.UseNames();   /* knob names ride the store → HostLink descriptor */

    loop.Use(pager)
        .Use(modRouter)
        .Use(perfPage)
        .Use(depthPage)
        .OnFrame(OnFrame)
        .OnPoll(OnPoll)
        .OnRender(OnRender);
#ifndef CAPICOLA_BENCH_LOG
    DescribeManual(host);
    loop.Use(host);   /* descriptor + USB come up inside BootLoad() */
#endif

    /* Restore slot 0 over the seeded defaults if a valid save exists (catch
     * re-arms on load). */
    presets.Init();
    presets.BootLoad();

    /* Loaded secondaries only reach the engine if we push them once. */
    for (uint8_t p = 0; p < NUM_POTS; p++)
        if (kSecEnabled[p]) SecApply(p, SecValue(p, secState[p].stored));

    hw.seed.SetLed(true);
#ifdef CAPICOLA_BENCH_LOG
    SerialLog::StartLog(false);
#endif
    hw.StartAudio(AudioCallback);

    for (;;) loop.Tick();
}
