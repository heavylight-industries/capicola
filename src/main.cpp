/**
 * @file main.cpp
 * @brief Capicola firmware — composition site on the alchemy-sdk surface layer.
 *
 * Perf-page knobs (ranges are the spec; the map lives in the VirtualKnob decls):
 *
 *   P1 Pitch      ±12 semitones (unity at noon; ±0.2 st detent snaps to true 0)
 *   P2 Stretch    1.0 (realtime) .. 0 (frozen), (1-x)^2.5 taper
 *   P3 Threshold  keep ratio 0..8 over the envelope average (90% = 4, top = mute)
 *   P4 Grain Size 32 .. 4096 keyframes
 *   P5 Quality    analyzer ε 0.1 (crunchy) .. 0.001 (max)
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
 *   Routing:   pot i = 3-zone source selector — input follower (orange) / output
 *              follower (blue, default) / CV IN (violet), zone-granular catch.
 *   Secondary: envelope smoothing, fade time, keyframe drive, drive
 *              character, MIX dry/wet, feedback bandpass — each with its own
 *              pot-catch.
 *
 * Routing zones and secondary norms both ride the preset store.
 *
 * B2 held 1.5 s saves everything as the boot state (QSPI slot 0); the press
 * still slices. B1+B3 held 0.5 s resets the CURRENT view to factory defaults
 * (stored values only — the save is untouched). Both confirm with a triple
 * dark blink.
 *
 * Panel jacks (2×5, panel-final names, code indices in parens):
 *   INL   TRIG IN (cv[0])   TRIG 1·in-gate (jacks[2])   TRIG 2·out-gate (jacks[4])   OUTL
 *   INR   CV IN   (cv[1])   ENV 1·in-env   (jacks[3])   ENV 2·out-env   (jacks[5])   OUTR
 * TRIG IN = external slice trigger (rising edge, always accepted). CV IN = the
 * single external mod CV (eurorack-bipolar). Trig outs are kept-event
 * monostables (~20 ms), 0/5 V; envelopes are 0..1 → 0..5 V. Same physical
 * channels as the old CV1..CV6 map — only the roles/names changed.
 *
 * Fade (= latency = retrigger floor) is the secondary page's P2, boot 20 ms.
 * The SDK's ControlLoop owns polling, pot-catch (Pager), CV dispatch, and ring
 * rendering (each knob's declarative Ring).
 */

#include "alchemy/control/pot_catch.h"       /* PotState, InitCatch, UpdateCatch */
#include "alchemy/led/anims/fill.h"
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
static constexpr float kGateVolts   = 5.0f;
static constexpr float kEnvOutVolts = 5.0f;

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

static VirtualKnob pitch = VirtualKnob(0, "Pitch")
    .Linear(-12.f, +12.f)
    .Ring(Bipolar(kOrange, kBlue, kCenter));   // up = orange (page hue), down = blue

static VirtualKnob stretch = VirtualKnob(1, "Stretch")
    .Linear(0.0f, 1.0f)
    .Ring(Level(kOrange));

/* Stretch taper: 1.0 (realtime) → 0 (frozen) at the CW stop. Linear gave 2x at
 * noon and dumped all the range into the last few percent; exp was too steep at
 * the top. (1-n)^2.5 lands ~5.7x at noon and reaches a true freeze. */
static float StretchCurve(float n)
{
    if (n < 0.0f) n = 0.0f; else if (n > 1.0f) n = 1.0f;
    return std::pow(1.0f - n, 2.5f);
}

static VirtualKnob threshold = VirtualKnob(2, "Threshold")
    .Linear(0.0f, 1.0f)
    .Ring(Level(kOrange));

static VirtualKnob grainSize = VirtualKnob(3, "Grain Size")
    .Linear(32.f, 4096.f)
    .Ring(Level(kOrange));

static VirtualKnob quality = VirtualKnob(4, "Quality")
    .Linear(0.1f, 0.001f)   // analyzer ε — CW = max fidelity, CCW = crunchy
    .Ring(Level(kOrange));

static VirtualKnob feedback = VirtualKnob(5, "Feedback")
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

static VirtualKnob cvDepth1 = VirtualKnob(0, "P1 Depth")
    .Linear(-1.f, +1.f).Ring(Bipolar(kPink, kOrange, kCenter)).Overdraw(DrawDepthGlow);
static VirtualKnob cvDepth2 = VirtualKnob(1, "P2 Depth")
    .Linear(-1.f, +1.f).Ring(Bipolar(kPink, kOrange, kCenter)).Overdraw(DrawDepthGlow);
static VirtualKnob cvDepth3 = VirtualKnob(2, "P3 Depth")
    .Linear(-1.f, +1.f).Ring(Bipolar(kPink, kOrange, kCenter)).Overdraw(DrawDepthGlow);
static VirtualKnob cvDepth4 = VirtualKnob(3, "P4 Depth")
    .Linear(-1.f, +1.f).Ring(Bipolar(kPink, kOrange, kCenter)).Overdraw(DrawDepthGlow);
static VirtualKnob cvDepth5 = VirtualKnob(4, "P5 Depth")
    .Linear(-1.f, +1.f).Ring(Bipolar(kPink, kOrange, kCenter)).Overdraw(DrawDepthGlow);
static VirtualKnob cvDepth6 = VirtualKnob(5, "P6 Depth")
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

/* Persisted zone per perf pot — rides the preset store as its own component
 * now that routing left the Pager. */
class RoutingStore : public Serializable
{
  public:
    uint8_t zone[NUM_POTS] = {1, 1, 1, 1, 1, 1};   // 1 = output follower (factory default)

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

static Page perfPage  = Page(0).Knobs(pitch, stretch, threshold,
                                      grainSize, quality, feedback);
static Page depthPage = Page(1).Knobs(cvDepth1, cvDepth2, cvDepth3,
                                      cvDepth4, cvDepth5, cvDepth6);

/* ── Surfaces ────────────────────────────────────────────────────────── */

static ControlLoop loop  (hw);
static Pager       pager (hw.buttons[0], 2, NUM_POTS);   // B1: perf ↔ depth

/* Startup settings = preset slot 0. The Pager is the only managed component —
 * its stored knob values are the module's settings. BootLoad() at startup; hold
 * B1+B2 kSaveHoldMs to overwrite (see OnPoll). */
static Presets presets(hw.seed.qspi);

#ifndef CAPICOLA_BENCH_LOG
/* Browser/CLI preset editor over the front-panel USB CDC (SDK default
 * transport). Identity strings are literals — kept by reference. */
static hostlink::Host host(presets, "capicola", "Capicola",
                           "0.3.0", CAPICOLA_GIT_HASH);
#endif

static constexpr uint32_t kSaveHoldMs  = 1500;
static constexpr uint32_t kResetHoldMs = 500;    // B1+B3: reset current page
static constexpr uint32_t kSaveFlashMs  = 600;   // 3 dark pulses at kSaveBlinkMs
static constexpr uint32_t kSaveBlinkMs  = 100;
static uint32_t saveFlashMs = 0;   // 0 = never; set on save/reset commit

/* Factory defaults (stored norms) — seeded at boot AND restored per-page by the
 * B1+B3 hold. One table so the two can never drift apart. (Routing defaults
 * are RoutingStore's zero-init: all input follower.) */
static constexpr float kPageDefaults[2][NUM_POTS] = {
    /* perf: pitch unity, stretch ~6x, threshold mid, grain, quality max,
     * feedback off. */
    {0.5f, 0.5f, 0.5f, 0.3f, 1.0f, 0.0f},
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

/* Secondary norms ride the preset store. */
class SecondaryStore : public Serializable
{
  public:
    size_t SerializedSize() const override { return NUM_POTS * 2; }   // uint16 each
    void   Serialize(uint8_t* out) const override
    {
        for (uint8_t p = 0; p < NUM_POTS; p++)
        {
            float n = secState[p].stored;
            if (n < 0.0f) n = 0.0f; else if (n > 1.0f) n = 1.0f;
            const uint16_t q = (uint16_t)(n * 65535.0f + 0.5f);
            out[p * 2]     = (uint8_t)(q & 0xFF);
            out[p * 2 + 1] = (uint8_t)(q >> 8);
        }
    }
    bool Deserialize(const uint8_t* in) override
    {
        for (uint8_t p = 0; p < NUM_POTS; p++)
        {
            const uint16_t q = (uint16_t)in[p * 2]
                             | (uint16_t)((uint16_t)in[p * 2 + 1] << 8);
            secState[p].stored = (float)q / 65535.0f;
        }
        return true;
    }
    uint32_t SchemaHash() const override
    {
        return 0x53454335u ^ (uint32_t)NUM_POTS;
    }
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

/* B3 press bookkeeping (written by OnPoll at 1 ms cadence, read by OnFrame /
 * OnRender). SHIFT arms only after a hold — clean taps belong to the routing
 * view gesture (double-tap in, single-tap out). */
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
        {
            for (uint8_t p = 0; p < NUM_POTS; p++)
            {
                if (!kSecEnabled[p]) continue;
                UpdateCatch(secState[p], ph[p]);
                SecApply(p, SecValue(p, secState[p].stored));
            }
        }

        prevActive = b3Active;
        prevPage   = b3Page;
    }

    /* Pitch detent: within ±0.2 st of noon the transform snaps to exactly 0 —
     * true unity is reachable by hand (the shift stepper stays the precise
     * route to any integer semitone). Applied after modulation, so a CV
     * wobbling inside the detent also reads as unity. */
    {
        float pv = pitch.Value();               // semitones (stepper adjusts the stored)
        if (pv > -0.2f && pv < 0.2f)
            pv = 0.0f;
        engine.SetSlicePitch(pv);
    }

    /* Secondary page open: skip the primaries so the two layers don't fight. */
    if (!SecondaryOpen())
    {
        engine.SetSliceStretch(StretchCurve(stretch.Value()));
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

/* 1 ms poll: B2 force trigger, B3 tap/hold tracking + the four CV outs. Gate
 * outs sit on the MCP4728 (I²C) so they write only on edges; envelope outs sit
 * on the STM DAC and stream every poll. */
static uint32_t sliceFlashMs = 0;   // last button-originated trigger

static void OnPoll(uint32_t t_ms)
{
    /* B2 held kSaveHoldMs = save current settings as the startup defaults
     * (slot 0). The press still slices on its rising edge. */
    static uint32_t saveHoldMs = 0;
    if (hw.buttons[1].Pressed())
    {
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
                    routing.zone[p] = 1;       // output follower
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

    /* B2 = force trigger, on every press. */
    if (hw.buttons[1].RisingEdge())
    {
        engine.TriggerSlice();
        sliceFlashMs = t_ms;
    }

    /* TRIG IN (cv[0]) = external slice trigger, always accepted. Rising edge
     * with hysteresis: fire above ~+2 V, re-arm below ~+1 V. Shares the B2
     * dark-flash so the panel confirms external hits too.
     * NOTE: this ADC read bench-dead in the jack's previous trig-in life —
     * the P1 6pm pip mirrors the raw value as the live probe. */
    {
        static bool trigArmed = true;
        const float trig = (loop.Cv()[0] - 0.5f) * 2.0f;   // bipolar ±1 ≈ ±5 V
        if (trigArmed && trig > 0.4f)
        {
            trigArmed = false;
            engine.TriggerSlice();
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

    static bool inHi = false, outHi = false;
    const bool ig = engine.InGate();
    const bool og = engine.OutGate();
    if (ig != inHi)  { inHi  = ig; hw.cv_jacks[2].SetVolts(ig ? kGateVolts : 0.0f); }  // panel CV2
    if (og != outHi) { outHi = og; hw.cv_jacks[4].SetVolts(og ? kGateVolts : 0.0f); }  // panel CV3

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
 *   P1/P2 6pm  the two input jacks (TRIG IN, CV IN) — green (+) / red (−),
 *              sqrt brightness. P1 doubles as the TRIG IN hardware probe.
 *   P3..P6 6pm the four output jacks (TRIG 1, TRIG 2, ENV 1, ENV 2). The
 *              ENV 1 pip turns red when the input clips. */
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

    /* P1/P2 pips: the two input jacks (TRIG IN, CV IN), bipolar (green +,
     * red −). sqrt keeps small voltages visible; kCvPipFloor swallows the
     * ADC's idle offset so an unpatched jack stays dark and a real signal
     * fades in from zero. */
    constexpr float kCvPipFloor = 0.08f;
    constexpr LedPanel::Rgb kCvPos = {0, 255, 40};    // green
    constexpr LedPanel::Rgb kCvNeg = {255, 10, 0};    // red
    const float* cvb = loop.Cv();
    for (uint8_t i = 0; i < 2; i++)
    {
        const float v = (cvb[i] - 0.5f) * 2.0f;   // bipolar ±1
        const float a = ((v >= 0.0f) ? v : -v) - kCvPipFloor;
        if (a <= 0.0f)
            continue;
        hw.leds.SetRingByHour(i, 6.0f, hw.leds.ScaleGlobal(
            LedPanel::Scale((v >= 0.0f) ? kCvPos : kCvNeg,
                            std::sqrt(a / (1.0f - kCvPipFloor)))));
    }

    const LedPanel::Rgb w = hw.leds.ScaleGlobal(kWhite);
    if (engine.InGate() || lit) hw.leds.SetRingByHour(2, 6.0f, w);   // B2 press shows too
    if (engine.OutGate())       hw.leds.SetRingByHour(3, 6.0f, w);

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
    hw.cv_jacks[2].EnableCvOutput();   // panel CV2 — input-follower gate
    hw.cv_jacks[4].EnableCvOutput();   // panel CV3 — output-follower gate
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

    /* Shift-secondary bank: park each at its default (= the engine's boot value),
     * caught, so the first shift session shows the live setting. */
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
