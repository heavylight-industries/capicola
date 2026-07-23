/**
 * @file main.cpp
 * @brief Capicola firmware — composition site on the alchemy-sdk surface layer.
 *
 * Perf-page knobs (ranges are the spec; the map lives in the VirtualKnob decls):
 *
 *   P1 Pitch      ±12 semitones (unity at noon; ±0.2 st detent snaps to true 0)
 *   P2 Stretch    1.0 (realtime) .. 0.01 (deep)
 *   P3 Threshold  keep ratio 0..8 over the envelope average (90% = 4, top = mute)
 *   P4 Grain Size 32 .. 4096 keyframes
 *   P5 Quality    analyzer ε 0.1 (crunchy) .. 0.001 (max)
 *   P6 Feedback   0 .. 2.0 — wet → tanh → SVF HP+BP mix → back into the recorder
 *
 * Buttons (panel-final): B1 = PAGE, B2 = SLICE, B3 = SHIFT.
 *
 * Pages (B1 cycles): perf (orange) → mod depth (pink). All three buttons wear
 * the active page's hue; activity flashes the LED OFF (B2 on a trigger, B3
 * while shift is armed).
 *   Depth view: pot i = bipolar depth (−1..+1) for pot i's modulation.
 *
 * Mod ROUTING is not on the page cycle: double-tap B3 to enter (buttons wear
 * blue), single-tap to exit. Pot i = 3-zone source selector — input follower
 * (blue) / output follower (orange) / CV IN (violet), with zone-granular
 * catch. Stored in presets via RoutingStore.
 *
 * B3 = SHIFT (armed after a ~0.2 s hold, so taps stay free for the routing
 * gesture). Perf page: P1 nudges pitch in exact integer semitones; P3..P6 swap
 * to a secondary parameter with its own pot-catch (envelope smoothing, dyadic
 * grain step, MIX dry/wet, feedback cutoff — P2 is currently free). Secondaries
 * are session-only. Depth page: hold + turn any pot resets its mod depth to 0.
 *
 * Chords: B1+B2 held 1.5 s saves everything as the boot state (QSPI slot 0).
 * B1+B3 held 0.5 s resets the CURRENT view (routing, if it's open) to factory
 * defaults. Both confirm with a triple dark blink.
 *
 * Panel jacks (2×5, panel-final names, code indices in parens):
 *   INL   TRIG IN (cv[0])   TRIG 1·in-gate (jacks[2])   TRIG 2·out-gate (jacks[4])   OUTL
 *   INR   CV IN   (cv[1])   ENV 1·in-env   (jacks[3])   ENV 2·out-env   (jacks[5])   OUTR
 * TRIG IN = external slice trigger (rising edge, always accepted). CV IN = the
 * single external mod CV (eurorack-bipolar). Trig outs are kept-event
 * monostables (~20 ms), 0/5 V; envelopes are 0..1 → 0..5 V. Same physical
 * channels as the old CV1..CV6 map — only the roles/names changed.
 *
 * Fade (= latency = retrigger floor, ~20 ms) is a fixed engine internal.
 * The SDK's ControlLoop owns polling, pot-catch (Pager), CV dispatch, and ring
 * rendering (each knob's declarative Ring).
 */

#include "alchemy/control/pot_catch.h"       /* PotState, InitCatch, UpdateCatch */
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

static constexpr LedPanel::Rgb kBlue   = {30, 110, 255};   // perf page / in-follower
static constexpr LedPanel::Rgb kIce    = {150, 210, 255};  // perf bipolar negative
static constexpr LedPanel::Rgb kOrange = {255, 110, 15};   // depth negative / out-follower
static constexpr LedPanel::Rgb kViolet = {160, 40, 255};   // depth positive / ext CV
static constexpr LedPanel::Rgb kCenter = {90, 90, 90};     // bipolar zero
static constexpr LedPanel::Rgb kWhite  = {255, 255, 255};
static constexpr LedPanel::Rgb kPink   = {255, 60, 150};   // depth page / positive depth

/* Mode color: worn by all three buttons and the page-advance indicator, one hue
 * per page. Button activity flashes OFF (legible over any base hue). The
 * routing view (B3 double-tap, not a pager page) wears kBlue. */
static constexpr LedPanel::Rgb kPageButton[2] = {
    kOrange,           // page 0 — perf
    kPink,             // page 1 — depth
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
    .Linear(1.0f, 0.01f)
    .Ring(Level(kOrange));

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
    .Linear(0.f, 2.0f)
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
static constexpr LedPanel::Rgb kSrcColors[kNumSrcZones] = {kBlue, kOrange,
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
    uint8_t zone[NUM_POTS] = {};   // 0 = input follower (factory default)

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
static bool         routingActive = false;
static bool         routeCaught[NUM_POTS] = {};

static void DrawRouteRing(uint8_t pot, uint8_t sel)
{
    /* Orange gets a higher dim floor: its low G/B channels quantize away and it
     * reads dimmer than its neighbors at equal scale. */
    static constexpr float kZoneDim[3] = {0.06f, 0.12f, 0.06f};
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
                           "0.2.0", CAPICOLA_GIT_HASH);
#endif

static constexpr uint32_t kSaveHoldMs  = 1500;
static constexpr uint32_t kResetHoldMs = 500;    // B1+B3: reset current page
static constexpr uint32_t kSaveFlashMs = 400;
static uint32_t saveFlashMs = 0;   // 0 = never; set on save/reset commit

/* Factory defaults (stored norms) — seeded at boot AND restored per-page by the
 * B1+B3 hold. One table so the two can never drift apart. (Routing defaults
 * are RoutingStore's zero-init: all input follower.) */
static constexpr float kPageDefaults[2][NUM_POTS] = {
    /* perf:  pitch unity, stretch ~half-speed, threshold mid, grain, mix full
     * wet, feedback off. */
    {0.5f, 0.5f, 0.5f, 0.3f, 1.0f, 0.0f},
    /* depths: noon = off */
    {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f},
};

/* ── Shift-secondary layer (perf page, B3 held) ──────────────────────────
 * P3..P6 swap to a second parameter with its own pot-catch (P1 keeps the
 * semitone stepper; P2 is free since Quality moved to the base layer). Every
 * default equals the engine's boot value, so entering shift shows the live
 * setting under the catch pip. Uncaught until the pot sweeps through the
 * default; on release the primary is restored (uncaught, so it re-catches).
 * Secondaries are session-only (not written to the preset).
 *
 *   P2  —          free
 *   P3  Smoothing  finder fc    Exp 4e-4..0.5                      default 0.0014
 *   P4  Grain step keyframes    dyadic 32,64,…,4096 (8 steps)      default 128
 *   P5  Mix        dry/wet      Linear 0..1                        default 1 (wet)
 *   P6  FB cutoff  feedback fc  Exp 2e-3..0.9                      default 0.02
 */
static PotState secState[NUM_POTS];        // [2..5] = P3..P6 catch state
static float    secPrimNorm[NUM_POTS];     // primary stored, snapshot at shift entry
static bool     secActive = false;         // a perf-page shift session is live

/* Which pots carry a secondary (P1 = stepper, P2 = free). */
static constexpr bool kSecEnabled[NUM_POTS] = {false, false, true,
                                               true,  true,  true};

/* Default secondary norms (each engine boot value expressed on its sweep). */
static constexpr float kSecDefault[NUM_POTS] = {
    0.5f,      // P1 — unused (semitone stepper)
    0.5f,      // P2 — free
    0.1757f,   // P3 smoothing → fc 0.0014
    0.30f,     // P4 grain     → 128 keyframes (dyadic step 2)
    1.0f,      // P5 mix       → full wet (classic Capicola)
    0.3769f,   // P6 fb cutoff → fc 0.02
};

/* Secondary engineering value from a caught norm. */
static float SecValue(uint8_t pot, float n)
{
    switch (pot)
    {
        case 2: return 4.0e-4f * std::pow(0.5f / 4.0e-4f, n);       // smoothing fc (Exp)
        case 3: { int i = (int)(n * 8.0f); if (i > 7) i = 7; if (i < 0) i = 0;
                  return (float)(32 << i); }                        // grain dyadic
        case 4: return n;                                           // mix (Linear)
        case 5: return 2.0e-3f * std::pow(0.9f / 2.0e-3f, n);       // fb cutoff fc (Exp)
        default: return 0.0f;
    }
}

/* Push a caught secondary value into the engine. */
static void SecApply(uint8_t pot, float v)
{
    switch (pot)
    {
        case 2: engine.SetTkeoCutoff(v);     break;
        case 3: engine.SetGrainSize(v);      break;
        case 4: engine.SetMix(v);            break;
        case 5: engine.SetFeedbackCutoff(v); break;
        default: break;
    }
}

/* Secondary ring: an orange level arc (dim while uncaught) plus a bright catch
 * pip at the stored default. */
static void DrawShiftArc(uint8_t pot, float level, bool caught)
{
    const LedPanel::Rgb on  = hw.leds.ScaleGlobal(LedPanel::Scale(kOrange, caught ? 0.9f : 0.4f));
    const LedPanel::Rgb off = hw.leds.ScaleGlobal(LedPanel::Scale(kOrange, 0.04f));
    const int lit = (int)(level * 12.0f + 0.5f);
    for (int s = 0; s <= 12; s++)
        hw.leds.SetRingByHour(pot, 7.5f + 0.75f * (float)s, (s <= lit) ? on : off);
    if (!caught)
        hw.leds.SetRingByHour(pot, 7.5f + 0.75f * (float)lit,
            hw.leds.ScaleGlobal(LedPanel::Mix(kOrange, kWhite, 0.7f)));
}

/* ── Frame hooks ─────────────────────────────────────────────────────── */

/* B3 press bookkeeping (written by OnPoll at 1 ms cadence, read by OnFrame /
 * OnRender). SHIFT arms only after a hold — clean taps belong to the routing
 * view gesture (double-tap in, single-tap out). */
static constexpr uint32_t kShiftArmMs  = 200;   // hold this long to arm shift
static constexpr uint32_t kTapMaxMs    = 250;   // release under this = a tap
static constexpr uint32_t kDoubleGapMs = 350;   // max gap between the two taps
static uint32_t b3DownMs   = 0;      // press timestamp (0 = not down)
static bool     b3Chorded  = false;  // B1 joined while down → chord, not a tap
static bool     shiftArmed = false;  // held past kShiftArmMs, B1-free

/* Knob values → engine, routing zones → mod sources. */
static void OnFrame()
{
    /* B3 = SHIFT, page-local meanings:
     *   perf + P1:     pitch steps in exact integer semitones (SetStored).
     *   perf + P2..P6: the shift-secondary layer (see the block above).
     *   depth + pot:   turning past kClearMove resets that pot's mod depth to 0. */
    {
        constexpr float kSemiStep  = 1.0f / 24.0f;   // pot travel per semitone
        constexpr float kClearMove = 0.01f;          // travel that counts as a turn
        static bool  shiftPrev = false;
        static float shiftAnchor[NUM_POTS] = {};
        /* Shift = B3 armed (held past kShiftArmMs, no B1 — B1+B3 is the reset
         * chord) and never while the routing view is open. */
        const bool    shift = shiftArmed && !routingActive;
        const uint8_t pg    = pager.Page();
        const float*  ph    = loop.Phys();

        const auto SnapPitch = [&](int semis) {
            if (semis > 12) semis = 12; else if (semis < -12) semis = -12;
            pager.SetStored(0, 0, ((float)semis + 12.0f) / 24.0f, ph);
        };

        if (shift && !shiftPrev)                    // ── shift pressed ──
        {
            for (uint8_t p = 0; p < NUM_POTS; p++)
                shiftAnchor[p] = ph[p];
            if (pg == 0)
            {
                SnapPitch((int)std::lround(pitch.Value()));
                for (uint8_t p = 1; p < 6; p++)      // P2..P6 → arm secondary catch
                {
                    secPrimNorm[p] = pager.Stored(0, p);   // remember the primary
                    if (kSecEnabled[p])
                        InitCatch(secState[p], ph[p]);
                }
                secActive = true;
            }
        }
        else if (shift && shiftPrev)                // ── shift held ──
        {
            if (pg == 0)
            {
                const float d = ph[0] - shiftAnchor[0];   // P1 semitone stepper
                if (d > kSemiStep || d < -kSemiStep)
                {
                    SnapPitch((int)std::lround(pitch.Value()) + ((d > 0) ? 1 : -1));
                    shiftAnchor[0] = ph[0];
                }
                for (uint8_t p = 1; p < 6; p++)            // P3..P6 secondaries
                {
                    if (!kSecEnabled[p]) continue;
                    UpdateCatch(secState[p], ph[p]);
                    SecApply(p, SecValue(p, secState[p].stored));
                }
            }
            else if (pg == 1)
            {
                for (uint8_t p = 0; p < NUM_POTS; p++)     // depth clear (all six)
                {
                    const float d = ph[p] - shiftAnchor[p];
                    if (d > kClearMove || d < -kClearMove)
                    {
                        pager.SetStored(1, p, 0.5f, ph);   // depth zero = noon
                        shiftAnchor[p] = ph[p];
                    }
                }
            }
        }
        else if (!shift && shiftPrev && secActive) // ── shift released ──
        {
            /* Restore the P2..P6 primaries the Pager tracked away while dialing
             * secondaries; SetStored re-arms catch, so nothing jumps. */
            for (uint8_t p = 1; p < 6; p++)
                pager.SetStored(0, p, secPrimNorm[p], ph);
            secActive = false;
        }
        shiftPrev = shift;
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

    /* While shift is held on the perf page, P2..P6 drive their secondaries — skip
     * their primaries so the two layers don't fight (setters are sticky; the
     * stored value is restored on release). */
    const bool shiftP0 = shiftArmed && !routingActive && pager.Page() == 0;
    if (!shiftP0)
    {
        engine.SetSliceStretch(stretch.Value());
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
    if (routingActive)
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
        pager.LockPage(pager.Page(), ph);
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
    /* B1+B2 held kSaveHoldMs = save current settings as the startup defaults
     * (slot 0). The chord claims both buttons: ConsumeButton keeps B1's release
     * from flipping the page, the B2-pressed guard skips the slice. */
    static uint32_t saveHoldMs = 0;
    if (hw.buttons[0].Pressed() && hw.buttons[1].Pressed())
    {
        pager.ConsumeButton();
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
            if (routingActive)
            {
                for (uint8_t p = 0; p < NUM_POTS; p++)
                {
                    routing.zone[p] = 0;
                    routeCaught[p]  = false;   // re-catch against the new zone
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

    /* B2 alone = force trigger. Suppressed while B1 or B3 is down so the chords
     * don't fire a stray slice. */
    if (hw.buttons[1].RisingEdge() && !hw.buttons[0].Pressed()
        && !hw.buttons[2].Pressed())
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

    /* B3 tap machine: clean taps drive the routing view (double-tap opens,
     * single-tap closes); a B1-free hold past kShiftArmMs arms SHIFT. Any B1/B2
     * involvement while down voids both — B3 is then chord material. */
    {
        static uint32_t b3LastTapMs = 0;
        if (hw.buttons[2].RisingEdge())
        {
            b3DownMs  = t_ms;
            b3Chorded = hw.buttons[0].Pressed() || hw.buttons[1].Pressed();
        }
        if (b3DownMs != 0 && hw.buttons[2].Pressed())
        {
            if (hw.buttons[0].Pressed() || hw.buttons[1].Pressed())
                b3Chorded = true;
            if (!b3Chorded && !routingActive
                && (t_ms - b3DownMs) >= kShiftArmMs)
                shiftArmed = true;
        }
        if (hw.buttons[2].FallingEdge())
        {
            const uint32_t held = t_ms - b3DownMs;
            if (!b3Chorded && !shiftArmed && held < kTapMaxMs)
            {
                if (routingActive)
                {
                    routingActive = false;
                    pager.LockPage(pager.Page(), loop.Phys());   // re-arm catch
                    b3LastTapMs   = 0;
                }
                else if (b3LastTapMs != 0
                         && (t_ms - b3LastTapMs) <= kDoubleGapMs)
                {
                    routingActive = true;
                    for (uint8_t p = 0; p < NUM_POTS; p++)
                        routeCaught[p] = false;   // in-zone pots catch next frame
                    b3LastTapMs = 0;
                }
                else
                    b3LastTapMs = t_ms;
            }
            b3DownMs   = 0;
            b3Chorded  = false;
            shiftArmed = false;
        }
    }

    /* Routing view claims B1 (no page flips underneath the overlay). */
    if (routingActive && hw.buttons[0].Pressed())
        pager.ConsumeButton();

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

    /* Mode color: all three buttons wear it together. B2 dark-flashes on a
     * slice, B3 goes dark while shift is armed. */
    const LedPanel::Rgb pc = hw.leds.ScaleGlobal(
        routingActive ? kBlue : kPageButton[pager.Page()]);
    hw.leds.SetButtonPair(0, pc);
    hw.leds.SetButtonPair(1, lit ? kOff : pc);
    hw.leds.SetButtonPair(2, shiftArmed ? kOff : pc);

    /* Routing view: repaint every ring with its zone selector. */
    if (routingActive)
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

    /* Perf page + shift armed: repaint P2..P6 with their secondary ring, over
     * the primary arcs. The 6pm CV pips above are untouched (this arc spans
     * 7:30→4:30, never 6:00). */
    if (shiftArmed && !routingActive && pager.Page() == 0)
        for (uint8_t p = 1; p < 6; p++)
            if (kSecEnabled[p])
                DrawShiftArc(p, secState[p].stored, secState[p].caught);

    /* Settings-saved confirm: all three buttons blink DARK briefly. Painted last
     * so it overrides the normal button layer. */
    if (saveFlashMs != 0 && (t_ms - saveFlashMs) < kSaveFlashMs)
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
    presets.Manage(routing);   /* mod-source zones (left the Pager in the
                                * B3-double-tap rework) */
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

    hw.seed.SetLed(true);
#ifdef CAPICOLA_BENCH_LOG
    SerialLog::StartLog(false);
#endif
    hw.StartAudio(AudioCallback);

    for (;;) loop.Tick();
}
