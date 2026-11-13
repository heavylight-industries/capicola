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
 *   P5 Quality    analyzer ε 0.08 (crunchy) .. 0.001 (max); its top 3 % plays the input's PCM, unshaped (the
 *                 keyframes still steer the heads), crossfaded once, over 20 ms, as the knob crosses (its
 *                 modulation stops short of the boundary)
 *   P6 Feedback   0 .. 1.5 stored, loop gain x4/3 (noon 1, top 2) — wet → sinc sat → SVF bandpass → the
 *                 comb → back into the recorder
 *
 * Buttons (panel-final): B1 = PAGE, B2 = SLICE, B3 = PAGE.
 *
 * Two page groups, each rotated by a single press of its own button and each
 * remembering where it was left. All three buttons wear the visible page's hue;
 * B2 flashes OFF on a trigger.
 *
 *   B1  perf (orange) → mod depth (pink)
 *   B3  engine settings (teal, the "secondary" page in code) → feedback and envelope settings (indigo, the
 *       "diffusion" slot in code) → mod routing (blue); engine settings first from boot
 *
 * Pressing the other group's button jumps to that group's remembered page; the
 * next press of the same button advances within the group.
 *
 *   Depth:     pot i = bipolar depth (−1..+1) for pot i's modulation; B2 held + a nudge zeroes it.
 *   Routing:   pot i = the source wheel, scaled to the setup (Sources): TKEO (orange, default) / PULS (magenta) /
 *              CONT (lime) / CV IN (white); with the output follower among the mods its TKEO (blue) / PULS
 *              (cyan) / CONT (lavender); with Trig 1 / Env 1 as inputs CV 2 (purple) / CV 3 (green).
 *              Zone-granular catch; each pot stores its source id.
 *   Engine:    envelope smoothing, fade time, keyframe drive, drive character, MIX dry/wet; P6 free (its
 *              preset slot holds the feedback bandpass, turned on the feedback and envelope page).
 *
 *   Feedback and envelope: P1 the feedback comb (0 = off .. a keyframe period .. the grain, its taps spreading
 *              as it goes), P2 the feedback bandpass (Engine P6's slot), P3 PULS's width, P4 the hits' fold
 *              (CONT and PULS), P5 their delay; P6 dark. Not saved but P2: it boots at its defaults, so saves
 *              keep 1.2.0's preset format.
 *
 * Routing zones and secondary norms both ride the preset store.
 *
 * Each button pair is one state move, confirmed with a triple white flash:
 *   B1+B2 held 0.33 s  save everything as the boot state (QSPI slot 0); the save
 *                      it replaces moves into the history (slots 1..15)
 *   B2+B3 held 0.5 s   load it back (nothing happens if the slot is empty)
 *   B1+B3 held 0.5 s   factory defaults on ALL four pages; the save is untouched
 * A chord has to form inside the hold window of B2/B3, or that button is already
 * a hold.
 *
 * B2 slices on RELEASE. Held alone past kHoldArmMs it is also the POLARITY FLIP
 * gesture on P1/P2 (Primary page only): the rate pins where it is and the knob
 * becomes a direction control — nudge it either way to flip polarity at that
 * same rate. Release and the flip becomes the knob's stored value, behind a
 * catch pip until the pot is swept through it. A hold that flipped anything
 * doesn't slice on release; one that didn't slices like a slow tap.
 *
 * B3 (printed SHIFT) pages on release. Held alone past kHoldArmMs it is the
 * SHIFT layer, on every page and all six knobs: each holds still and toggles —
 * nudged toward its snap target (factory, or the save point per the Options
 * setting) it snaps there, nudged back it returns. A hold that snapped anything
 * doesn't page on release.
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
#include "host_controls.h"
#include "git_stamp.h"                   /* CAPICOLA_GIT_HASH, per build */
#include "jack_routing.h"                /* which signal drives which CV jack */
#include "capicola_manual_text.h"        /* knob names — generated from docs/manual.html */
#include "audio_engine.h"
#include "mod_swap.h"
#ifdef CAPICOLA_BENCH_LOG
#include "AdaptiveComb.h"                 /* (the boot micro benchmark) */
#include "Shapers.h"
#endif

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
static constexpr LedPanel::Rgb kViolet = {160, 40, 255};   // CV 2 — the Trig 1 jack as an input
static constexpr LedPanel::Rgb kCenter = {90, 90, 90};     // bipolar zero
static constexpr LedPanel::Rgb kWhite  = {255, 255, 255};
static constexpr LedPanel::Rgb kPink   = {255, 60, 150};   // depth page / positive depth
static constexpr LedPanel::Rgb kTeal   = {0, 220, 200};    // secondary page
static constexpr LedPanel::Rgb kIndigo = {120, 70, 255};   // diffusion page
static constexpr LedPanel::Rgb kGreen  = {20, 255, 40};    // CV 3 — the Env 1 jack as an input
static constexpr LedPanel::Rgb kCvWhite = {255, 225, 190}; // CV IN — warm, a full-RGB white reads blue

/* Mode color: worn by all three buttons, one hue per page. Activity flashes OFF.
 * B1 pages are warm, B3 pages cool. */
static constexpr LedPanel::Rgb kPageButton[2] = {
    kOrange,           // B1 page 0 — perf
    kPink,             // B1 page 1 — depth
};

static constexpr uint8_t kB3Routing   = 0;
static constexpr uint8_t kB3Secondary = 1;
static constexpr uint8_t kB3Diffusion = 2;

static constexpr LedPanel::Rgb kB3PageButton[3] = {
    kBlue,             // B3 page 0 — CV routing
    kTeal,             // B3 page 1 — secondary settings
    kIndigo,           // B3 page 2 — diffusion
};

/* B3's rotation: secondary → comb (the diffusion slot) → routing → secondary (heavylight, 2026-11-02: routing after
 * the comb page; the comb page only where the build has it). Indexed by page: routing, secondary, comb. */
#ifdef CAPICOLA_FB_DIFFUSER
static constexpr uint8_t kB3Next[3] = {kB3Secondary, kB3Diffusion, kB3Routing};
#else
static constexpr uint8_t kB3Next[3] = {kB3Secondary, kB3Routing, kB3Secondary};
#endif


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

    bool Lit(uint32_t t_ms) const
    {
        return startMs != 0u && (t_ms - startMs) < kTrigLedMs;
    }

    /* The jack-side pulse: high for kTrigPulseMs. */
    bool High(uint32_t t_ms) const
    {
        return startMs != 0u && (t_ms - startMs) < kTrigPulseMs;
    }

  private:
    uint32_t lastCount = 0;
    uint32_t startMs   = 0;   // 0 = never fired
};

static TrigOut trig1;   // TRIG 1 — every slice the engine takes
static TrigOut trig2;   // TRIG 2 — kept output-follower transients

/* ── Hardware / engine ───────────────────────────────────────────────── */

static AlchemyLabV2                 hw;
static AlchemyLabAudio::AudioEngine engine;


/* Trig 1 / Env 1 as CV inputs (CV 2, CV 3), and what Trig 2 / Env 2 carry
 * then (jack_routing.h). Options fields, so they ride presets; OnPoll applies
 * them to the jacks whenever they change, whoever changed them. */
static jacks::Options jackOpts;

/* ── Mod routing (replaces CvMatrix) ─────────────────────────────────────
 * Each perf pot picks its modulation source — input follower, output follower,
 * the CV IN jack, and with the jacks flipped, CV 2 and CV 3 — and takes its
 * bipolar depth from the depth view, crossfading on a switch. A CvSource
 * because two of the sources are internal engine signals. */
class ModRouter : public CvSource
{
  public:
    /* Source ids, stored per pot in presets: append only (0..4 as 1.2 stored them). */
    static constexpr uint8_t kSrcInEnv   = 0;   // TKEO: the input follower's envelope
    static constexpr uint8_t kSrcOutEnv  = 1;   // the output follower's envelope
    static constexpr uint8_t kSrcCv      = 2;   // panel CV IN (cv[1])
    static constexpr uint8_t kSrcCv2     = 3;   // Trig 1 jack as input (cv[2])
    static constexpr uint8_t kSrcCv3     = 4;   // Env 1 jack as input (cv[3])
    static constexpr uint8_t kSrcInPuls  = 5;   // PULS from the input's hits
    static constexpr uint8_t kSrcInCont  = 6;   // CONT from the input's hits
    static constexpr uint8_t kSrcOutPuls = 7;   // PULS from the output follower's hits
    static constexpr uint8_t kSrcOutCont = 8;   // CONT from the output follower's hits
    static constexpr uint8_t kNumSrc     = 9;

    void SetDepth (uint8_t pot, float d)   { if (pot < NUM_POTS) depth[pot] = d; }
    void SetSource(uint8_t pot, uint8_t s) { if (pot < NUM_POTS) src[pot]   = s; }

    float DeltaAtPage(uint8_t page, uint8_t pot) const override
    {
        return (page == 0 && pot < NUM_POTS) ? delta[pot] : 0.0f;
    }

    void Update(const float* cv, uint32_t t_ms) override
    {
        sig[kSrcInEnv]   = engine.EnvNormIn();      // unipolar 0..1
        sig[kSrcOutEnv]  = engine.EnvNormOut();
        sig[kSrcInPuls]  = engine.PulseIn();        // -1..1 (folded strengths go negative)
        sig[kSrcInCont]  = engine.ContourIn();
        sig[kSrcOutPuls] = engine.PulseOut();
        sig[kSrcOutCont] = engine.ContourOut();
        sig[kSrcCv]     = (cv[1] - 0.5f) * 2.0f;      // bipolar ±1
        sig[kSrcCv2]    = jackOpts.cvIn ? (cv[2] - 0.5f) * 2.0f : 0.0f;
        sig[kSrcCv3]    = jackOpts.cvIn ? (cv[3] - 0.5f) * 2.0f : 0.0f;
        for (uint8_t p = 0; p < NUM_POTS; p++)
            delta[p] = swap[p].Out(src[p] < kNumSrc ? src[p] : kSrcInEnv, depth[p], sig, t_ms);
    }

    /* Live value of a source, for the routing rings and the pips. */
    float Signal(uint8_t s) const { return (s < kNumSrc) ? sig[s] : 0.0f; }

  private:
    float   sig  [kNumSrc]  = {};
    float   depth[NUM_POTS] = {};
    uint8_t src  [NUM_POTS] = {};   // defaults to input follower
    float   delta[NUM_POTS] = {};
    modswap::Swap swap[NUM_POTS];   // a source change crossfades (heavylight, 2026-11-02: the routing page played live)
};

static ModRouter modRouter;

/* ── Page 0 — performance knobs ──────────────────────────────────────── */

static constexpr float kPitchMax    = 2.0f;    // speed at either stop
static constexpr float kPitchDetent = 0.03f;   // ±0.03x holds each unity

/* ── Polarity flip gesture, shared by Pitch and Stretch ───────────────────
 * Hold B2 and the knob stops being a value and becomes
 * a DIRECTION control: the magnitude pins where it was, and the MOMENTUM of the
 * knob sets polarity — nudge it toward negative and playback flips to reverse at
 * that same rate, nudge back toward positive and it flips forward. Release and
 * the value the gesture left behind is written into the pager, so the knob is
 * decoupled behind the pager's own catch pip until the pot is swept through it,
 * and a save keeps the flip.
 *
 * Momentum is measured on the physical pot only. Modulation never flips a
 * polarity: a CV or follower riding P1/P2 under a held B2 would otherwise flip
 * on its own and eat the release's slice.
 *
 * B2 is the only hold source. Its slice used to fire on press, so every flip
 * also snapped the head to now; it now fires on release, and not at all if the
 * hold nudged P1 or P2. A sustained TRIG IN gate was tried as a second source
 * and taken out: a long gate then made both knobs inert for value, which is the
 * wrong thing to happen to a patch that just uses long gates.
 *
 * One struct per knob; the magnitude map and its inverse are passed in, since
 * Pitch and Stretch taper differently. */
static constexpr float kGestureNudge = 0.015f;   // 1.5% of sweep counts as a move

struct GestureKnob
{
    PotState view;                  // the pager's catch for this pot — what the ring shows
    bool     commit      = false;   // a flip to write into the pager as the new stored value
    bool     heldPrev    = false;
    bool     latchRev    = false;   // polarity the momentum has latched
    bool     nudged      = false;   // moved during this B2 hold — the release won't slice
    float    ref         = 0.75f;   // pot position the nudge is measured from
    float    frozenMag   = 1.0f;
    float    liveMag     = 1.0f;    // magnitude to play this frame
    float    heldNorm    = 0.75f;   // live gesture value; becomes the stored value
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
        if (!g.heldPrev)                       // gesture opens: pin rate and side
        {
            /* From what is PLAYING, not from the knob. After a release while
             * flipped the pot still sits on the old side, so seeding from it
             * would snap polarity back the instant B2 is pressed again.
             * displayNorm is last frame's effective value — exactly right
             * whether the knob is caught or still decoupled. */
            const float cur = g.displayNorm;
            g.frozenMag = magFn(cur);
            g.latchRev  = (cur < 0.5f);
            g.ref       = phys;              // momentum is measured on the pot itself
        }
        if (phys < g.ref - kGestureNudge)      { g.latchRev = true;  g.ref = phys; g.nudged = true; }
        else if (phys > g.ref + kGestureNudge) { g.latchRev = false; g.ref = phys; g.nudged = true; }
        g.heldNorm = normFn(g.frozenMag, g.latchRev);
        g.liveMag  = g.frozenMag;        // exact — no norm round trip
        eff = g.heldNorm;
    }
    else
    {
        /* Just released: what the gesture left playing becomes the knob's
         * stored value in the pager (the caller writes it — `commit`), so a
         * save keeps the flip and the pager's own catch pip decouples the pot.
         * Only if it flipped something: a hold that nudged nothing leaves the
         * knob as it was. The release frame plays the held value, since `n`
         * was read before the write. */
        if (g.heldPrev && g.nudged)
        {
            g.commit = true;
            eff      = g.heldNorm;
        }
        else
            eff = n;
        if (eff < 0.0f) eff = 0.0f; else if (eff > 1.0f) eff = 1.0f;
        g.liveMag = magFn(eff);
    }
    g.heldPrev      = held;
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
     * itself or a decoupled P1/P2 would look live when every other knob dims.
     * `view` is the pager's catch, copied in each frame by OnFrame. */
    const float k = g.view.caught ? 1.0f : 0.30f;

    FillDesc desc;
    desc.mode         = FillMode::Center;
    desc.color        = LedPanel::Scale(kOrange, k);   // forward
    desc.neg_color    = LedPanel::Scale(kBlue,   k);   // reverse
    desc.center_color = LedPanel::Scale(kCenter, k);

    RingFrame f;
    f.Begin(arc);
    f.Base(desc, g.displayNorm, t_ms);
    f.Emit(panel, pot, g.view);
}

VirtualKnob pitch = VirtualKnob(0, manual_text::kPrimaryName[0])
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

VirtualKnob stretch = VirtualKnob(1, manual_text::kPrimaryName[1])
    .Ident("perf.stretch")
    .Linear(0.0f, 1.0f)
    /* Direction, not a percent: noon is freeze. */
    .Disp("{\"kind\":\"bipolar\",\"neg\":\"Reverse\",\"pos\":\"Forward\",\"center\":\"Freeze\"}")
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

VirtualKnob threshold = VirtualKnob(2, manual_text::kPrimaryName[2])
    .Ident("perf.threshold")
    .Linear(0.0f, 1.0f)
    .Ring(Level(kOrange));

VirtualKnob grainSize = VirtualKnob(3, manual_text::kPrimaryName[3])
    .Ident("perf.grain")
    .Linear(32.f, 4096.f)
    .Ring(Level(kOrange));

/* Quality's top 3 % (heavylight, 2026-11-02: a detent, crossfaded only as it is crossed, never blended): the heads
 * read the input's PCM there, unshaped. The knob's map is linear 0.08 .. 0.001: kPcmBelow is ε at the boundary. */
static constexpr float kPcmTop   = 0.97f;
static constexpr float kPcmBelow = 0.08f - kPcmTop * (0.08f - 0.001f);

VirtualKnob quality = VirtualKnob(4, manual_text::kPrimaryName[4])
    .Ident("perf.quality")
    /* ε 0.08..0.001. 0.1 was a bit much at the CCW end and 0.03 was not
     * crunchy enough; 0.08 sits between them. */
    .Linear(0.08f, 0.001f)  // analyzer ε — CW = max fidelity, CCW = crunchy
    /* The host would otherwise render the raw epsilon as a backwards
     * "0.1 .. 0.001" sweep; fidelity percent is what the knob means. */
    .Disp("{\"kind\":\"norm\"}")
    .Ring(Level(kOrange));

VirtualKnob feedback = VirtualKnob(5, manual_text::kPrimaryName[5])
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

VirtualKnob cvDepth1 = VirtualKnob(0, manual_text::kDepthName[0])
    .Ident("depth.p1")
    .Linear(-1.f, +1.f).Ring(Bipolar(kPink, kOrange, kCenter)).Overdraw(DrawDepthGlow);
VirtualKnob cvDepth2 = VirtualKnob(1, manual_text::kDepthName[1])
    .Ident("depth.p2")
    .Linear(-1.f, +1.f).Ring(Bipolar(kPink, kOrange, kCenter)).Overdraw(DrawDepthGlow);
VirtualKnob cvDepth3 = VirtualKnob(2, manual_text::kDepthName[2])
    .Ident("depth.p3")
    .Linear(-1.f, +1.f).Ring(Bipolar(kPink, kOrange, kCenter)).Overdraw(DrawDepthGlow);
VirtualKnob cvDepth4 = VirtualKnob(3, manual_text::kDepthName[3])
    .Ident("depth.p4")
    .Linear(-1.f, +1.f).Ring(Bipolar(kPink, kOrange, kCenter)).Overdraw(DrawDepthGlow);
VirtualKnob cvDepth5 = VirtualKnob(4, manual_text::kDepthName[4])
    .Ident("depth.p5")
    .Linear(-1.f, +1.f).Ring(Bipolar(kPink, kOrange, kCenter)).Overdraw(DrawDepthGlow);
VirtualKnob cvDepth6 = VirtualKnob(5, manual_text::kDepthName[5])
    .Ident("depth.p6")
    .Linear(-1.f, +1.f).Ring(Bipolar(kPink, kOrange, kCenter)).Overdraw(DrawDepthGlow);

/* ── Mod routing view (B3 page 0; not a pager page) ──────────────────────
 * Zone source selector per pot: input follower / output follower / CV IN, and
 * CV 2 / CV 3 when Trig 1 and Env 1 are flipped to inputs (CCW→CW). The ring
 * shows every zone in its color, the selected one lit by its source's live
 * level; the dim zones double as the position map. No catch pip — but catch
 * is zone-granular: a pot decouples on entry and re-locks the moment it sits
 * inside its stored zone. */

static constexpr uint8_t kMaxSrcZones = ModRouter::kNumSrc;
/* The routing wheel scales with the setup (heavylight, 1.3): it shows the sources the current Options make
 * available, CCW → CW, and stores each pot's source id (ModRouter's, append only), never its place on the wheel,
 * so a preset means the same source whatever the wheel shows. Always TKEO, PULS, CONT, CV IN; with the output
 * follower among the mods, its TKEO, PULS and CONT; with Trig 1 / Env 1 as inputs, CV 2 and CV 3. */
static uint8_t Sources(uint8_t* list)
{
    uint8_t n = 0;
    list[n++] = ModRouter::kSrcInEnv;
    list[n++] = ModRouter::kSrcInPuls;
    list[n++] = ModRouter::kSrcInCont;
    list[n++] = ModRouter::kSrcCv;
    if (jackOpts.outFollower) {
        list[n++] = ModRouter::kSrcOutEnv;
        list[n++] = ModRouter::kSrcOutPuls;
        list[n++] = ModRouter::kSrcOutCont;
    }
    if (jackOpts.cvIn) {
        list[n++] = ModRouter::kSrcCv2;
        list[n++] = ModRouter::kSrcCv3;
    }
    return n;
}

/* Each source's hue, by id; neighbours on any wheel differ. */
static constexpr LedPanel::Rgb kSrcColors[kMaxSrcZones] = {
    kOrange,             // TKEO
    kBlue,               // TKEO out
    kCvWhite,            // CV IN
    kViolet,             // CV 2
    kGreen,              // CV 3
    {255, 0, 150},       // PULS (magenta)   (heavylight, 2026-11-02: more contrast round the wheel)
    {170, 255, 0},       // CONT (lime)
    {0, 190, 255},       // PULS out (cyan)
    {150, 120, 255},     // CONT out (lavender)
};

static uint8_t NumZones() { uint8_t l[kMaxSrcZones]; return Sources(l); }

/* The source a pot's place on the wheel names. */
static uint8_t SourceAt(float v)
{
    uint8_t l[kMaxSrcZones];
    const int n = Sources(l);
    int z = (int)(v * (float)n);
    z = (z > n - 1) ? n - 1 : (z < 0) ? 0 : z;
    return l[z];
}

/* A stored source the setup doesn't offer now plays as TKEO; the stored id is kept, so offering it again restores
 * it. */
static uint8_t EffectiveZone(uint8_t id)
{
    uint8_t l[kMaxSrcZones];
    const uint8_t n = Sources(l);
    for (uint8_t i = 0; i < n; i++) if (l[i] == id) return id;
    return ModRouter::kSrcInEnv;
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

/* Raised by every preset apply — boot, the panel's load, and HostLink's
 * load-from-slot and full-state live push alike — so OnFrame can re-arm catch
 * against the new values. Presets has no after-load hook; every apply deserializes the
 * whole managed set, RoutingStore included, so that is where it's raised. */
static bool stateApplied = false;

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
            zone[p] = (in[p] < kMaxSrcZones) ? in[p]
                                             : (uint8_t)(kMaxSrcZones - 1);
        stateApplied = true;
        return true;
    }
    uint32_t SchemaHash() const override
    {
        return 0x43563034u ^ (uint32_t)NUM_POTS
                           ^ ((uint32_t)5u << 8);   // (1.2's five zones: the same bytes now hold source ids)
    }

#ifndef CAPICOLA_BENCH_LOG
    /* Self-describe to HostLink so the routing page is editable and
     * documented on the web programmer instead of an opaque blob. One byte
     * per pot maps straight onto an enum field — no storage change, so the
     * schema hash (and every saved preset) is untouched. Prose and labels
     * live in capicola_manual.cpp with the rest of the manual. */
    bool Describe(hostlink::ComponentWriter& w) const override
    {
        /* A settings page, so the programmer draws it on the panel like the
         * pager's pages, in the page's own color. */
        static const char* const kPage [1] = {"Mod Routing"};
        static const char* const kColor[1] = {"#1e6eff"};   // kBlue
        w.Ident("routing").Kind("settings").Label("Mod Routing").Grid(1, NUM_POTS);
        w.PageMeta(kPage, kColor, 1);
        w.Meta("pageHelp", manual_text::kRoutingPageHelpJson);   // "About this page"
        for (uint8_t p = 0; p < NUM_POTS; p++)
        {
            const SeeRef see[1] = {SeeRef(*kRouteSeeKnob[p])};
            if (!w.Field(kRouteFieldId[p], kRouteFieldName[p], p,
                         hostlink::FieldType::Enum, 0.0f /* in follower */,
                         kRouteDispJson, kMaxSrcZones, 0, (int16_t)p,
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
static uint8_t b3Page   = kB3Secondary;   // (the first B3 press opens the secondary settings)

/* B1 page held frozen while a B3 page is up. */
static uint8_t underPage = 0;
static float   underNorm[NUM_POTS] = {};

static inline bool RoutingOpen()   { return b3Active && b3Page == kB3Routing; }
static inline bool SecondaryOpen() { return b3Active && b3Page == kB3Secondary; }
static inline bool DiffusionOpen() { return b3Active && b3Page == kB3Diffusion; }

/* The selected zone's brightness follows its source live — an envelope as
 * it is, a CV by |volts| — over a floor that keeps the selection readable, so
 * the page shows at a glance whether anything is arriving. */
static void DrawRouteRing(uint8_t pot, uint8_t sel, float level)
{
    /* Orange gets a higher dim floor: its low G/B channels quantize away and it
     * reads dimmer than its neighbors at equal scale. (By source id.) */
    static constexpr float kZoneDim[kMaxSrcZones] = {0.06f, 0.03f, 0.03f, 0.03f, 0.03f, 0.03f, 0.03f, 0.03f, 0.03f};   // (heavylight, 2026-11-02: half 1.3's)
    static constexpr float kSelFloor = 0.25f;
    /* The arc split evenly among however many sources the wheel shows, in order, exactly as the knob splits its
     * travel (SourceAt) (heavylight, 2026-11-02: even zones, whatever their number). The 13 LEDs are 13 equal
     * cells, so every zone gets 13/n of them; an LED a border crosses mixes its two zones by how much of its cell
     * each covers. (A zone is never narrower than a cell: at most two meet in one.) */
    uint8_t l[kMaxSrcZones];
    const int n = Sources(l);
    if (level < 0.0f) level = 0.0f; else if (level > 1.0f) level = 1.0f;
    auto zone = [&](int z) {
        const uint8_t id = l[z];
        const float   k  = (id == sel) ? kSelFloor + (1.0f - kSelFloor) * level : kZoneDim[id];
        return LedPanel::Scale(kSrcColors[id], k);
    };
    for (int s = 0; s <= 12; s++)
    {
        const float lo = (float)s * (1.0f / 13.0f);
        const float hi = (float)(s + 1) * (1.0f / 13.0f);
        int i = (int)(lo * (float)n);
        if (i > n - 1) i = n - 1;
        const float edge = (float)(i + 1) / (float)n;   // where zone i ends
        LedPanel::Rgb c = zone(i);
        if (i < n - 1 && edge < hi) c = LedPanel::Mix(c, zone(i + 1), (hi - edge) / (hi - lo));
        hw.leds.SetRingByHour(pot, 7.5f + 0.75f * (float)s, hw.leds.ScaleGlobal(c));
    }
}

Page perfPage  = Page(0).Name("Primary").Color("#ff6e0f").Knobs(pitch, stretch, threshold,
                                      grainSize, quality, feedback);
Page depthPage = Page(1).Name("Depth").Color("#ff3c96").Knobs(cvDepth1, cvDepth2, cvDepth3,
                                      cvDepth4, cvDepth5, cvDepth6);

/* ── Surfaces ────────────────────────────────────────────────────────── */

static ControlLoop loop  (hw);
static Pager       pager (hw.buttons[0], 2, NUM_POTS);   // B1: perf ↔ depth

static inline bool DepthOpen() { return !b3Active && pager.Page() == 1; }

/* Startup settings = preset slot 0: the Pager's stored knob values, routing,
 * secondary and options. BootLoad() at startup; hold B1+B2 kSaveHoldMs to save
 * (see SaveToHistory, OnPoll). */
static Presets presets(hw.seed.qspi);

#ifndef CAPICOLA_BENCH_LOG
/* Browser/CLI preset editor over the front-panel USB CDC (SDK default
 * transport). Identity strings are literals — kept by reference. */
/* The descriptor carries the whole manual — every control's help with its
 * facts — which outgrew the SDK's default 24 KiB. SDRAM has room to spare;
 * the HELLO length field is 32-bit. */
static char DSY_SDRAM_BSS hostDescriptor[64 * 1024];
static char DSY_SDRAM_BSS hostRefresh   [64 * 1024];   // a refresh rebuilds into this

/* capicola_manual.cpp — republish the jacks for the current jack options. */
void DescribeJacks(hostlink::Host& host, const jacks::Options& o);

hostlink::Host host(presets, "capicola", "Capicola",
                           "1.3.0-exp", CAPICOLA_GIT_HASH);

/* src/capicola_manual.cpp — attaches the firmware-authored manual (prose,
 * jacks, buttons) to the descriptor. Called before BootLoad() brings USB up. */
void DescribeManual(hostlink::Host& host);
#endif

static constexpr uint32_t kSaveHoldMs  = 330;
static constexpr uint32_t kLoadHoldMs  = 500;    // B2+B3: load the boot state
static constexpr uint32_t kResetHoldMs = 500;    // B1+B3: factory, all pages
static constexpr uint32_t kSaveFlashMs  = 600;   // 3 white pulses at kSaveBlinkMs
static constexpr uint32_t kSaveBlinkMs  = 100;
static uint32_t saveFlashMs = 0;   // 0 = never; set when a save/load/reset completes

/* Start the confirm flash now — read the clock after the work, not before: a
 * save writes the history slot first, and flash writes block the loop. */
static void ConfirmFlash()
{
    const uint32_t now = daisy::System::GetNow();
    saveFlashMs = now ? now : 1u;
}


/* Factory defaults (stored norms) — seeded at boot AND restored per-page by the
 * B1+B3 hold. One table so the two can never drift apart. (Routing zones default
 * to the input follower in RoutingStore's own initializer.) */
static constexpr float kPageDefaults[2][NUM_POTS] = {
    /* perf: pitch unity forward, stretch ~1.75x forward (not realtime: at 1.0
     * a pitched grain at "now" is just the OLA taking over), threshold mid,
     * grain 642 keyframes, quality max, feedback off. */
    {0.75f, 0.9f, 0.5f, 0.15f, 1.0f, 0.0f},
    /* depths: noon = off */
    {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f},
};

/* ── Secondary settings (B3 page 1) ──────────────────────────────────────
 *   P1  Smoothing  envelope fc  Exp 5e-5..0.125                    default 0.002 (~48 Hz)
 *   P2  Fade       OLA crossfade  Exp 10..250 ms                   default 20 ms
 *   P3  Drive      keyframe shaper gain   Linear 0.5..4.0         default 1.0
 *   P4  Character  quake ← clean → sinc   Linear 0..1             default 1 (sinc)
 *   P5  Mix        dry/wet      Linear 0..1                        default 1 (wet)
 *   P6  FB band    feedback bandpass fc   Exp 100 Hz..0.9         default 0.02 (turned on the feedback and envelope page)
 */
static PotState secState[NUM_POTS];

/* P6 (the feedback bandpass) is shown and turned on the feedback and envelope page (its P2) but still stored here, in the preset
 * store's slot: dark on this page, applied like the rest. */
static constexpr bool kSecEnabled[NUM_POTS] = {true, true, true,
                                               true, true, false};

/* Default secondary norms (each engine boot value expressed on its sweep). */
/* The feedback bandpass's lowest centre: 100 Hz (heavylight, 2026-11-02; 1.2 went down to 48 Hz). */
static constexpr float kFbBandLo = 100.0f / 24000.0f;

static constexpr float kSecDefault[NUM_POTS] = {
    0.4715f,   // P1 smoothing → fc 0.002
    0.2153f,   // P2 fade      → 960 samples (20 ms)
    0.1429f,   // P3 drive     → 1.0
    1.0f,      // P4 character → full sinc
    1.0f,      // P5 mix       → full wet (classic Capicola)
    0.2918f,   // P6 fb band   → fc 0.02
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
        case 5: return kFbBandLo * std::pow(0.9f / kFbBandLo, n);   // fb band fc (Exp, 100 Hz up)
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

/* ── Diffusion settings (B3 page 2) ──────────────────────────────────────
 *   P1  Size       the feedback comb               Linear 0..1      default 0 (off)
 *                  (0 = off, 1.2's feedback; then one keyframe period, the taps together, .. the grain's
 *                  length, the taps spread to golden-ratio shares of it)
 *   P2  FB band    the feedback bandpass's centre  (Secondary P6's: its slot, map and store)
 *   P3  Width      PULS's width (sigma)            Exp 25 ms .. 1 s  default 50 ms
 *   P4  Fold       the hits' strengths through a sine folder (CONT and PULS)   Linear 0..1   default 0 (none)
 *   P5  Delay      CONT and PULS later than their hits                         Linear 0..500 ms  default 0
 *   P6             dark
 *   P3..P6         dark, for later
 * Not in the preset store (a new component would change the preset schema and
 * strand every save): it boots at these defaults. Values are norms, applied as
 * they are. */
static PotState difState[NUM_POTS];

/* PULS's width (feedback and envelope page P3), seconds: 25 ms .. 1 s, exponential (heavylight, 2026-11-02: below a third of the
 * old 4 ms .. 1 s map was far too fast, so that third is the floor; 50 ms by default). */
static constexpr float kPulseWidthLo = 0.0252f;
/* CONT and PULS's delay (feedback and envelope page P5): 0 .. 500 ms, linear (the dt2-keyframer's DLY reach, 0 .. 508 ms). */
static constexpr float kModDelayMax = 0.5f;
static constexpr float kPulseWidthHi = 1.0f;

static constexpr bool kDifEnabled[NUM_POTS] = {true, true, true,
                                               true, true, false};

static constexpr float kDifDefault[NUM_POTS] = {0.0f, 0.2918f, 0.1862f, 0.0f, 0.0f, 0.0f};   // (P2 = kSecDefault[5]; P3 50 ms)

static void DifApply(uint8_t pot, float n)
{
    switch (pot)
    {
        case 0: engine.SetCombSize(n);   break;
        case 1: secState[5].stored = n;  break;   // (the Secondary store's slot: pushed, saved and loaded there)
        case 2: engine.SetPulseWidth(kPulseWidthLo * std::pow(kPulseWidthHi / kPulseWidthLo, n)); break;
        case 3: engine.SetHitFold(n); break;
        case 4: engine.SetModDelay(kModDelayMax * n); break;
        default: break;
    }
}

/* The settings page open now (secondary or diffusion): its pot states and
 * which pots it wires; null on any other page. */
static PotState* SettingsState(const bool*& enabled)
{
    if (SecondaryOpen()) { enabled = kSecEnabled; return secState; }
    if (DiffusionOpen()) { enabled = kDifEnabled; return difState; }
    enabled = nullptr;
    return nullptr;
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
        static const char* const kPage [1] = {"Engine settings"};
        static const char* const kColor[1] = {"#00dcc8"};   // kTeal
        w.Ident("secondary").Kind("settings").Label("Engine settings").Grid(1, NUM_POTS);
        w.PageMeta(kPage, kColor, 1);
        w.Meta("pageHelp", manual_text::kSecondaryPageHelpJson);   // "About this page"
        /* Cross-links: smoothing shapes what threshold sees, drive and
         * character are one shaper, fb tone voices feedback, mix feeds the
         * outputs. */
        static const SeeRef kSee[NUM_POTS][2] = {
            {SeeRef(threshold), SeeRef()},
            {SeeRef(), SeeRef()},
            {SeeRef("sec.character"), SeeRef()},
            {SeeRef("sec.drive"), SeeRef()},
            {SeeRef("out.l"), SeeRef("out.r")},
            {SeeRef(feedback), SeeRef()},
        };
        static constexpr uint8_t kNumSee[NUM_POTS] = {1, 0, 1, 1, 2, 1};
        for (uint8_t p = 0; p < NUM_POTS; p++)
            if (!w.Field(kSecFieldId[p], kSecFieldName[p],
                         p * (uint32_t)sizeof(float),
                         hostlink::FieldType::F32, kSecDefault[p],
                         kSecFieldDisp[p], 0u, 0, (int16_t)p,
                         kSecFieldHelp[p], kNumSee[p] ? kSee[p] : nullptr, kNumSee[p]))
                return false;
        return true;
    }
#endif
};

static SecondaryStore secondary;

#ifndef CAPICOLA_BENCH_LOG
/* Options field identity and prose — defined in capicola_manual.cpp. */
extern const char* const kOptSnapId;
extern const char* const kOptSnapName;
extern const char* const kOptSnapDisp;
extern const char* const kOptSnapHelp;
extern const char* const kOptJacksId;
extern const char* const kOptJacksName;
extern const char* const kOptJacksDisp;
extern const char* const kOptJacksHelp;
extern const char* const kOptTrig2Id;
extern const char* const kOptTrig2Name;
extern const char* const kOptEnv2Id;
extern const char* const kOptEnv2Name;
extern const char* const kOptOutDisp;
extern const char* const kOptOutsHelp;
#endif

/* Settings with no knob, edited from the web programmer. They ride the preset
 * store like everything else, so a save carries them. */
class OptionsStore : public Serializable
{
  public:
    uint8_t snapToSaved = 0;   // SHIFT snap target: 0 = factory, 1 = the save point
                               // bytes 1..3: jackOpts (file scope — ModRouter reads it)

    size_t SerializedSize() const override { return 1 + jacks::kBytes; }
    void   Serialize(uint8_t* out) const override
    {
        out[0] = snapToSaved;
        jacks::Encode(jackOpts, out + 1);
    }
    bool   Deserialize(const uint8_t* in) override
    {
        snapToSaved = in[0] ? 1 : 0;
        jackOpts    = jacks::Decode(in + 1);
        return true;
    }
    uint32_t SchemaHash() const override { return 0x4F505433u; }

#ifndef CAPICOLA_BENCH_LOG
    bool Describe(hostlink::ComponentWriter& w) const override
    {
        w.Ident("options").Label("Options");
        /* Each option links to what it acts on; the jacks link back. */
        static const SeeRef kSnapSee [2] = {SeeRef("b3"), SeeRef("presets")};
        static const SeeRef kJacksSee[2] = {SeeRef("trig.1"), SeeRef("env.1")};
        static const SeeRef kTrig2See[2] = {SeeRef("trig.2"), SeeRef("opt.jacks")};
        static const SeeRef kEnv2See [2] = {SeeRef("env.2"), SeeRef("opt.jacks")};
        return w.Field(kOptSnapId, kOptSnapName, 0, hostlink::FieldType::Enum,
                       0.0f, kOptSnapDisp, 2u, -1, -1, kOptSnapHelp, kSnapSee, 2)
            && w.Field(kOptJacksId, kOptJacksName, 1, hostlink::FieldType::Enum,
                       jacks::Options{}.cvIn ? 1.0f : 0.0f, kOptJacksDisp, 4u,
                       -1, -1, kOptJacksHelp, kJacksSee, 2)
            && w.Field(kOptTrig2Id, kOptTrig2Name, 2, hostlink::FieldType::Enum,
                       (float)jacks::Options{}.trig2, kOptOutDisp, jacks::kNumSignals,
                       -1, -1, kOptOutsHelp, kTrig2See, 2)
            && w.Field(kOptEnv2Id, kOptEnv2Name, 3, hostlink::FieldType::Enum,
                       (float)jacks::Options{}.env2, kOptOutDisp, jacks::kNumSignals,
                       -1, -1, kOptOutsHelp, kEnv2See, 2);
    }
#endif
};

static OptionsStore options;

/* The save point (slot 0) as values, for the SHIFT snap's "saved" target. Read
 * back out of flash whenever slot 0's record changes — boot, a panel save, a
 * write from the programmer — so it is always exactly what B2+B3 would load.
 * Offsets come from walking the managed set, the same order Presets writes. */
static float   savedPage[2][NUM_POTS];   // Pager::Serialize: page-major floats
static uint8_t savedZone[NUM_POTS];
static float   savedSec [NUM_POTS];
static bool    savedValid = false;

static void RefreshSaved()
{
    static uint8_t buf[256];
    savedValid = false;
    const Presets::SlotInfo info = presets.InfoFor(0);
    if (!info.valid || !info.schema_match || presets.LiveSize() > sizeof(buf)) return;
    uint16_t total = 0;
    if (presets.ReadSlotBytes(0, 0, buf, sizeof(buf), nullptr, &total) <= 0
        || total != presets.LiveSize())
        return;
    size_t off = 0;
    for (uint8_t i = 0; i < presets.NumManaged(); i++)
    {
        const Serializable* c = presets.ManagedAt(i);
        if (c == &pager)
            std::memcpy(savedPage, buf + off, sizeof(savedPage));
        else if (c == &routing)
            for (uint8_t p = 0; p < NUM_POTS; p++)
                savedZone[p] = (buf[off + p] < kMaxSrcZones) ? buf[off + p] : 0;
        else if (c == &secondary)
            std::memcpy(savedSec, buf + off, sizeof(savedSec));
        off += c->SerializedSize();
    }
    savedValid = true;
}

/* Panel saves name themselves "Save 1", "Save 2", … so the programmer's slot
 * cards show the history's order. A save that is untitled, or already one of
 * ours, gets the next number; a name typed in the programmer is kept. Names
 * ride last in every record (UseNames pins them there), so each slot's name is
 * its record's final PresetName::kCapacity bytes, whatever its schema. */
static PresetName* presetName = nullptr;   // from presets.UseNames()

static bool AutoNumber(const char* s, uint32_t& n)
{
    if (std::strncmp(s, "Save ", 5) != 0 || s[5] == '\0') return false;
    n = 0;
    for (const char* c = s + 5; *c; c++)
    {
        if (*c < '0' || *c > '9' || n > 99999999u) return false;
        n = n * 10u + (uint32_t)(*c - '0');
    }
    return true;
}

static void NameThisSave()
{
    if (!presetName) return;
    uint32_t n = 0;
    const char* cur = presetName->Get();
    if (cur[0] != '\0' && !AutoNumber(cur, n)) return;   // the user's own name

    uint32_t last = 0;
    char     nm[PresetName::kCapacity];
    for (uint8_t i = 0; i < Presets::kNumSlots; i++)
    {
        const Presets::SlotInfo h = presets.InfoFor(i);
        if (!h.valid || h.length < PresetName::kCapacity) continue;
        if (presets.ReadSlotBytes(i, h.length - PresetName::kCapacity, (uint8_t*)nm,
                                  PresetName::kCapacity, nullptr, nullptr)
            != (int32_t)PresetName::kCapacity)
            continue;
        nm[PresetName::kCapacity - 1] = '\0';
        if (AutoNumber(nm, n) && n > last) last = n;
    }

    char  out[PresetName::kCapacity] = "Save ";
    char  digits[12];
    int   d = 0;
    for (uint32_t v = last + 1u; v; v /= 10u) digits[d++] = (char)('0' + v % 10u);
    char* o = out + 5;
    while (d) *o++ = digits[--d];
    *o = '\0';
    presetName->Set(out);
}

/* Save as a history, in constant time: the save point in slot 0 is copied into
 * an empty history slot — or, once all fifteen are full, over the oldest — and
 * the live state lands in slot 0, the boot state. Two writes, however long the
 * history. The store's sequence counter is global, so the lowest-sequence
 * history slot is the oldest. Records move byte-exact with their own schema
 * hash and name. Older saves are recalled from the web programmer. */
static bool SaveToHistory()
{
    static uint8_t DSY_SDRAM_BSS buf[kPresetBlobCapacity];
    if (presets.InfoFor(0).valid)
    {
        uint8_t  dst    = 0;
        uint32_t oldest = 0xFFFFFFFFu;
        for (uint8_t i = 1; i < Presets::kNumSlots; i++)
        {
            const Presets::SlotInfo h = presets.InfoFor(i);
            if (!h.valid) { dst = i; break; }        // an empty slot first
            if (h.seq < oldest) { oldest = h.seq; dst = i; }
        }
        uint32_t schema = 0;
        uint16_t total  = 0;
        if (presets.ReadSlotBytes(0, 0, buf, sizeof(buf), &schema, &total) >= 0)
            presets.WriteSlotRaw(dst, schema, buf, total);
    }
    return presets.Save(0);
}

/* Settings ring: a level arc (dim while uncaught) plus a bright catch pip, in
 * the page's hue. */
static void DrawShiftArc(uint8_t pot, float level, bool caught, LedPanel::Rgb hue, uint32_t t_ms)
{
    FillDesc d;
    d.mode          = FillMode::Edge;
    d.color         = LedPanel::Scale(hue, caught ? 0.9f : 0.4f);
    d.passive_color = LedPanel::Scale(hue, 0.04f);
    DrawFill(hw.leds, pot, 7.5f, 0.75f, 13, level, d, t_ms);
    if (!caught)
        hw.leds.SetRingByHour(pot, 7.5f + 9.0f * level,
            hw.leds.ScaleGlobal(LedPanel::Mix(hue, kWhite, 0.7f)));
}

/* Every ring on a settings page (secondary or diffusion), so it reads the same
 * over either B1 page. Unwired pots sit dark. */
static void DrawSettingsRings(const PotState* st, const bool* enabled, LedPanel::Rgb hue, uint32_t t_ms)
{
    const LedPanel::Rgb off = hw.leds.ScaleGlobal(LedPanel::Scale(hue, 0.04f));
    for (uint8_t p = 0; p < NUM_POTS; p++)
    {
        if (enabled[p])
            DrawShiftArc(p, st[p].stored, st[p].caught, hue, t_ms);
        else
            for (int s = 0; s <= 12; s++)
                hw.leds.SetRingByHour(p, 7.5f + 0.75f * (float)s, off);
    }
}

/* ── Frame hooks ─────────────────────────────────────────────────────── */

/* B3 press bookkeeping, written by OnPoll at 1 ms cadence and read by OnFrame /
 * OnRender. */
static bool b1Chorded = false;   // another button joined while down → not a page press
static bool b2Chorded = false;   // ... → not a slice
static bool b3Chorded = false;

/* B2 or B3 held alone this long is a hold, not a tap: B2 polarity flip, B3 snap. */
static constexpr uint32_t kHoldArmMs = 100;
static bool b2HoldArmed  = false;   // past the window: a hold, not a chord
static bool b3HoldArmed  = false;
static bool shiftUsed    = false;   // this B3 hold snapped a knob — the release won't page
static bool flipHeld = false;   // armed on the Primary page — P1/P2 are direction controls
static bool depthHeld = false;  // B2 held on the depth page: a nudged knob's depth snaps to zero
static bool depthZeroed = false; // ... and this hold zeroed one: its release doesn't slice

/* Stored value and snap target of pot p on the visible page: its factory
 * default, or its value in the save point when the option asks for that and
 * slot 0 holds one. Routing reports zones. False for a disabled secondary. */
static bool SnapTarget(uint8_t p, float& cur, float& def)
{
    const bool saved = options.snapToSaved && savedValid;
    if (RoutingOpen())
    {
        cur = (float)routing.zone[p];
        def = saved ? (float)savedZone[p] : 0.0f;
        return true;
    }
    if (SecondaryOpen())
    {
        cur = secState[p].stored;
        def = saved ? savedSec[p] : kSecDefault[p];
        return kSecEnabled[p];
    }
    if (DiffusionOpen())   // (not saved: always its factory value; P2 is Secondary P6's)
    {
        cur = difState[p].stored;
        def = (p == 1) ? (saved ? savedSec[5] : kSecDefault[5]) : kDifDefault[p];
        return kDifEnabled[p];
    }
    const uint8_t pg = pager.Page();
    cur = pager.Stored(pg, p);
    def = saved ? savedPage[pg][p] : kPageDefaults[pg][p];
    return true;
}

/* Hold pot p on the visible page at v, uncaught and armed on whichever side
 * the pot sits. Re-asserted every frame while B3 is held, after the pager has
 * updated and before the rings render, so a catch the pager makes as the pot
 * crosses v — or comes within kCatchTolerance of it — is undone unseen. Never a
 * proximity catch: that lit the ring up for a frame. Routing takes a zone. */
static void HoldValue(uint8_t p, float v, const float* ph)
{
    if (RoutingOpen())
    {
        routing.zone[p] = (uint8_t)(v + 0.5f);
        routeCaught[p]  = false;   // the routing block won't re-catch while B3 is held
        return;
    }
    const float side = (ph[p] >= v) ? 2.0f * kCatchTolerance : -2.0f * kCatchTolerance;
    const bool* en;
    if (PotState* st = SettingsState(en))
    {
        st[p].stored       = v;
        st[p].caught       = false;
        st[p].sign_at_lock = (side > 0.0f) ? 1 : -1;
        return;
    }
    float away[NUM_POTS];   // the pot, pushed out of proximity on its own side
    std::memcpy(away, ph, sizeof(away));
    if (std::fabs(ph[p] - v) < 2.0f * kCatchTolerance) away[p] = v + side;
    pager.SetStored(pager.Page(), p, v, away);
}

/* After any preset apply or a factory reset, re-arm every catch so no pot jumps
 * the new stored values: routing per zone, secondary if it is open (it re-arms
 * on entry otherwise), and the B1 page held under a B3 page. Pager pages —
 * P1/P2's flips included — re-arm themselves. */
static void RearmAfterStateChange()
{
    const float* ph = loop.Phys();
    for (uint8_t p = 0; p < NUM_POTS; p++)
    {
        routeCaught[p] = false;
        if (SecondaryOpen() && kSecEnabled[p]) InitCatch(secState[p], ph[p]);
        if (DiffusionOpen() && kDifEnabled[p]) InitCatch(difState[p], ph[p]);
        if (b3Active) underNorm[p] = pager.Stored(underPage, p);
    }
}

/* Knob values → engine, routing zones → mod sources. */
#ifdef CAPICOLA_BENCH_LOG
/* ── Profile stream (bench image) ──────────────────────────────────────
 * One CSV row every kProfileMs over the front USB (a plain serial port: no
 * HostLink), its header every kProfileHeaderRows rows. tools/profile.py
 * captures and reports it. Integers only (no float printf):
 *   loads     in 0.01 % of one block's real time (budget = core cycles a block)
 *   counts    since the previous row (keyframes, detector peaks/kept, punches,
 *             stereo follows, leash splices, front/back guard rescues)
 *   controls  x1000 (grain in keyframes and fade in samples as they are)
 *   cv        milliunits, 500 ~ 0 V (the bench jack-map probe)
 * Send "BOOT" to reboot into the bootloader (tools/profile.py flash). */
static constexpr uint32_t kProfileMs         = 50;
static constexpr uint32_t kProfileHeaderRows = 20;
static volatile bool      profileBootReq     = false;

static void OnUsbRx(uint8_t* buf, uint32_t* len)
{
    static const char kWord[] = "BOOT";
    static uint8_t    got     = 0;
    for (uint32_t i = 0; i < *len; i++)
    {
        got = (buf[i] == (uint8_t)kWord[got]) ? (uint8_t)(got + 1)
            : (buf[i] == (uint8_t)kWord[0]) ? 1 : 0;
        if (got == 4) { profileBootReq = true; got = 0; }
    }
}

/* Boot benchmark: before presets load, the real engine runs a fixed workload
 * (strikes, a tone and noise, stereo) at fixed settings, timed per stage, so
 * memory layouts and engine versions compare like for like. The stream repeats
 * the results as "#bench" rows with each header. */
struct BenchCase {
    const char* name;
    float pitch, stretch, grain, quality;
    float thresh;   // the detector's keep ratio (100 = muted: no hits, no onset searches)
    float hits;     // the strikes' level (0: the tone and noise alone)
    float feedback, diffusion;   // (the comb's cases: feedback up, the feedback and envelope page's P1)
    bool  pcm;                   // (the top of Quality)
    AlchemyLabAudio::AudioEngine::Profile pf;
};
static BenchCase benchCases[] = {
    { "unity",        1.00f, 1.0f, 642.0f, 0.001f,   2.0f, 0.7f, 0.0f, 0.0f, false, {} },
    { "pitch2_st05",  1.94f, 0.5f, 642.0f, 0.001f,   2.0f, 0.7f, 0.0f, 0.0f, false, {} },
    { "small_coarse", 0.70f, 0.3f, 128.0f, 0.020f,   2.0f, 0.7f, 0.0f, 0.0f, false, {} },
    { "unity_muted",  1.00f, 1.0f, 642.0f, 0.001f, 100.0f, 0.7f, 0.0f, 0.0f, false, {} },
    { "tone_only",    1.00f, 1.0f, 642.0f, 0.001f,   2.0f, 0.0f, 0.0f, 0.0f, false, {} },
    { "fb_nodiff",    1.00f, 1.0f, 642.0f, 0.001f,   2.0f, 0.7f, 0.9f, 0.0f, false, {} },
    { "fb_comb1",     1.00f, 1.0f, 642.0f, 0.001f,   2.0f, 0.7f, 0.9f, 1.0f, false, {} },
    { "unity_pcm",    1.00f, 1.0f, 642.0f, 0.001f,   2.0f, 0.7f, 0.0f, 0.0f, true,  {} },
    { "pitch2_pcm",   1.94f, 0.5f, 642.0f, 0.001f,   2.0f, 0.7f, 0.0f, 0.0f, true,  {} },
};

/* Micro benchmark (boot): each per-sample component alone, core cycles per
 * sample x100, on the same synthetic input. "#micro" rows in the stream. */
struct MicroCase { const char* name; uint32_t cps100; };
static MicroCase microCases[] = {
    { "analyzer", 0 }, { "detector", 0 }, { "detector_out", 0 }, { "bspline_full", 0 },
    { "fb_sinc", 0 }, { "fb_svf", 0 }, { "comb_off", 0 }, { "comb_on", 0 },   // (one channel's feedback chain)
};
#ifdef CAPICOLA_FB_DIFFUSER
static capicola::AdaptiveComb __attribute__((section(".d2_bss"))) microComb;
#endif
/* In AXI SRAM, not DTCM: DTCM holds the stack too, and 12 KB more of .bss there left the stack too small for
 * Shapers::Init's tables (the bench image overflowed it and died at boot, 2026-11-02). */
static capicola::Shapers __attribute__((section(".axi_bss"))) microShapers;
static capicola::SparseLine<65536> DSY_SDRAM_BSS microRing;
static capicola::Analyzer<65536> __attribute__((section(".axi_bss"))) microAn;
static capicola::Detector         __attribute__((section(".axi_bss"))) microDet;
static capicola::Detector         __attribute__((section(".axi_bss"))) microDetOut;

static void RunMicroBench()
{
    constexpr int N = 48000;
    static float x[1024];
    uint32_t rng = 777u;
    for (int i = 0; i < 1024; i++) {
        rng = rng * 1664525u + 1013904223u;
        x[i] = 0.3f * std::sin(6.2831853f * 220.0f * (float)i / 48000.0f)
             + ((float)(rng >> 9) / 4194304.0f - 1.0f) * 0.01f;
    }
    microRing.Init();
    microAn.Init(microRing, 48000.0f);
    microAn.SetThreshold(0.001f);
    microDet.Init(48000.0f);
    microDet.SetCutoff(0.002f);
    microDet.SetThreshold(2.0f);
    microDetOut.Init(48000.0f);
    microDetOut.SetCutoff(0.002f);
    capicola::DeluxeLine<float, 8> dl;
    dl.Init();
    volatile float sink = 0.0f;
    uint32_t t0 = capicola_cycles();
    for (int i = 0; i < N; i++) microAn.Analyze(x[i & 1023]);
    uint32_t t1 = capicola_cycles();
    for (int i = 0; i < N; i++) microDet.Analyze(x[i & 1023]);
    uint32_t t2 = capicola_cycles();
    for (int i = 0; i < N; i++) microDetOut.Analyze(x[i & 1023]);
    uint32_t t3 = capicola_cycles();
    for (int i = 0; i < N; i++) { dl.Write(x[i & 1023]); sink = sink + dl.ReadBSplineFull(2.0f, 1).tkeo; }
    uint32_t t4 = capicola_cycles();
    microShapers.Init();
    capicola::StateVariable svf;
    svf.Init();
    svf.SetControls(0.02f, 0.01f);
    const uint32_t s0 = capicola_cycles();
    for (int i = 0; i < N; i++) sink = sink + microShapers.ReadSinc(x[i & 1023]);
    const uint32_t s1 = capicola_cycles();
    for (int i = 0; i < N; i++) { svf.Tick(x[i & 1023]); sink = sink + svf.GetNormBandpass(); }
    const uint32_t s2 = capicola_cycles();
    uint32_t s3 = s2, s4 = s2;
#ifdef CAPICOLA_FB_DIFFUSER
    microComb.Init();
    for (int i = 0; i < N; i++) {
        if ((i & 63) == 0) { microComb.SetKeyframes(12.0f, 7700.0f); microComb.Block(64); }
        sink = sink + microComb.Process(x[i & 1023]);
    }
    s3 = capicola_cycles();
    microComb.SetSize(1.0f);
    for (int i = 0; i < N; i++) {
        if ((i & 63) == 0) { microComb.SetKeyframes(12.0f, 7700.0f); microComb.Block(64); }
        sink = sink + microComb.Process(x[i & 1023]);
    }
    s4 = capicola_cycles();
#endif
    microCases[0].cps100 = (uint32_t)((uint64_t)(t1 - t0) * 100u / N);
    microCases[1].cps100 = (uint32_t)((uint64_t)(t2 - t1) * 100u / N);
    microCases[2].cps100 = (uint32_t)((uint64_t)(t3 - t2) * 100u / N);
    microCases[3].cps100 = (uint32_t)((uint64_t)(t4 - t3) * 100u / N);
    microCases[4].cps100 = (uint32_t)((uint64_t)(s1 - s0) * 100u / N);
    microCases[5].cps100 = (uint32_t)((uint64_t)(s2 - s1) * 100u / N);
    microCases[6].cps100 = (uint32_t)((uint64_t)(s3 - s2) * 100u / N);
    microCases[7].cps100 = (uint32_t)((uint64_t)(s4 - s3) * 100u / N);
}

static void RunBootBench()
{
    RunMicroBench();
    using AlchemyLabAudio::AudioEngine;
    constexpr int kBlocks = 1500;   // 2 s
    constexpr int kWarmup = 16;     // not timed
    const int B = (int)AUDIO_BLOCK_SAMPLES;
    static float il[AUDIO_BLOCK_SAMPLES], ir[AUDIO_BLOCK_SAMPLES];
    static float ol[AUDIO_BLOCK_SAMPLES], orr[AUDIO_BLOCK_SAMPLES];
    for (BenchCase& bc : benchCases)
    {
        engine.Init();
        engine.SetSlicePitch(bc.pitch);
        engine.SetSliceStretch(bc.stretch);
        engine.SetGrainSize(bc.grain);
        engine.SetQuality(bc.quality);
        engine.SetPcm(bc.pcm);
        engine.SetFade(960.0f);
        engine.SetTransientThreshold(bc.thresh);
        engine.SetMix(1.0f);
        engine.SetFeedback(bc.feedback);
        engine.SetCombSize(bc.diffusion);
        uint32_t rng = 12345u;
        int n = 0;
        AudioEngine::Profile drop;
        engine.TakeProfile(drop);
        for (int b = 0; b < kBlocks; b++)
        {
            for (int i = 0; i < B; i++, n++)
            {
                rng = rng * 1664525u + 1013904223u;
                const float noise = ((float)(rng >> 9) / 4194304.0f - 1.0f) * 0.002f;
                const int   j     = n % 12000;   // a strike every 250 ms
                const float tj    = (float)j / 48000.0f;
                const float hit   = bc.hits * std::exp(-tj * 25.0f)
                                  * std::sin(6.2831853f * 180.0f * tj);
                const float tone  = 0.15f * std::sin(6.2831853f * 220.0f * (float)n / 48000.0f);
                il[i] = hit + tone + noise;
                ir[i] = 0.8f * hit - tone + noise;
            }
            engine.ProcessBlock(il, ir, ol, orr, (std::size_t)B);
            if (b == kWarmup - 1) engine.TakeProfile(drop);   // (the start clears the ring)
        }
        engine.TakeProfile(bc.pf);
    }
    engine.Init();   // (a clean engine for the real start)
}

/* One transfer to the host. A CDC transfer of an exact multiple of the 64-byte
 * packet needs a zero-length packet to end it, which this stack never sends: the
 * port then stays busy for good (the first session's stream stopped after 3 s).
 * Such a line gets one more space before its \r\n. */
static bool ProfileSend(daisy::UsbHandle& usb, char* buf, int n, int cap)
{
    if (n <= 0) return false;
    if (n % 64 == 0 && n + 1 < cap && n >= 2)
    {
        buf[n + 1] = buf[n - 1];   // '\n'
        buf[n]     = buf[n - 2];   // '\r'
        buf[n - 2] = ' ';
        n++;
    }
    return usb.TransmitExternal((uint8_t*)buf, (size_t)n) == daisy::UsbHandle::Result::OK;
}

static void ProfileTick(uint32_t t)
{
    using AlchemyLabAudio::AudioEngine;
    static uint32_t last = 0, rows = 0, dropped = 0;
    static capicola::RecorderStats pl = {}, pr = {};
    static char row[768];
    if (profileBootReq)   // (an explicit flash request: the bootloader waits for it)
        daisy::System::ResetToBootloader(daisy::System::BootloaderMode::DAISY_INFINITE_TIMEOUT);
    if (t - last < kProfileMs) return;
    last = t;

    daisy::UsbHandle usb;   // (stateless: the logger's port)
    if (rows % kProfileHeaderRows == 0)
    {
        static char kHeader[] =
            "t_ms,blocks,budget,dropped,total,total_max,feedback,cap_l,render_l,cap_r,render_r,"
            "align,post,cap_l_max,render_l_max,cap_r_max,render_r_max,"
            "kf_l,kf_r,det_l,det_r,kept_l,kept_r,punch_l,punch_r,follow_l,follow_r,"
            "splice_l,splice_r,front_l,front_r,back_l,back_r,silent_l,silent_r,"
            "pitch,stretch,thresh,grain,mix,feedback_amt,fade,quality,"
            "cv0,cv1,cv2,cv3,cv4,cv5,"
            "nan_fb,nan_out,trips,diffusion,spread,diff_amt,period_l,period_r,nested,pcm\r\n";
        ProfileSend(usb, kHeader, (int)sizeof(kHeader) - 1, (int)sizeof(kHeader));
        daisy::System::Delay(2);
        /* #bench,case,blocks,budget,total,total_max,feedback,cap_l,render_l,cap_r,render_r,
         *        align,post,cap_l_max,render_l_max,cap_r_max,render_r_max  (0.01 %) */
        const uint64_t bb = AudioEngine::BlockBudgetCycles();
        for (const BenchCase& bc : benchCases)
        {
            const AudioEngine::Profile& q = bc.pf;
            const uint64_t nb = q.blocks ? q.blocks : 1;
            auto a = [&](int k) { return (unsigned)(q.sum[k] * 10000u / (nb * bb)); };
            auto m = [&](int k) { return (unsigned)((uint64_t)q.max[k] * 10000u / bb); };
            const int len = std::snprintf(row, sizeof(row),
                "#bench,%s,%lu,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                bc.name, (unsigned long)q.blocks, (unsigned long)bb,
                a(AudioEngine::kStTotal), m(AudioEngine::kStTotal), a(AudioEngine::kStFeedback),
                a(AudioEngine::kStCapL), a(AudioEngine::kStRenderL),
                a(AudioEngine::kStCapR), a(AudioEngine::kStRenderR),
                a(AudioEngine::kStAlign), a(AudioEngine::kStPost),
                m(AudioEngine::kStCapL), m(AudioEngine::kStRenderL),
                m(AudioEngine::kStCapR), m(AudioEngine::kStRenderR));
            ProfileSend(usb, row, len, (int)sizeof(row));
            daisy::System::Delay(2);
        }
        for (const MicroCase& mc : microCases)   // #micro,name,cycles_per_sample_x100
        {
            const int len = std::snprintf(row, sizeof(row), "#micro,%s,%lu\r\n",
                                          mc.name, (unsigned long)mc.cps100);
            ProfileSend(usb, row, len, (int)sizeof(row));
            daisy::System::Delay(2);
        }
    }
    rows++;

    AudioEngine::Profile pf;
    engine.TakeProfile(pf);
    const capicola::RecorderStats l = engine.StatsL(), r = engine.StatsR();
    const AudioEngine::Controls&  c = engine.GetControls();
    const uint64_t budget = AudioEngine::BlockBudgetCycles();
    const uint32_t nb     = pf.blocks ? pf.blocks : 1;
    auto avg = [&](int k) { return (unsigned)(pf.sum[k] * 10000u / ((uint64_t)nb * budget)); };
    auto mx  = [&](int k) { return (unsigned)((uint64_t)pf.max[k] * 10000u / budget); };
    auto k1  = [](float v) { return (int)(v * 1000.0f + (v < 0.0f ? -0.5f : 0.5f)); };
    const float* cv = loop.Cv();
    const int n = std::snprintf(row, sizeof(row),
        "%lu,%lu,%lu,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,"
        "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%d,%d,"
        "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
        "%lu,%lu,%lu,%d,%d,%d,%d,%d,%lu,%d\r\n",
        (unsigned long)t, (unsigned long)pf.blocks, (unsigned long)budget, (unsigned long)dropped,
        avg(AudioEngine::kStTotal), mx(AudioEngine::kStTotal), avg(AudioEngine::kStFeedback),
        avg(AudioEngine::kStCapL), avg(AudioEngine::kStRenderL),
        avg(AudioEngine::kStCapR), avg(AudioEngine::kStRenderR),
        avg(AudioEngine::kStAlign), avg(AudioEngine::kStPost),
        mx(AudioEngine::kStCapL), mx(AudioEngine::kStRenderL),
        mx(AudioEngine::kStCapR), mx(AudioEngine::kStRenderR),
        (unsigned long)(l.keyframes - pl.keyframes), (unsigned long)(r.keyframes - pr.keyframes),
        (unsigned long)(l.detected - pl.detected), (unsigned long)(r.detected - pr.detected),
        (unsigned long)(l.kept - pl.kept), (unsigned long)(r.kept - pr.kept),
        (unsigned long)(l.punches - pl.punches), (unsigned long)(r.punches - pr.punches),
        (unsigned long)(l.follows - pl.follows), (unsigned long)(r.follows - pr.follows),
        (unsigned long)(l.splices - pl.splices), (unsigned long)(r.splices - pr.splices),
        (unsigned long)(l.fronts - pl.fronts), (unsigned long)(r.fronts - pr.fronts),
        (unsigned long)(l.backs - pl.backs), (unsigned long)(r.backs - pr.backs),
        (int)l.silent, (int)r.silent,
        k1(c.pitch), k1(c.stretch), k1(c.thresh), (int)(c.grain + 0.5f), k1(c.mix),
        k1(c.feedback), (int)(c.fade + 0.5f), k1(c.quality),
        k1(cv[0]), k1(cv[1]), k1(cv[2]), k1(cv[3]), k1(cv[4]), k1(cv[5]),
        (unsigned long)pf.nanFb, (unsigned long)pf.nanOut, (unsigned long)pf.trips,
        k1(c.diffusion), k1(c.spread), k1(engine.DiffAmount()),
        (int)(engine.DiffPeriod(0) + 0.5f), (int)(engine.DiffPeriod(1) + 0.5f), (unsigned long)pf.nested,
        engine.PcmOn() ? 1 : 0);
    pl = l;
    pr = r;
    if (n > 0 && !ProfileSend(usb, row, n, (int)sizeof(row)))
        dropped++;
}
#endif

static void OnFrame()
{
    /* A preset landed since last frame — from the panel, boot or the
     * programmer. Before anything below re-asserts the old values. */
    if (stateApplied)
    {
        stateApplied = false;
        RearmAfterStateChange();
    }

    /* B3 view changes: arm the incoming page's catch, re-arm B1's on the way out. */
    {
        static bool    prevActive = false;
        static uint8_t prevPage   = kB3Secondary;
        const float*   ph         = loop.Phys();

        /* Pager::Update runs before this hook and tracks the caught pot, so the
         * B1 page underneath follows the knob unless we hold it. Snapshot on
         * entry, re-assert every frame; the pot locks out once it moves past
         * kCatchTolerance. */
        /* The feedback and envelope page's P2 mirrors Secondary P6's stored value (a preset load, the reset, a HostLink write)
         * while the page is closed; open, its pot writes it back (DifApply). */
        if (!DiffusionOpen()) difState[1].stored = secState[5].stored;

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
            else if (SecondaryOpen() || DiffusionOpen())
            {
                const bool* en;
                PotState*   st = SettingsState(en);
                for (uint8_t p = 0; p < NUM_POTS; p++)
                    if (en[p]) InitCatch(st[p], ph[p]);
            }
            else
                pager.LockPage(pager.Page(), ph);
        }

        {
            const bool* en;
            if (PotState* st = SettingsState(en))
                for (uint8_t p = 0; p < NUM_POTS; p++)
                    if (en[p]) UpdateCatch(st[p], ph[p]);
        }

        /* Push every changed secondary, whoever moved it — a caught pot, a
         * preset load, the B1+B3 reset, or a HostLink live write. Gating this
         * on SecondaryOpen() used to strand anything the panel didn't move.
         * Seeded out of range so the first frame always pushes. */
        static float applied[NUM_POTS] = {-1.f, -1.f, -1.f, -1.f, -1.f, -1.f};
        for (uint8_t p = 0; p < NUM_POTS; p++)
        {
            if (secState[p].stored == applied[p]) continue;   // (all six: P6 is turned from the feedback and envelope page)
            applied[p] = secState[p].stored;
            SecApply(p, SecValue(p, secState[p].stored));
        }
        static float difApplied[NUM_POTS] = {-1.f, -1.f, -1.f, -1.f, -1.f, -1.f};
        for (uint8_t p = 0; p < NUM_POTS; p++)
        {
            if (!kDifEnabled[p] || difState[p].stored == difApplied[p]) continue;
            difApplied[p] = difState[p].stored;
            DifApply(p, difState[p].stored);
        }

        prevActive = b3Active;
        prevPage   = b3Page;
    }

    /* B3 held is the SHIFT layer on every page but Routing and Depth (there it
     * is control all, below), and it is binary: each knob
     * holds still and only toggles between where it was when the hold armed
     * and its snap target (factory, or the save point). A nudge toward the
     * target snaps there, and a nudge back returns.
     * All six knobs, P1/P2 included. Same momentum test as the gesture,
     * re-based on each nudge. A knob is only held once its pot has moved, so
     * untouched rings don't dim on every hold. A knob left at its default
     * waits behind the catch pip after release. */
    {
        static bool  prev = false;
        static float start  [NUM_POTS];   // pot position when the hold armed
        static float ref    [NUM_POTS];   // pot position the nudge is measured from
        static float orig   [NUM_POTS];   // value when the hold armed
        static bool  atDef  [NUM_POTS];
        static bool  touched[NUM_POTS];
        const float* ph = loop.Phys();
        const bool   snap = b3HoldArmed && !RoutingOpen() && !DepthOpen();
        if (snap)
            for (uint8_t p = 0; p < NUM_POTS; p++)
            {
                float cur, def;
                if (!SnapTarget(p, cur, def)) continue;
                if (!prev)
                {
                    start[p] = ref[p] = ph[p];
                    orig[p]    = cur;
                    atDef[p]   = false;
                    touched[p] = false;
                }
                if (std::fabs(ph[p] - start[p]) >= kCatchTolerance) touched[p] = true;
                const float d = ph[p] - ref[p];
                if (d <= -kGestureNudge || d >= kGestureNudge)
                {
                    ref[p]    = ph[p];
                    atDef[p]  = (def - orig[p]) * d > 0.0f;   // toward the default
                    shiftUsed = true;
                }
                if (touched[p]) HoldValue(p, atDef[p] ? def : orig[p], ph);
            }
        prev = snap;
    }

    /* Control all (heavylight, 2026-11-02, as on the Digitakt): B3 held on Routing or Depth, the first knob you turn
     * leads and the other five follow, each by the same amount from where it stood (Depth: the same change of depth,
     * here; Routing: the same number of places on the wheel, in the routing block below), each stopping at its own
     * end. The leader only moves once caught, its pip as always; the followers stay uncaught after the hold. */
    {
        static bool   prev = false;
        static float  ref [NUM_POTS];   // each depth where it stood when the hold (or its latest leader) began
        static int8_t lead = -1;
        const bool   all = b3HoldArmed && DepthOpen();
        const float* ph  = loop.Phys();
        if (all && !prev)
        {
            lead = -1;
            for (uint8_t p = 0; p < NUM_POTS; p++) ref[p] = pager.Stored(1, p);
        }
        if (all)
        {
            for (uint8_t p = 0; p < NUM_POTS; p++)
                if ((int8_t)p != lead && pager.Caught(p) && std::fabs(pager.Stored(1, p) - ref[p]) >= kGestureNudge)
                {
                    for (uint8_t q = 0; q < NUM_POTS; q++)
                        if (q != p) ref[q] = pager.Stored(1, q);
                    lead = (int8_t)p;
                }
            if (lead >= 0)
            {
                const float d = pager.Stored(1, (uint8_t)lead) - ref[lead];
                for (uint8_t p = 0; p < NUM_POTS; p++)
                {
                    if ((int8_t)p == lead) continue;
                    float v = ref[p] + d;
                    v = v < 0.0f ? 0.0f : v > 1.0f ? 1.0f : v;
                    HoldValue(p, v, ph);
                }
                shiftUsed = true;
            }
        }
        prev = all;
    }

    /* B2 held on the depth page (heavylight, 2026-11-02): nudge a knob and its depth snaps to zero, held there
     * until B2 lets go (then behind its catch pip, as after a snap). A hold that zeroed any doesn't slice. */
    {
        static bool  prev = false;
        static float ref[NUM_POTS];
        static bool  zeroed[NUM_POTS];
        const float* ph = loop.Phys();
        if (depthHeld)
            for (uint8_t p = 0; p < NUM_POTS; p++)
            {
                if (!prev) { ref[p] = ph[p]; zeroed[p] = false; }
                if (!zeroed[p] && std::fabs(ph[p] - ref[p]) >= kGestureNudge) { zeroed[p] = true; depthZeroed = true; }
                if (zeroed[p]) HoldValue(p, kPageDefaults[1][p], ph);
            }
        prev = depthHeld;
    }

    /* Pitch carries the same B2 gesture as Stretch. Value() is the modulated
     * speed; the gesture works in norms, and the map is linear, so convert. */
    {
        const float n    = (pitch.Value() + kPitchMax) / (2.0f * kPitchMax);
        const float eff  = GestureNorm(pitchG, n, loop.Phys()[0],
                                       flipHeld, PitchMag, PitchNormFor);
        engine.SetSlicePitch((eff < 0.5f) ? -pitchG.liveMag : pitchG.liveMag);
    }

    /* The primaries reach the engine on every page (heavylight, 2026-11-02: on a settings page their modulation
     * froze until it closed). The pots don't move them there: the B1 page under a B3 page is held (underNorm). */
    {
        /* P2 carries the same B2 gesture as P1: hold to flip the grid's
         * direction at a constant stretch, release and the knob is page-
         * switched until its catch pip is consumed. */
        {
            const float eff  = GestureNorm(stretchG, stretch.Value(), loop.Phys()[1],
                                           flipHeld, StretchMag, StretchNormFor);
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
        /* The top of Quality (PCM) is the knob's own place, never its modulation's: below it, modulation stops
         * short of the boundary (ε no finer than kPcmBelow), so it can't push the sound in and out of PCM. */
        const bool pcmTop = pager.Stored(0, 4) >= kPcmTop;
        const float eps   = quality.Value();
        engine.SetQuality  (pcmTop || eps >= kPcmBelow ? eps : kPcmBelow);   // analyzer ε
        engine.SetPcm      (pcmTop);
        engine.SetFeedback (feedback.Value());     // filtered feedback
    }

    modRouter.SetDepth(0, cvDepth1.Value());
    modRouter.SetDepth(1, cvDepth2.Value());
    modRouter.SetDepth(2, cvDepth3.Value());
    modRouter.SetDepth(3, cvDepth4.Value());
    modRouter.SetDepth(4, cvDepth5.Value());
    modRouter.SetDepth(5, cvDepth6.Value());

    /* Polarity flips land in the pager; P1/P2's rings read its catch back. */
    for (uint8_t p = 0; p < 2; p++)
    {
        GestureKnob& g = p ? stretchG : pitchG;
        if (g.commit)
        {
            g.commit = false;
            pager.SetStored(0, p, g.heldNorm, loop.Phys());
        }
        g.view.stored = pager.Stored(0, p);
        g.view.caught = (pager.Page() == 0) ? pager.Caught(p) : true;
    }

    /* Routing view open: pots edit the stored zones with zone-granular catch —
     * a pot decouples on entry and re-locks the moment it sits inside its
     * stored zone. The pager page underneath is re-locked every frame so those
     * primaries can't catch while their pots are steering zones. */
    if (RoutingOpen())
    {
        const float* ph = loop.Phys();
        uint8_t l[kMaxSrcZones];
        const int n = Sources(l);
        auto place = [&](uint8_t id) { int i = 0; while (i < n - 1 && l[i] != id) i++; return (l[i] == id) ? i : 0; };
        static bool   prev = false;
        static int    from[NUM_POTS];   // control all: each knob's place when the hold (or its latest leader) began
        static int8_t lead = -1;
        const bool all = b3HoldArmed;
        if (all && !prev)
        {
            lead = -1;
            for (uint8_t p = 0; p < NUM_POTS; p++) from[p] = place(EffectiveZone(routing.zone[p]));
        }
        for (uint8_t p = 0; p < NUM_POTS; p++)
        {
            const uint8_t z = SourceAt(ph[p]);
            if (!routeCaught[p] && z == EffectiveZone(routing.zone[p]))
                routeCaught[p] = true;
            if (!routeCaught[p]) continue;
            if (all && (int8_t)p != lead && z != EffectiveZone(routing.zone[p]))
            {
                for (uint8_t q = 0; q < NUM_POTS; q++)
                    if (q != p) from[q] = place(EffectiveZone(routing.zone[q]));
                from[p] = place(EffectiveZone(routing.zone[p]));
                lead = (int8_t)p;
            }
            routing.zone[p] = z;
        }
        if (all && lead >= 0)
        {
            const int d = place(routing.zone[lead]) - from[lead];
            for (uint8_t p = 0; p < NUM_POTS; p++)
            {
                if ((int8_t)p == lead) continue;
                int i = from[p] + d;
                i = i < 0 ? 0 : i > n - 1 ? n - 1 : i;
                routing.zone[p] = l[i];
                routeCaught[p]  = false;
            }
            shiftUsed = true;
        }
        prev = all;
    }

    for (uint8_t p = 0; p < NUM_POTS; p++)
        modRouter.SetSource(p, EffectiveZone(routing.zone[p]));

#ifdef CAPICOLA_BENCH_LOG
    ProfileTick(daisy::System::GetNow());
#endif
}

/* 1 ms poll: B2 force trigger, B3 tap/hold tracking + the four CV outs. Trig
 * outs sit on the MCP4728 (I²C) so they write only on pulse edges; envelope
 * outs sit on the STM DAC and stream every poll. */
static uint32_t sliceFlashMs = 0;   // last button-originated trigger

static void OnPoll(uint32_t t_ms)
{
    /* Slot 0 changes behind our back too (the programmer can write it), so
     * the save-point snapshot follows its record's sequence number. */
    {
        static uint32_t lastSeq   = 0xFFFFFFFFu;
        static uint32_t lastCheck = 0;
        if (t_ms - lastCheck >= 100u)
        {
            lastCheck = t_ms;
            const uint32_t seq = presets.InfoFor(0).seq;
            if (seq != lastSeq) { lastSeq = seq; RefreshSaved(); }
        }
    }

    const bool b1 = hw.buttons[0].Pressed();
    const bool b2 = hw.buttons[1].Pressed();
    const bool b3 = hw.buttons[2].Pressed();

    /* B1+B2 held kSaveHoldMs = save everything (all four pages) as the boot
     * state, QSPI slot 0; the save it replaces joins the history. Not while
     * B2 is already a hold. */
    static uint32_t saveHoldMs = 0;
    if (b1 && b2 && !b3 && !b2HoldArmed)
    {
        pager.ConsumeButton();   // B1 is the pager now — eat its release
        if (++saveHoldMs == kSaveHoldMs)
        {
            NameThisSave();
            SaveToHistory();
            ConfirmFlash();
        }
    }
    else
        saveHoldMs = 0;

    /* B2+B3 held kLoadHoldMs = load the boot state back. An empty slot does
     * nothing and doesn't blink. Not while B2 or B3 is already a hold. Secondaries
     * reach the engine through OnFrame's push. */
    static uint32_t loadHoldMs = 0;
    if (b2 && b3 && !b1 && !b2HoldArmed && !b3HoldArmed)
    {
        if (++loadHoldMs == kLoadHoldMs && presets.HasValid(0) && presets.Load(0))
            ConfirmFlash();   // catch re-arms on the next frame (stateApplied)
    }
    else
        loadHoldMs = 0;

    /* B1+B3 held kResetHoldMs = factory defaults on every page, so there is
     * always one way home. Stored values only — the QSPI save is untouched, so
     * B2+B3 undoes it. */
    static uint32_t resetHoldMs = 0;
    if (b1 && b3 && !b2 && !b3HoldArmed)
    {
        if (++resetHoldMs == kResetHoldMs)
        {
            const float* ph = loop.Phys();
            for (uint8_t p = 0; p < NUM_POTS; p++)
            {
                for (uint8_t pg = 0; pg < 2; pg++)
                    pager.SetStored(pg, p, kPageDefaults[pg][p], ph);
                routing.zone[p]    = 0;                // input follower
                secState[p].stored = kSecDefault[p];
                difState[p].stored = kDifDefault[p];
            }
            RearmAfterStateChange();
            ConfirmFlash();
        }
    }
    else
        resetHoldMs = 0;

    /* B2 = force trigger, on release, so a press can still become a chord or a
     * hold. A chorded release never slices, and neither does a hold that
     * flipped P1/P2. A forced slice is a real slice, so it pulses TRIG 1
     * exactly like an auto-detected one. Held past kHoldArmMs alone it is the
     * polarity flip hold, live only on the Primary page where P1/P2 are Pitch and
     * Stretch; once armed it stays a hold until release. */
    static uint32_t b2HoldMs = 0;
    if (hw.buttons[1].RisingEdge())
    {
        b2Chorded       = b1 || b3;
        depthZeroed     = false;
        pitchG.nudged   = false;   // cleared here, not on release: OnFrame's
        stretchG.nudged = false;   // decouple still has to read it
    }
    if (b2 && (b1 || b3))
        b2Chorded = true;
    if (b2 && !b2Chorded && !b2HoldArmed && ++b2HoldMs >= kHoldArmMs)
        b2HoldArmed = true;
    flipHeld = b2HoldArmed && b2 && !b3Active && pager.Page() == 0;
    depthHeld = b2HoldArmed && b2 && DepthOpen();
    if (hw.buttons[1].FallingEdge())
    {
        if (!b2Chorded && !pitchG.nudged && !stretchG.nudged && !depthZeroed)
        {
            engine.TriggerSlice();
            trig1.Fire(t_ms);
            sliceFlashMs = t_ms;
        }
        b2Chorded   = false;
        b2HoldArmed = false;
        b2HoldMs    = 0;
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

    /* B3 rotates its own page group on release: first press opens on the
     * remembered page, each press after that advances. A chorded press is never
     * a page press. Held past kHoldArmMs alone it is the SHIFT layer (snap, in
     * OnFrame), or control all on Routing and Depth; once armed it stays a hold
     * until release, and a hold that moved a knob doesn't page — one that didn't
     * pages like a slow tap. */
    static uint32_t b3HoldMs = 0;
    if (hw.buttons[2].RisingEdge())
    {
        b3Chorded = b1 || b2;
        shiftUsed = false;
    }
    if (b3 && (b1 || b2))
        b3Chorded = true;
    if (b3 && !b3Chorded && !b3HoldArmed && ++b3HoldMs >= kHoldArmMs)
        b3HoldArmed = true;
    if (hw.buttons[2].FallingEdge())
    {
        if (!b3Chorded && !shiftUsed)
        {
            if (b3Active) b3Page = kB3Next[b3Page];
            else          b3Active = true;
        }
        b3Chorded   = false;
        b3HoldArmed = false;
        b3HoldMs    = 0;
    }

    /* B1 while a B3 page is up returns to B1's remembered page instead of
     * advancing it — the pager never sees the release. */
    if (hw.buttons[0].RisingEdge())
        b1Chorded = b2 || b3;
    if (b1)
    {
        if (b2 || b3)
        {
            b1Chorded = true;
            pager.ConsumeButton();   // a chord, or B1 under a B2 hold: never a page press
        }
        if (b3Active)
            pager.ConsumeButton();
    }
    if (hw.buttons[0].FallingEdge())
    {
        if (b3Active && !b1Chorded) b3Active = false;
        b1Chorded = false;
    }

    /* Trig 1 / Env 1 direction follows the option, whoever set it — panel
     * load, boot, or the programmer. Every output is rewritten from scratch
     * after a change. */
    static bool  jacksApplied = false;
    static float written[jacks::kNumJacks] = {-1.0f, -1.0f, -1.0f, -1.0f};
    if (jackOpts.cvIn != jacksApplied)
    {
        jacksApplied = jackOpts.cvIn;
        if (jackOpts.cvIn)
        {
            hw.cv_jacks[2].DisableCvOutput();   // TRIG 1 → CV 2
            hw.cv_jacks[3].DisableCvOutput();   // ENV 1  → CV 3
        }
        else
        {
            hw.cv_jacks[2].SetVolts(0.0f);
            hw.cv_jacks[3].SetVolts(0.0f);
            hw.cv_jacks[2].EnableCvOutput();
            hw.cv_jacks[3].EnableCvOutput();
        }
        for (float& w : written) w = -1.0f;
    }

#ifndef CAPICOLA_BENCH_LOG
    /* The programmer names each jack by its current role: republish the jacks
     * and refresh the descriptor whenever the jack options change. */
    {
        static jacks::Options described;   // DescribeManual published the factory roles
        if (jackOpts.cvIn != described.cvIn || jackOpts.trig2 != described.trig2
            || jackOpts.env2 != described.env2 || jackOpts.outFollower != described.outFollower)
        {
            described = jackOpts;
            DescribeJacks(host, jackOpts);
            host.RequestDescriptorRefresh();
        }
    }
#endif

    /* TRIG 1 = every slice the engine actually took (auto-detected, B2 and TRIG
     * IN alike); TRIG 2 = the output follower's kept transients. Which jack
     * carries which is jacks::Route's call. */
    trig1.Follow(engine.SliceCount(),    t_ms);
    trig2.Follow(engine.OutEventCount(), t_ms);

    jacks::Signals sig;
    sig.inTrig  = trig1.High(t_ms);
    sig.outTrig = trig2.High(t_ms);
    sig.inEnv   = engine.EnvNormIn();
    sig.outEnv  = engine.EnvNormOut();
    sig.inCont  = engine.ContourIn();
    sig.outCont = engine.ContourOut();
    sig.inPuls  = engine.PulseIn();
    sig.outPuls = engine.PulseOut();
    const jacks::Plan plan = jacks::Route(jackOpts, sig);

    /* Trig 1, Env 1 and Trig 2 sit on the I²C MCP4728 — write only on a real
     * change (a trigger edge is 5 V). Env 2 is the STM32 DAC and streams. */
    CvJack* const out[jacks::kNumJacks] = {&hw.cv_jacks[2], &hw.cv_jacks[3],
                                           &hw.cv_jacks[4], &hw.cv_jacks[5]};
#ifdef CAPICOLA_NO_OUTS
    /* The click hunt: the trigger and envelope outputs never written (do their edges reach the audio?) */
    for (uint8_t j = 0; j < jacks::kNumJacks && false; j++)
#else
    for (uint8_t j = 0; j < jacks::kNumJacks; j++)
#endif
    {
        if (!plan.output[j]) continue;
        if (j != jacks::kEnv2 && std::fabs(plan.volts[j] - written[j]) <= 0.02f) continue;
        written[j] = plan.volts[j];
        out[j]->SetVolts(plan.volts[j]);
    }
}

/* Always-on layer, drawn last every frame regardless of page:
 *   B1/B2/B3   wear the mode color (blue while the routing view is open);
 *              activity flashes the LED OFF.
 *   P3..P6 6pm the slices, CONT, TKEO and PULS (the factory jacks' signals). The
 *              slice pip runs off the same events as its jack, held kTrigLedMs so a
 *              10 ms pulse is visible. The TKEO pip turns red on input clip. */
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
        {
            const uint8_t z = EffectiveZone(routing.zone[p]);
            DrawRouteRing(p, z, std::fabs(modRouter.Signal(z)));
        }

    constexpr LedPanel::Rgb kCvNeg = {255, 10, 0};    // red

    /* The pips (heavylight, 2026-11-02): P3 the slices, P4 the input's contour, P5 TKEO, P6 the input's pulses, as
     * levels (folded, by magnitude), so the modulators show on every page. */
    const LedPanel::Rgb w = hw.leds.ScaleGlobal(kWhite);
    if (trig1.Lit(t_ms)) hw.leds.SetRingByHour(2, 6.0f, w);
    hw.leds.SetRingByHour(3, 6.0f,
        hw.leds.ScaleGlobal(LedPanel::Scale(kWhite, std::fabs(engine.ContourIn()))));

    /* in-env pip: white by envelope, red when the input clips. */
    constexpr float kClipAt = 0.98f;
    const LedPanel::Rgb inEnv = (engine.InPeak() >= kClipAt)
        ? kCvNeg
        : LedPanel::Scale(kWhite, engine.EnvNormIn());
    hw.leds.SetRingByHour(4, 6.0f, hw.leds.ScaleGlobal(inEnv));
    hw.leds.SetRingByHour(5, 6.0f,
        hw.leds.ScaleGlobal(LedPanel::Scale(kWhite, std::fabs(engine.PulseIn()))));

    /* Secondary page: repaint every ring over the B1 page underneath. The 6pm CV
     * pips above are untouched (these arcs span 7:30→4:30, never 6:00). */
    if (SecondaryOpen())
        DrawSettingsRings(secState, kSecEnabled, kTeal, t_ms);
    if (DiffusionOpen())
        DrawSettingsRings(difState, kDifEnabled, kIndigo, t_ms);

    /* Save / load / reset confirm: all three buttons flash bright white three
     * times. Painted last so it overrides the normal button layer. */
    const uint32_t flashAge = t_ms - saveFlashMs;
    if (saveFlashMs != 0 && flashAge < kSaveFlashMs
        && ((flashAge / kSaveBlinkMs) & 1u) == 0u)
    {
        const LedPanel::Rgb confirm = hw.leds.ScaleGlobal(kWhite);
        hw.leds.SetButtonPair(0, confirm);
        hw.leds.SetButtonPair(1, confirm);
        hw.leds.SetButtonPair(2, confirm);
    }
}

/* ── Audio callback ──────────────────────────────────────────────────── */

static void AudioCallback(daisy::AudioHandle::InputBuffer  in,
                          daisy::AudioHandle::OutputBuffer out,
                          size_t                           size)
{
#ifdef CAPICOLA_PROFILE
    /* The probe: a callback begun while one is still running (re-entered). */
    static volatile int depth = 0;
    if (++depth > 1) engine.CountNested();
#endif
    engine.ProcessBlock(in[0], in[1], out[0], out[1], size);
#ifdef CAPICOLA_PROFILE
    depth--;
#endif
}

/* ── Main ────────────────────────────────────────────────────────────── */

int main(void) {
    hw.Init(daisy::SaiHandle::Config::SampleRate::SAI_48KHZ, AUDIO_BLOCK_SAMPLES);
    engine.Init();
#ifdef CAPICOLA_BENCH_LOG
    RunBootBench();
#endif

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
    {
        secState[p].stored = kSecDefault[p];
        difState[p].stored = kDifDefault[p];   // (never loaded: not in the store)
    }

    pager.SetPageColor(0, kOrange);
    pager.SetPageColor(1, kPink);

    presets.Manage(pager);
    presets.Manage(routing);     /* mod-source zones */
    presets.Manage(secondary);   /* secondary-settings norms */
    presets.Manage(options);     /* knobless settings — SHIFT snap target */
    presetName = &presets.UseNames();   /* names ride the store; panel saves number themselves */

    loop.Use(pager)
        .Use(modRouter)
        .Use(perfPage)
        .Use(depthPage)
        .OnFrame(OnFrame)
        .OnPoll(OnPoll)
        .OnRender(OnRender);
#ifndef CAPICOLA_BENCH_LOG
    host.DescriptorBuffer(hostDescriptor, sizeof(hostDescriptor))
        .DescriptorRefreshBuffer(hostRefresh, sizeof(hostRefresh));
    // Custom projections edit one field and rearm only its panel pickup.
    host.Fast(pager, hostlink::FastProjection{
        0, static_cast<uint16_t>(pager.SerializedSize()), nullptr,
        [](void*, uint8_t* out, uint16_t) { pager.Serialize(out); },
        [](void*, uint16_t offset, const uint8_t* data, uint16_t size) {
            return capicola::host_controls::SetPager(pager, offset, data, size,
                loop.Phys(), b3Active, underPage, underNorm);
        }
    });
    host.Fast(routing, hostlink::FastProjection{
        0, static_cast<uint16_t>(routing.SerializedSize()), nullptr,
        [](void*, uint8_t* out, uint16_t) { routing.Serialize(out); },
        [](void*, uint16_t offset, const uint8_t* data, uint16_t size) {
            return capicola::host_controls::SetRouting(routing.zone, routeCaught,
                kMaxSrcZones, offset, data, size);
        }
    });
    host.Fast(secondary, hostlink::FastProjection{
        0, static_cast<uint16_t>(secondary.SerializedSize()), nullptr,
        [](void*, uint8_t* out, uint16_t) { secondary.Serialize(out); },
        [](void*, uint16_t offset, const uint8_t* data, uint16_t size) {
            return capicola::host_controls::SetSecondary(secState, difState[1],
                offset, data, size, loop.Phys());
        }
    });
    host.Fast(options, hostlink::FastProjection{
        0, static_cast<uint16_t>(options.SerializedSize()), nullptr,
        [](void*, uint8_t* out, uint16_t) { options.Serialize(out); },
        [](void*, uint16_t offset, const uint8_t* data, uint16_t size) {
            return capicola::host_controls::SetOptions(options.snapToSaved,
                jackOpts, routeCaught, offset, data, size);
        }
    });
    DescribeManual(host);
    loop.Use(host);   /* descriptor + USB come up inside BootLoad() */
#endif

    /* Restore slot 0 over the seeded defaults if a valid save exists (catch
     * re-arms on load). */
    presets.Init();
    presets.BootLoad();

    /* Loaded secondaries only reach the engine if we push them once. */
    for (uint8_t p = 0; p < NUM_POTS; p++)
        SecApply(p, SecValue(p, secState[p].stored));   // (all six: P6 is turned from the feedback and envelope page)

    hw.seed.SetLed(true);
#ifdef CAPICOLA_BENCH_LOG
    SerialLog::StartLog(false);   // (brings the front USB up as a plain serial port)
    {
        daisy::UsbHandle usb;
        usb.SetReceiveCallback(OnUsbRx, daisy::UsbHandle::FS_EXTERNAL);
    }
#endif
    hw.StartAudio(AudioCallback);

    for (;;) loop.Tick();
}
