/**
 * @file capicola_manual.cpp
 * @brief The firmware-authored interactive manual.
 *
 * Prose and panel metadata, carried in the HostLink descriptor and rendered
 * by the web programmer alongside the live module. Ranges, units, defaults,
 * colors and labels are NOT restated here — the host reads those straight
 * from the control declarations in main.cpp. This file adds only what a
 * reader cannot infer from the panel.
 *
 * Panel jacks and buttons are declared here rather than in main.cpp:
 * alchemy::Jack is pure descriptor metadata (no state, no runtime, no
 * schema-hash impact), and Capicola's three buttons are hand-driven in
 * main.cpp's poll hook, so their VirtualButtons carry no ButtonBank state
 * either. Both exist solely to be documented.
 */

#include "alchemy/host_link/host.h"
#include "alchemy/hw/alchemy_lab_v2_layout.h"   /* kButtonB1..B3 */
#include "alchemy/surface/jack.h"
#include "alchemy/surface/manual.h"
#include "alchemy/surface/page.h"
#include "alchemy/surface/virtual_button.h"
#include "alchemy/surface/virtual_knob.h"

using namespace alchemy;

/* Declared in main.cpp — the perf page, the depth page, and the host. */
extern VirtualKnob pitch, stretch, threshold, grainSize, quality, feedback;
extern VirtualKnob cvDepth1, cvDepth2, cvDepth3, cvDepth4, cvDepth5, cvDepth6;
extern Page        perfPage, depthPage;

/* ── Panel jacks (bottom 2×5 grid, left column in / right column out) ─── */

static Jack in_l  = Jack("in.l", "In L", JackSig::AudioIn).Short("IN L");
static Jack in_r  = Jack("in.r", "In R", JackSig::AudioIn).Short("IN R");

static Jack trig_in = Jack("trig.in", "Trig In", JackSig::Trig).Short("TRIG IN");
static Jack cv_in   = Jack("cv.in",   "CV In",   JackSig::CvBi).Short("CV IN");

static Jack trig_1 = Jack("trig.1", "Trig 1", JackSig::Trig).Short("TRIG 1");
static Jack env_1  = Jack("env.1",  "Env 1",  JackSig::CvUni).Short("ENV 1");

static Jack trig_2 = Jack("trig.2", "Trig 2", JackSig::Trig).Short("TRIG 2");
static Jack env_2  = Jack("env.2",  "Env 2",  JackSig::CvUni).Short("ENV 2");

static Jack out_l = Jack("out.l", "Out L", JackSig::AudioOut).Short("OUT L");
static Jack out_r = Jack("out.r", "Out R", JackSig::AudioOut).Short("OUT R");

/* ── Panel buttons (metadata only — main.cpp owns the behavior) ───────── */

static VirtualButton b1 = VirtualButton(kButtonB1, "Page / Reset")
    .Ident("b1")
    .Role(VirtualButton::Role::Modal)
    .Action("click", "Primary / Depth")
    .Action("hold-b1-b3", "Reset visible page");

static VirtualButton b2 = VirtualButton(kButtonB2, "Slice / Save")
    .Ident("b2")
    .Role(VirtualButton::Role::Modal)
    .Action("press", "Slice")
    .Action("hold", "Save boot state");

static VirtualButton b3 = VirtualButton(kButtonB3, "Page")
    .Ident("b3")
    .Role(VirtualButton::Role::Modal)
    .Action("click", "Routing / Secondary");

/* ── Routing-page fields ──────────────────────────────────────────────────
 * RoutingStore (main.cpp) emits these as enum fields; the words live here
 * with the rest of the manual. Zone order matches kSrcColors / ZoneOf(). */

extern const char* const kRouteDispJson =
    "{\"kind\":\"enum\",\"labels\":[\"In Env\",\"Out Env\",\"CV In\"]}";

extern const char* const kRouteFieldId[6] = {
    "route.p1", "route.p2", "route.p3", "route.p4", "route.p5", "route.p6",
};

extern const char* const kRouteFieldName[6] = {
    "Pitch Source",    "Stretch Source", "Threshold Source",
    "Grain Size Source", "Quality Source", "Feedback Source",
};

extern const VirtualKnob* const kRouteSeeKnob[6] = {
    &pitch, &stretch, &threshold, &grainSize, &quality, &feedback,
};

extern const char* const kRouteFieldHelp[6] = {
    "Source for **Pitch**.",
    "Source for **Stretch**.",
    "Source for **Threshold** — a follower steering the detector's own "
    "threshold is where the module starts breathing.",
    "Source for **Grain Size**.",
    "Source for **Quality**.",
    "Source for **Feedback**. The output follower acts as a governor here: "
    "the louder it gets, the further the loop gain can go before it runs.",
};

/* ── Secondary-page fields ────────────────────────────────────────────────
 * SecondaryStore (main.cpp) emits these as f32 fields. Both cutoffs are
 * Nyquist-normalized internally, so displayed Hz = fc x 24000 at 48 kHz;
 * fade converts from samples the same way. */

extern const char* const kSecFieldId[6] = {
    "sec.smoothing", "sec.fade",  "sec.drive",
    "sec.character", "sec.mix",   "sec.fbtone",
};

extern const char* const kSecFieldName[6] = {
    "Smoothing", "Fade", "Drive", "Character", "Mix", "FB Tone",
};

extern const char* const kSecFieldDisp[6] = {
    "{\"kind\":\"exp\",\"lo\":1.2,\"hi\":3000,\"unit\":\"Hz\"}",
    "{\"kind\":\"exp\",\"lo\":10,\"hi\":250,\"unit\":\"ms\"}",
    "{\"kind\":\"linear\",\"lo\":0.5,\"hi\":4}",
    "{\"kind\":\"linear\",\"lo\":0,\"hi\":1}",
    "{\"kind\":\"norm\"}",
    "{\"kind\":\"exp\",\"lo\":48,\"hi\":21600,\"unit\":\"Hz\"}",
};

extern const char* const kSecFieldHelp[6] = {
    "Cutoff of the follower both envelopes run through. Low pass filters "
    "the signal before detection.",

    "Real time latency, and button triggered slice duration. Longer times "
    "means more latency with longer crossfades on button press.",

    "Gain into the keyframe shaper. Applied to the keyframes themselves to "
    "reduce aliasing.",

    "Tape-like dropout saturation curve to clean to wobbly boost saturator. "
    "The tape saturator reduces the volume of low amplitude signals, the "
    "boost saturator boosts them.",

    "Dry/wet mix.",

    "Bandpass centre inside the feedback loop. Affects the tonality of the "
    "feedback.",
};

/* ── Module-level manual ──────────────────────────────────────────────── */

static Manual manual =
    Manual()
        .Tagline("Transient-locked keyframe time stretcher")
        .Preamble(
            "Incoming stereo is stretched, and in parallel its envelope and "
            "transients are extracted. When a transient passes the "
            "threshold, the lagging read head snaps to the current time "
            "with a short crossfade and stretches on from there — time "
            "stretching that stays true to the rhythm of the input. Pitch "
            "and time are fully decoupled.\n\n"
            "The engine is [Keyframe Time Stretching via Extrema Sampling]"
            "(https://github.com/heavylight-industries/dafx26-paper), "
            "DAFx26.")
        .Section("getting-around", "Getting Around",
                 "Four pages in two groups. **B1** walks Primary → Depth, "
                 "**B3** walks Routing → Secondary. Each group remembers "
                 "where it was left, so the other group's button jumps back "
                 "to where you were.\n\n"
                 "Every page stores its own value per knob. After a page "
                 "switch a knob stays decoupled until the pot sweeps "
                 "through the stored value — the ring shows a pip there — "
                 "then it catches. Nothing jumps. Routing catches per zone.")
        .Section("routing", "Routing",
                 "Each pot picks the modulation source for the "
                 "same-numbered Primary knob, CCW → CW: input follower, "
                 "output follower, CV In. The ring shows all three zones "
                 "dim with the selected one lit, so it doubles as the "
                 "position map.\n\n"
                 "The default is the input follower, so the module "
                 "self-patches out of the box: every performance control is "
                 "normalized to the envelope followers, and the dynamics of "
                 "the signal modulate them in real time. The output "
                 "follower is the wilder option — it hears what the module "
                 "is already doing, so depth alone closes a loop.\n\n"
                 "Both followers are the detectors' own smoothed TKEO "
                 "envelopes — input side is max of L/R, output side runs on "
                 "the post-mix sum. The output follower never triggers a "
                 "slice; it is a signal source only.")
        .Section("panel", "Reading the Panel",
                 "All three buttons wear the visible page's hue and are "
                 "never off, so activity reads as a **dark flash**: B2 "
                 "blinks off on a slice, and all three blink three times to "
                 "confirm a save or a reset.\n\n"
                 "The pip at six o'clock on **P3**–**P6** mirrors the four "
                 "CV outputs — Trig 1, Trig 2, Env 1, Env 2. Env 1 turns "
                 "red when the input clips.")
        .PresetsHelp(
            "Slot 0 is the boot state: **B2** held writes it, and the "
            "module loads it at power-up. The remaining slots are read and "
            "written from the programmer.");

/* ── Entity prose ─────────────────────────────────────────────────────── */

/* Split from DescribeManual() so a host build can render and validate the
 * descriptor — every .SeeAlso() target and gesture-help key is checked at
 * descriptor build — without linking the USB stack. */
void AttachManualHelp()
{
    perfPage.Help(
        "The six knobs you play. Depth for each lives on the Depth page, "
        "source on the Routing page.");
    depthPage.Help(
        "Bipolar depth for the same-numbered Primary knob; noon is off. "
        "Applied to the knob's *position*, so a modulated knob keeps its "
        "own taper and endpoints.");

    pitch.Help("Grain pitch, decoupled from time. A detent at noon snaps to "
               "true unity.")
        .SeeAlso(stretch, cvDepth1);
    stretch.Help("Playback rate of the keyframe grid: realtime at CCW, a "
                 "**true freeze** at the stop — the grid stops rather than "
                 "crawling. Triggers still re-anchor it while frozen.")
        .SeeAlso(pitch, cvDepth2);
    threshold.Help("Self-calibrating: events are kept at `knob × running "
                   "average` of the detector's own envelope. **The top of the knob mutes "
                   "auto-triggering** — only B2 or Trig In slices.")
        .SeeAlso(trig_in, b2);
    grainSize.Help("How much keyframe material a grain covers before the "
                   "head is sliced on.");
    quality.Help("Analyzer keyframe threshold. CW is full fidelity; CCW "
                 "drops keyframes for a sparse, crunchy reconstruction. Lower "
                 "quality = longer buffer.");
    feedback.Help("Loop gain. Wet output → sinc saturator → bandpass → back "
                  "into the recorder, where it is re-sliced and re-pitched. "
                  "**Above unity is deliberately unstable.** The tap sits "
                  "before the mix blend, so Mix down does not starve it.")
        .SeeAlso(threshold);

    cvDepth1.Help("Depth for **Pitch**.").SeeAlso(pitch);
    cvDepth2.Help("Depth for **Stretch**.").SeeAlso(stretch);
    cvDepth3.Help("Depth for **Threshold**.").SeeAlso(threshold);
    cvDepth4.Help("Depth for **Grain Size**.").SeeAlso(grainSize);
    cvDepth5.Help("Depth for **Quality**.").SeeAlso(quality);
    cvDepth6.Help("Depth for **Feedback**.").SeeAlso(feedback);

    in_l.Help("Left input. The two channels are detected and sliced "
              "independently.");
    in_r.Help("Right input.");
    trig_in.Help("External slice trigger. Rising edge, always accepted — "
                 "even at threshold mute.")
        .SeeAlso(threshold, b2);
    cv_in.Help("External modulation source, routable to any Primary knob "
               "from the Routing page. The only bipolar source of the "
               "three.");
    trig_1.Help("Triggers a slice, same as pressing the slice button.")
        .SeeAlso(threshold);
    trig_2.Help("Gate on the output follower's transients.");
    env_1.Help("Input follower envelope, mirrored on the P5 pip.");
    env_2.Help("Output follower envelope, mirrored on the P6 pip.");
    out_l.Help("Left output, post-mix.");
    out_r.Help("Right output, post-mix.");

    b1.Help("Rotates between the two primary pages (warm colors).")
        .GestureHelp("click",
                     "Steps Primary → Depth, or returns to where the group "
                     "was left.")
        .GestureHelp("hold-b1-b3",
                     "With B3, half a second: resets the visible page to "
                     "factory defaults.")
        .SeeAlso(b3);
    b2.Help("Slice, and save.")
        .GestureHelp("press",
                     "Fires a slice, jumping the read head to the current "
                     "time.")
        .GestureHelp("hold",
                     "A second and a half: saves all four pages as the boot "
                     "state. The press still slices on the way down.")
        .SeeAlso(threshold, trig_in);
    b3.Help("Rotates between the two secondary pages (cool colors).")
        .GestureHelp("click",
                     "Steps Routing → Secondary, or returns to where the "
                     "group was left.")
        .SeeAlso(b1);
}

void DescribeManual(hostlink::Host& host)
{
    AttachManualHelp();

    host.Jacks(in_l, in_r, trig_in, cv_in, trig_1, env_1, trig_2, env_2,
               out_l, out_r)
        .Buttons(b1, b2, b3)
        .Attach(manual);
}
