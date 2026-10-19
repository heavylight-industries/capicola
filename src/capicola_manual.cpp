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

#include "capicola_manual_text.h"   /* every word below — generated from docs/manual.html */
#include "jack_routing.h"           /* jacks::Options — which role each jack is in */

using namespace alchemy;

/* Declared in main.cpp — the perf page, the depth page, and the host. */
extern VirtualKnob pitch, stretch, threshold, grainSize, quality, feedback;
extern VirtualKnob cvDepth1, cvDepth2, cvDepth3, cvDepth4, cvDepth5, cvDepth6;
extern Page        perfPage, depthPage;

/* ── Panel jacks (bottom 2×5 grid, left column in / right column out) ─── */

static Jack in_l  = Jack("in.l", "In L", JackSig::AudioIn).Short("IN L");
static Jack in_r  = Jack("in.r", "In R", JackSig::AudioIn).Short("IN R");

static Jack trig_in = Jack("trig.in", "J3", JackSig::Trig).Short("J3");
static Jack cv_in = Jack("cv.in", "J4", JackSig::CvBi).Short("J4");
static Jack trig_1 = Jack("trig.1", "J5", JackSig::CvBi).Short("J5");
static Jack env_1 = Jack("env.1", "J6", JackSig::CvBi).Short("J6");
static Jack trig_2 = Jack("trig.2", "J7", JackSig::Trig).Short("J7");
static Jack env_2 = Jack("env.2", "J8", JackSig::CvUni).Short("J8");
static Jack out_l = Jack("out.l", "Out L", JackSig::AudioOut).Short("OUT L");
static Jack out_r = Jack("out.r", "Out R", JackSig::AudioOut).Short("OUT R");

static Jack published[10] = {in_l, in_r, trig_in, cv_in, trig_1, env_1,
                             trig_2, env_2, out_l, out_r};

/* ── Panel buttons (metadata only — main.cpp owns the behavior) ───────── */

/* Names from the manual: what each button does, printed role first. */
static VirtualButton b1 = VirtualButton(kButtonB1, manual_text::kButtonName[0])
    .Ident("b1")
    .Role(VirtualButton::Role::Modal)
    .Action("click", "Primary / Depth")
    .Action("hold-b1-b2", "Save boot state")
    .Action("hold-b1-b3", "Factory reset, all pages");

static VirtualButton b2 = VirtualButton(kButtonB2, manual_text::kButtonName[1])
    .Ident("b2")
    .Role(VirtualButton::Role::Modal)
    .Action("press", "Slice")
    .Action("hold", "Polarity flip: P1/P2 direction")
    .Action("hold-b2-b3", "Load boot state");

static VirtualButton b3 = VirtualButton(kButtonB3, manual_text::kButtonName[2])
    .Ident("b3")
    .Role(VirtualButton::Role::Modal)
    .Action("click", "Routing / Secondary")
    .Action("hold", "Shift: snap knobs to their target");

/* ── Routing-page fields ──────────────────────────────────────────────────
 * RoutingStore (main.cpp) emits these as enum fields; the words live here
 * with the rest of the manual. Zone order matches kSrcColors / ZoneOf(). */

extern const char* const kRouteDispJson =
    "{\"kind\":\"enum\",\"labels\":[\"In Env\",\"Out Env\"]}";

extern const char* const kRouteFieldId[6] = {
    "route.p1", "route.p2", "route.p3", "route.p4", "route.p5", "route.p6",
};

extern const char* const kRouteFieldName[6] = {
    manual_text::kRouteName[0], manual_text::kRouteName[1], manual_text::kRouteName[2],
    manual_text::kRouteName[3], manual_text::kRouteName[4], manual_text::kRouteName[5],
};

extern const VirtualKnob* const kRouteSeeKnob[6] = {
    &pitch, &stretch, &threshold, &grainSize, &quality, &feedback,
};

extern const char* const kRouteFieldHelp[6] = {
    manual_text::kRouteHelp[0], manual_text::kRouteHelp[1], manual_text::kRouteHelp[2],
    manual_text::kRouteHelp[3], manual_text::kRouteHelp[4], manual_text::kRouteHelp[5],
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
    manual_text::kSecName[0], manual_text::kSecName[1], manual_text::kSecName[2],
    manual_text::kSecName[3], manual_text::kSecName[4], manual_text::kSecName[5],
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
    manual_text::kSecHelp[0], manual_text::kSecHelp[1], manual_text::kSecHelp[2],
    manual_text::kSecHelp[3], manual_text::kSecHelp[4], manual_text::kSecHelp[5],
};

/* ── Options (no knob — programmer only) ───────────────────────────────── */

extern const char* const kOptSnapId   = "opt.snap";
extern const char* const kOptSnapName = "Shift Snap Target";
extern const char* const kOptSnapDisp =
    "{\"kind\":\"enum\",\"labels\":[\"Factory\",\"Save point\"]}";
extern const char* const kOptSnapHelp = manual_text::kSnapTarget;

/* ── Module-level manual ──────────────────────────────────────────────── */

/* The overviews with no entity to hang on, and the two gestures that belong
 * to no button, ride as sections. */
static Manual manual =
    Manual()
        .Tagline(manual_text::kTagline)
        .Preamble(manual_text::kPreamble)
        .Section("routing",   "Routing",   manual_text::kRoutingOverview)
        .Section("secondary", "Secondary", manual_text::kSecondaryOverview)
        .Section("gestures",  "Gestures",  manual_text::kGesturesOverview)
        .Section("catch",     "Pot Catch", manual_text::kPotCatch)
        .Section("slicing",   "Slicing",   manual_text::kSlicing)
        .Section("panel",     "Reading the Panel", manual_text::kReadingPanel)
        .PresetsHelp(manual_text::kHistory);

/* ── Entity prose ─────────────────────────────────────────────────────── */

/* Split from DescribeManual() so a host build can render and validate the
 * descriptor — every .SeeAlso() target and gesture-help key is checked at
 * descriptor build — without linking the USB stack. */
void AttachManualHelp()
{
    using namespace manual_text;

    perfPage.Help(kPrimaryPage);
    depthPage.Help(kDepthPage);

    pitch    .Help(kPrimaryKnob[0]).SeeAlso(stretch, cvDepth1);
    stretch  .Help(kPrimaryKnob[1]).SeeAlso(pitch, cvDepth2);
    threshold.Help(kPrimaryKnob[2]).SeeAlso(b2);
    grainSize.Help(kPrimaryKnob[3]).SeeAlso(quality);
    quality  .Help(kPrimaryKnob[4]).SeeAlso(grainSize, stretch);
    feedback .Help(kPrimaryKnob[5]).SeeAlso(threshold);

    VirtualKnob* const depth[6] = {&cvDepth1, &cvDepth2, &cvDepth3,
                                   &cvDepth4, &cvDepth5, &cvDepth6};
    VirtualKnob* const target[6] = {&pitch, &stretch, &threshold,
                                    &grainSize, &quality, &feedback};
    for (int p = 0; p < 6; p++)
        depth[p]->Help(kDepthKnob[p]).SeeAlso(*target[p]);

    /* Audio help is fixed; assignable jack help follows the selected function. */
    in_l.Help(kJack[0]);
    out_l.Help(kJack[4]);
    in_r.Help(kJack[5]);
    out_r.Help(kJack[9]);

    b1.Help(kButton[0])
        .GestureHelp("click", kButtonTap[0])
        .GestureHelp("hold-b1-b2", kGestureSave)
        .GestureHelp("hold-b1-b3", kGestureReset)
        .SeeAlso(b3);
    b2.Help(kButton[1])
        .GestureHelp("press", kButtonTap[1])
        .GestureHelp("hold", kGestureFlip)
        .GestureHelp("hold-b2-b3", kGestureLoad)
        .SeeAlso(threshold);
    b3.Help(kButton[2])
        .GestureHelp("click", kButtonTap[2])
        .GestureHelp("hold", kGestureSnap)
        .SeeAlso(b1, "opt.snap");
}

/* Stable connector identities, with live signal classes and approved help. */
void DescribeJacks(hostlink::Host& host, const jacks::Options& o)
{
    published[0] = in_l;
    published[1] = in_r;
    for (uint8_t j = 0; j < jacks::kNumJacks; ++j)
    {
        const uint8_t f = jacks::FunctionAt(o, j);
        const JackSig sig = jacks::IsCvInput(f) ? JackSig::CvBi
            : f == jacks::kInputEnvelope || f == jacks::kOutputEnvelope
                ? JackSig::CvUni : JackSig::Trig;
        published[j + 2] = Jack(manual_text::kJackId[j], manual_text::kJackPanel[j], sig)
            .Short(manual_text::kJackPanel[j])
            .Help(manual_text::kJackFunctionHelp[f])
            .SeeAlso(manual_text::kJackFieldId[j]);
    }
    published[8] = out_l;
    published[9] = out_r;
    host.Jacks(published);
}

void DescribeManual(hostlink::Host& host)
{
    AttachManualHelp();
    DescribeJacks(host, jacks::Options{});   // factory roles; main.cpp follows changes
    host.Buttons(b1, b2, b3).Attach(manual);
}
