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

static Jack trig_in = Jack("trig.in", "Trig In", JackSig::Trig).Short("TRIG IN");
static Jack cv_in   = Jack("cv.in",   "CV In",   JackSig::CvBi).Short("CV IN");

/* The four follower jacks change role with the jack options, so each gets one
 * declaration per role: Trig 1 / Env 1 as CV 2 / CV 3 inputs or as printed,
 * Trig 2 / Env 2 typed by what they carry. DescribeJacks() publishes the set
 * that matches, and main.cpp refreshes the descriptor whenever the options
 * change. Printed names stay as the silkscreen labels. */
static Jack trig_1_in   = Jack("trig.1", "CV 2 / Trig 1", JackSig::CvBi).Short("TRIG 1");
static Jack trig_1_out  = Jack("trig.1", "Trig 1",        JackSig::Trig).Short("TRIG 1");
static Jack env_1_in    = Jack("env.1",  "CV 3 / Env 1",  JackSig::CvBi).Short("ENV 1");
static Jack env_1_out   = Jack("env.1",  "Env 1",         JackSig::CvUni).Short("ENV 1");

static Jack trig_2_trig = Jack("trig.2", "Trig 2", JackSig::Trig).Short("TRIG 2");
static Jack trig_2_env  = Jack("trig.2", "Trig 2", JackSig::CvUni).Short("TRIG 2");
static Jack env_2_trig  = Jack("env.2",  "Env 2",  JackSig::Trig).Short("ENV 2");
static Jack env_2_env   = Jack("env.2",  "Env 2",  JackSig::CvUni).Short("ENV 2");


static Jack out_l = Jack("out.l", "Out L", JackSig::AudioOut).Short("OUT L");
static Jack out_r = Jack("out.r", "Out R", JackSig::AudioOut).Short("OUT R");

/* The set the descriptor currently carries; DescribeJacks() rewrites it. */
static Jack published[10] = {in_l, in_r, trig_in, cv_in, trig_1_in, env_1_in,
                             trig_2_trig, env_2_env, out_l, out_r};

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
    "{\"kind\":\"enum\",\"labels\":[\"In Env\",\"Out Env\",\"CV In\",\"CV 2\",\"CV 3\"]}";

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

extern const char* const kOptJacksId   = "opt.jacks";
extern const char* const kOptJacksName = "Trig 1 / Env 1 Jacks";
extern const char* const kOptJacksDisp =
    "{\"kind\":\"enum\",\"labels\":[\"Outputs\",\"CV inputs\"]}";
extern const char* const kOptJacksHelp = manual_text::kJackMode;

/* Trig 2 / Env 2 in CV-input mode. Label order = jacks::Signal. */
extern const char* const kOptTrig2Id   = "opt.trig2";
extern const char* const kOptTrig2Name = "Trig 2 Jack";
extern const char* const kOptEnv2Id    = "opt.env2";
extern const char* const kOptEnv2Name  = "Env 2 Jack";
extern const char* const kOptOutDisp =
    "{\"kind\":\"enum\",\"labels\":[\"Input triggers\",\"Output triggers\","
    "\"Input envelope\",\"Output envelope\"]}";
extern const char* const kOptOutsHelp = manual_text::kJackOuts;

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
    threshold.Help(kPrimaryKnob[2]).SeeAlso(trig_in, b2);
    grainSize.Help(kPrimaryKnob[3]).SeeAlso(quality);
    quality  .Help(kPrimaryKnob[4]).SeeAlso(grainSize, stretch);
    feedback .Help(kPrimaryKnob[5]).SeeAlso(threshold);

    VirtualKnob* const depth[6] = {&cvDepth1, &cvDepth2, &cvDepth3,
                                   &cvDepth4, &cvDepth5, &cvDepth6};
    VirtualKnob* const target[6] = {&pitch, &stretch, &threshold,
                                    &grainSize, &quality, &feedback};
    for (int p = 0; p < 6; p++)
        depth[p]->Help(kDepthKnob[p]).SeeAlso(*target[p]);

    /* kJack is in panel grid order: top row L→R, then the bottom row. */
    in_l   .Help(kJack[0]);
    trig_in.Help(kJack[1]).SeeAlso(threshold, b2);
    for (Jack* j : {&trig_1_in, &trig_1_out})
        j->Help(kJack[2]).SeeAlso(threshold, b2, "opt.jacks");
    for (Jack* j : {&trig_2_trig, &trig_2_env})
        j->Help(kJack[3]).SeeAlso("opt.trig2");
    out_l  .Help(kJack[4]);
    in_r   .Help(kJack[5]);
    cv_in  .Help(kJack[6]);
    for (Jack* j : {&env_1_in, &env_1_out})
        j->Help(kJack[7]).SeeAlso("opt.jacks");
    for (Jack* j : {&env_2_trig, &env_2_env})
        j->Help(kJack[8]).SeeAlso("opt.env2");
    out_r  .Help(kJack[9]);

    b1.Help(kButton[0])
        .GestureHelp("click", kButtonTap[0])
        .GestureHelp("hold-b1-b2", kGestureSave)
        .GestureHelp("hold-b1-b3", kGestureReset)
        .SeeAlso(b3);
    b2.Help(kButton[1])
        .GestureHelp("press", kButtonTap[1])
        .GestureHelp("hold", kGestureFlip)
        .GestureHelp("hold-b2-b3", kGestureLoad)
        .SeeAlso(threshold, trig_in);
    b3.Help(kButton[2])
        .GestureHelp("click", kButtonTap[2])
        .GestureHelp("hold", kGestureSnap)
        .SeeAlso(b1, "opt.snap");
}

/* Publish the jack set for these options. Out of CV-input mode the four
 * follower jacks are exactly as printed, whatever the Trig 2 / Env 2 choices. */
void DescribeJacks(hostlink::Host& host, const jacks::Options& o)
{
    auto env = [](uint8_t s) { return s == jacks::kInEnv || s == jacks::kOutEnv; };
    const bool t2env = o.cvIn && env(o.trig2);
    const bool e2env = !o.cvIn || env(o.env2);
    published[0] = in_l;
    published[1] = in_r;
    published[2] = trig_in;
    published[3] = cv_in;
    published[4] = o.cvIn ? trig_1_in : trig_1_out;
    published[5] = o.cvIn ? env_1_in  : env_1_out;
    published[6] = t2env  ? trig_2_env : trig_2_trig;
    published[7] = e2env  ? env_2_env  : env_2_trig;
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
