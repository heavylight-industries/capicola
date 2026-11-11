/**
 * @file tests/host/capicola_tests.cpp
 * @brief Host-side unit + invariant tests for the DSP core in lib/.
 *
 * Layers under test, bottom-up: DeluxeLine (interpolated ring reads),
 * TabulatedFunction/Shapers (LUT policies), filters, SparseLine (keyframe
 * ring + window search), Analyzer (extrema sparsifier), Detector (TKEO
 * transient finder), Granule (fences + splices), KeyframeRecorder
 * (state machine + grain trading).  Everything is deterministic: fixed
 * seeds, synthetic signals with known ground truth.
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "DeluxeLine.h"
#include "Contour.h"
#include "Pulse.h"
#include "PcmRing.h"
#include "Diffuser.h"
#include "AdaptiveComb.h"
#include "SparseLine.h"
#include "Analyzer.h"
#include "Detector.h"
#include "Granule.h"
#include "KeyframeRecorder.h"
#include "StereoAlign.h"
#include "Shapers.h"
#include "filter.h"
#include "jack_routing.h"   /* src/ — the firmware's CV jack routing */
#include "mod_swap.h"       /* src/ — the crossfade between a knob's mod sources */

using namespace capicola;

/* ── Tiny test harness (same shape as the SDK's tests/host) ─────────── */

static int g_checks   = 0;
static int g_failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        g_checks++;                                                        \
        if (!(cond)) {                                                     \
            g_failures++;                                                  \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                              \
    do {                                                                   \
        g_checks++;                                                        \
        const double va = (a), vb = (b), vt = (tol);                       \
        if (!(std::fabs(va - vb) <= vt)) {                                 \
            g_failures++;                                                  \
            std::printf("FAIL %s:%d  |%s - %s| <= %s  (%g vs %g)\n",       \
                        __FILE__, __LINE__, #a, #b, #tol, va, vb);         \
        }                                                                  \
    } while (0)

static void section(const char* name) { std::printf("── %s\n", name); }

/* Deterministic RNG (xorshift32) — reproducible fuzz. */
static uint32_t g_rng = 0xC0FFEEu;
static uint32_t rng_reset(uint32_t seed) { return g_rng = seed; }
static float frand()   // [0, 1)
{
    g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5;
    return (float)(g_rng >> 8) / 16777216.0f;
}
static float frand2() { return 2.0f * frand() - 1.0f; }   // [-1, 1)

static constexpr float kFs = 48000.0f;

/* ── The keyframe shaper's level, the sine folder, bipolar strengths ──── */

static void test_shape_level()
{
    section("shaper level, folder, bipolar strengths");
    static Shapers sh;
    sh.Init();
    // The sine table is the folder: sin over any argument, wrapping.
    float worst = 0.0f;
    for (float x = -30.0f; x < 30.0f; x += 0.013f) worst = std::fmax(worst, std::fabs(sh.ReadSin(x) - std::sin(x)));
    CHECK(worst < 2e-3f);
    // Drive and Character change the colour, not the level: a sine at the reference level comes out at its own RMS
    // (within 0.5 dB) through every setting, and bypass is the input exactly.
    static KeyframeRecorder<1024> kr;
    static SparseLine<1024> ring;
    kr.Init(ring, &sh);
    bool level = true;
    for (float ch : {0.0f, 0.2f, 0.5f, 0.8f, 1.0f})
    for (float dr : {0.5f, 1.0f, 2.0f, 4.0f}) {
        kr.SetDistortCharacter(ch);
        kr.SetDistortDrive(dr);
        double in = 0.0, out = 0.0;
        for (int i = 0; i < 1000; i++) {
            const float x = 0.3f * std::sin(0.0371f * (float)i);
            const float y = kr.ShapedForTest(x);
            in += (double)x * x; out += (double)y * y;
        }
        const double db = 10.0 * std::log10(out / in);
        // Never louder; level everywhere but full tape dropout at low drive, which nearly silences the reference
        // sine and would need more than kShapeNormMax: quieter there, as the dropout means.
        const bool dropoutCorner = ch < 0.25f && dr < 1.0f;
        if (db > 0.5 || (!dropoutCorner && db < -0.5)) { level = false; std::printf("   character %.1f drive %.1f: %+.2f dB\n", ch, dr, db); }
    }
    CHECK(level);
    kr.SetDistortCharacter(0.5f);
    CHECK(kr.ShapedForTest(0.123f) == 0.123f);
    // A folded strength may be negative: the contour goes there, and its pulse dips.
    Contour c;
    c.Init();
    c.Hit(-0.6f, 1.0);
    CHECK_NEAR(c.At(1.0 + Contour::kFirstGap), -0.6, 1e-6);
    Pulse p;
    p.Init();
    p.SetWidth(0.05f);
    for (int i = 0; i < 200; i++) p.Tick(i / 750.0, 1.0f / 750.0f);
    p.Hit(-0.8f, 1.0, 0.0f);
    CHECK_NEAR(p.Tick(1.0, 1.0f / 750.0f), -0.8, 1e-6);
}

/* ── Pulse (PULS) ───────────────────────────────────────────────────── */

static void test_pulse()
{
    section("Pulse (PULS)");
    const double dt = 1.0 / 750.0;   // (the engine's block rate)
    // Input side, lead 20 ms: each peak on the heard hit at its strength, starting from 0, never a jump, at any width;
    // the width swept live mid-pulse stays continuous.
    for (float w : {0.004f, 0.02f, 0.2f}) {
        Pulse p;
        p.Init();
        p.SetWidth(w);
        for (int i = 0; i < 200; i++) p.Tick(i * dt, (float)dt);   // (the width glides to w)
        const double hits[] = {1.0, 1.05, 1.4};
        const float  ss[]   = {0.9f, 0.4f, 1.0f};
        float prev = 0.0f, maxStep = 0.0f, atPeak[3] = {0, 0, 0};
        double det[3] = {-1, -1, -1};   // (when each was detected: the block it landed in)
        int k = 0;
        for (double t = 200 * dt; t < 3.0; t += dt) {
            if (k < 3 && t >= hits[k]) { p.Hit(ss[k], t, 0.02f); det[k] = t; k++; }
            const float v = p.Tick(t, (float)dt);
            for (int j = 0; j < 3; j++) if (det[j] >= 0.0 && std::fabs(t - (det[j] + 0.02)) < dt * 0.5) atPeak[j] = v;
            maxStep = std::fmax(maxStep, std::fabs(v - prev));
            prev = v;
        }
        CHECK(atPeak[0] >= 0.9f - 0.02f);       // (on the heard hit: at least its strength, within a block's step)
        CHECK(atPeak[2] >= 1.0f - 0.02f);
        CHECK(maxStep < (w < 0.01f ? 0.6f : 0.25f));   // (no cut: the narrowest pulse is a few blocks wide)
        CHECK(p.Tick(10.0, (float)dt) == 0.0f);     // (retired)
    }
    // Live width: widened while a pulse falls, the pulse widens smoothly (no step past a 20 ms pulse's own fall);
    // widened after a pulse retired, it stays retired (it would come back out of nothing).
    {
        Pulse p;
        p.Init();
        p.SetWidth(0.02f);
        for (int i = 0; i < 200; i++) p.Tick(i * dt, (float)dt);
        p.Hit(1.0f, 1.0, 0.02f);
        float prev = 0.0f, maxStep = 0.0f;
        for (double t = 1.0; t < 1.6; t += dt) {
            if (t > 1.03) p.SetWidth(0.5f);
            const float v = p.Tick(t, (float)dt);
            if (t > 1.022) maxStep = std::fmax(maxStep, std::fabs(v - prev));   // (the fall: after the peak at 1.02)
            prev = v;
        }
        CHECK(maxStep < 0.06f);
        CHECK(prev > 0.1f);    // (still there, wide)
        Pulse q;
        q.Init();
        q.SetWidth(0.01f);
        for (int i = 0; i < 200; i++) q.Tick(i * dt, (float)dt);
        q.Hit(1.0f, 1.0, 0.02f);
        for (double t = 1.0; t < 1.2; t += dt) q.Tick(t, (float)dt);   // (retired by 1.06)
        q.SetWidth(0.5f);
        float most = 0.0f;
        for (double t = 1.2; t < 1.6; t += dt) most = std::fmax(most, q.Tick(t, (float)dt));
        CHECK(most == 0.0f);
    }
    // Output side, lead 0: at its peak from its hit (fast attack), then the fall.
    {
        Pulse p;
        p.Init();
        p.SetWidth(0.05f);
        for (int i = 0; i < 200; i++) p.Tick(i * dt, (float)dt);
        p.Hit(0.7f, 1.0, 0.0f);
        CHECK_NEAR(p.Tick(1.0, (float)dt), 0.7, 1e-6);
        CHECK_NEAR(p.Tick(1.05, (float)dt), 0.7 * std::exp(-0.5), 2e-3);
    }
}

/* ── Contour (CONT) ─────────────────────────────────────────────────── */

static void test_contour()
{
    section("Contour (CONT)");
    Contour c;
    c.Init();
    CHECK(c.Tick(0.0) == 0.0f);
    // Hits at 1.0 s (0.8), 1.5 s (0.3), 1.6 s (0.9): each glides from where the contour is to its strength, over the
    // gap since the hit before (the first over kFirstGap), and holds; never past either end between hits.
    const double ts[] = {1.0, 1.5, 1.6};
    const float  ss[] = {0.8f, 0.3f, 0.9f};
    bool between = true, smooth = true;
    float prev = 0.0f;
    int k = 0;
    for (double t = 0.0; t < 4.0; t += 0.001) {
        if (k < 3 && t >= ts[k]) { c.Hit(ss[k], t); k++; }
        const float v = c.Tick(t);
        if (std::fabs(v - prev) > 0.02f) smooth = false;   // (a smoothstep, never a jump)
        if (k >= 1) {
            const float a = (k >= 2) ? std::fmin(ss[k - 2], ss[k - 1]) : 0.0f, b = std::fmax(k >= 2 ? ss[k - 2] : 0.0f, ss[k - 1]);
            if (v < std::fmin(a, prev) - 1e-6f || v > std::fmax(b, prev) + 1e-6f) between = false;
        }
        prev = v;
    }
    CHECK(between);
    CHECK(smooth);
    CHECK_NEAR(c.Value(), 0.9, 1e-6);                  // (held after the last hit's glide)
    Contour d;
    d.Init();
    d.Hit(1.0f, 10.0);
    CHECK_NEAR(d.At(10.0 + Contour::kFirstGap * 0.5), 0.5, 1e-4);   // (half way: a smoothstep's middle)
    CHECK_NEAR(d.At(10.0 + Contour::kFirstGap), 1.0, 1e-6);
    d.Hit(0.0f, 10.001);                               // (closer than the hold-off: kMinGap)
    CHECK_NEAR(d.At(10.001 + Contour::kMinGap), 0.0, 1e-6);
    d.Hit(0.5f, 30.0);                                 // (after a long gap: kMaxGap)
    CHECK_NEAR(d.At(30.0 + Contour::kMaxGap), 0.5, 1e-6);
}

/* ── PCM at the top of Quality ──────────────────────────────────────── */

static void test_pcm()
{
    section("PCM (the top of Quality)");
    {   // The ring: lockstep indices, Hermite through the samples, what it Holds.
        static int16_t buf[1000];
        PcmRing r;
        r.Init(buf, 1000);
        for (int i = 0; i < 2500; i++) r.Write(0.5f * std::sin(0.01f * (float)i));
        CHECK(!r.Holds(1499.0) && r.Holds(1502.0) && r.Holds(2497.0) && !r.Holds(2498.0));
        float worst = 0.0f;
        for (double t = 1502.0; t < 2497.0; t += 0.37)
            worst = std::fmax(worst, std::fabs(r.Read(t) - 0.5f * std::sin(0.01f * (float)t)));
        CHECK(worst < 1e-4f);
        r.Reset();
        CHECK(!r.Holds(10.0));
    }
    // A live recorder at pitch 1, stretch 1, the shaper at bypass: from the keyframes the output is the input
    // reconstructed; from the PCM it is the input itself, one fade late, to int16's step.
    static KeyframeRecorder<65536> kr;
    static SparseLine<65536> ring;
    static int16_t pcmBuf[1 << 18];
    static PcmRing pcm;
    pcm.Init(pcmBuf, 1 << 18);
    static Shapers sh;
    sh.Init();
    std::vector<float> in(48000 * 3), out(in.size());
    rng_reset(9);
    for (size_t i = 0; i < in.size(); i++)
        in[i] = 0.3f * std::sin(0.031f * (float)i) + 0.2f * std::sin(0.0071f * (float)i) + 0.05f * frand2();
    double errKf = 0.0, errPcm = 0.0;
    for (int pass = 0; pass < 2; pass++) {
        kr.Init(ring, &sh, &pcm);
        kr.SetDistortCharacter(0.5f);   // (bypass)
        kr.SetThreshold(0.001f);
        kr.SetGrainFade(960.0f);
        kr.SetTransientThreshold(1.0e9f);   // (no hits: the heads just ride behind now)
        kr.SubmitRequest(Request::LIVE_EFFECT);
        kr.SetPcm(pass == 1);
        for (size_t b = 0; b + 64 <= in.size(); b += 64) kr.ProcessBlock(&in[b], &out[b], 64);
        // The best lag in the last half second, to 0.01 sample (the head rides about a fade behind the live edge, at
        // a fraction: grains seat on keyframe times), against the input read through the same Catmull-Rom.
        auto at = [&](double t) {
            const long n = (long)std::floor(t);
            const float f = (float)(t - (double)n);
            const float xm = in[n - 1], x0 = in[n], x1 = in[n + 1], x2 = in[n + 2];
            const float c1 = 0.5f * (x1 - xm), c2 = xm - 2.5f * x0 + 2.0f * x1 - 0.5f * x2;
            const float c3 = 0.5f * (x2 - xm) + 1.5f * (x0 - x1);
            return ((c3 * f + c2) * f + c1) * f + x0;
        };
        double best = 1e30;
        for (double lag = 890.0; lag <= 1010.0; lag += 0.01) {
            double e = 0.0;
            for (size_t i = in.size() - 24000; i < in.size(); i += 4) { const double d = out[i] - at((double)i - lag); e += d * d; }
            if (e < best) best = e;
        }
        (pass ? errPcm : errKf) = std::sqrt(best / 6000.0);
    }
    std::printf("   rms error vs the input: keyframes %.2e, PCM %.2e\n", errKf, errPcm);
    CHECK(errPcm < 5e-4);   // (int16, and the lag found to 0.01 sample)
    CHECK(errPcm < 0.05 * errKf);
}

/* ── AdaptiveComb ───────────────────────────────────────────────────── */

static void test_adaptive_comb()
{
    section("AdaptiveComb");
    static AdaptiveComb c;

    // Size 0: the feedback passes bit for bit.
    c.Init();
    rng_reset(5);
    bool exact = true;
    for (int i = 0; i < 48000; i++) {
        if ((i & 63) == 0) { c.SetKeyframes(5.0f + 40.0f * frand(), 3000.0f + 20000.0f * frand()); c.Block(64); }
        const float x = frand2();
        if (c.Process(x) != x) exact = false;
    }
    CHECK(exact);

    // On, across Size and Spread with the keyframes moving: finite, never past the taps' sum at their scale
    // (1/8 .. 1/sqrt(8) of eight taps: the input's peak .. sqrt(8) of it), no louder than the input on noise, and
    // the taps inside the line.
    for (float size : {0.01f, 0.03f, 0.3f, 0.6f, 1.0f}) {
        c.Init();
        c.SetSize(size);
        bool ok = true;
        double pin = 0.0, pout = 0.0;
        for (int i = 0; i < 48000 * 3; i++) {
            if ((i & 63) == 0) { c.SetKeyframes(3.0f + 60.0f * frand(), 500.0f + 60000.0f * frand()); c.Block(64); }
            const float x = 0.5f * frand2();
            const float y = c.Process(x);
            if (!std::isfinite(y) || std::fabs(y) > 0.5f * std::sqrt(8.0f) + 1e-4f) ok = false;
            if (i >= 48000) { pin += (double)x * x; pout += (double)y * y; }
        }
        CHECK(ok);
        CHECK(10.0 * std::log10(pout / pin) < 1.0);
        CHECK(c.TapBase() >= 8.0f && c.TapBase() <= 8000.0f);
    }

    // Off again: it glides out, then passes bit for bit.
    c.SetSize(0.0f);
    for (int i = 0; i < 48000; i++) { if ((i & 63) == 0) c.Block(64); c.Process(0.1f); }
    CHECK(c.Process(0.25f) == 0.25f);
}

/* ── Diffuser ───────────────────────────────────────────────────────── */

static void test_diffuser()
{
    section("Diffuser");
    static Diffuser d;

    // Diffusion 0: the input passes bit for bit, at any spread.
    d.Init();
    d.SetSpread(1.0f);
    rng_reset(7);
    bool exact = true;
    for (int i = 0; i < 48000; i++) { const float x = frand2(); if (d.Process(x) != x) exact = false; }
    CHECK(exact);
    CHECK(d.Idle());

    // Full diffusion on noise: finite, and never louder than the input (the wet side is power-normalised for taps
    // that line up, so loose ones lose a few dB; in the feedback loop that is the safe side).
    for (float spread : {0.0f, 1.0f}) {
        d.Init();
        d.SetAmount(1.0f);
        d.SetSpread(spread);
        rng_reset(11);
        double pin = 0.0, pout = 0.0;
        bool finite = true;
        for (int i = 0; i < 48000 * 4; i++) {
            if ((i & 63) == 0) { d.SetGrain(500.0f + 9000.0f * frand()); d.Block(64); }   // (taps on the move)
            const float x = 0.5f * frand2();
            const float y = d.Process(x);
            if (!std::isfinite(y)) finite = false;
            if (i >= 48000 * 2) { pin += (double)x * x; pout += (double)y * y; }
        }
        CHECK(finite);
        const double db = 10.0 * std::log10(pout / pin);
        CHECK(db > -6.0 && db < 1.0);
        std::printf("   spread %.0f: power %+.2f dB\n", spread, db);

        // Then silence: the network rings down to nothing.
        float last = 0.0f;
        for (int i = 0; i < 48000 * 20; i++) { if ((i & 63) == 0) d.Block(64); const float y = d.Process(0.0f); if (i >= 48000 * 19) last = std::fmax(last, std::fabs(y)); }
        CHECK(last < 1e-6f);
    }

    // Turned off again: it glides out and goes idle, then passes bit for bit.
    d.SetAmount(0.0f);
    for (int i = 0; i < 48000; i++) d.Process(0.1f);
    CHECK(d.Idle());
    CHECK(d.Process(0.25f) == 0.25f);
}

/* ── DeluxeLine ─────────────────────────────────────────────────────── */

static void test_deluxe_line()
{
    {   // The integer-delay B-spline equals the full read at that delay (the detector's per-sample path).
        DeluxeLine<float, 8> d;
        d.Init();
        rng_reset(3);
        float worst = 0.0f;
        for (int i = 0; i < 20000; i++) {
            d.Write(frand2());
            const CubicResult a = d.ReadBSplineFull(2.0f, 1), b = d.ReadBSplineFullInteger(2);
            worst = std::fmax(worst, std::fabs(a.value - b.value) + std::fabs(a.d1 - b.d1)
                                    + std::fabs(a.d2 - b.d2) + std::fabs(a.tkeo - b.tkeo));
        }
        CHECK(worst < 1e-5f);
    }
    section("DeluxeLine");

    DeluxeLine<float, 8> dl;
    dl.Init();
    for (int i = 0; i < 10; i++) dl.Write((float)i);

    /* Read(k) is a writePos-relative tap: 1 = newest.  Regression for the
     * stale absolute-slot indexing. */
    CHECK_NEAR(dl.Read(1.0f), 9.0f, 0.0);
    CHECK_NEAR(dl.Read(3.0f), 7.0f, 0.0);

    /* Linear read: integer delays hit samples, fractional interpolates. */
    CHECK_NEAR(dl.ReadLinear(2.0f), 8.0f, 1e-6);
    CHECK_NEAR(dl.ReadLinear(2.5f), 7.5f, 1e-6);

    /* B-splines reproduce affine signals exactly: the ramp comes back as
     * itself.  Sample values are 2..9 in the ring; the lag-2 tap is 8. */
    CHECK_NEAR(dl.ReadBSpline(2.0f), 8.0f, 1e-5);
    CHECK_NEAR(dl.ReadBSpline(2.5f), 7.5f, 1e-5);

    /* d1 is d/d(delay) — the delay axis points backward in time, so an
     * increasing ramp has NEGATIVE d1, magnitude = the per-sample step. */
    CubicResult ri = dl.ReadBSplineD1Integer(2);
    CubicResult rf = dl.ReadBSplineFull(2.0f, 1);
    CHECK_NEAR(ri.value, rf.value, 1e-6);
    CHECK_NEAR(ri.d1,    rf.d1,    1e-6);
    CHECK(ri.d1 < 0.0f);
    CHECK_NEAR(ri.d1, -1.0f, 1e-5);

    /* Hermite (Catmull-Rom) interpolates: integer delays return samples
     * exactly, and it is exact on affine signals too. */
    CHECK_NEAR(dl.ReadHermite(2.0f), 8.0f, 1e-5);
    CHECK_NEAR(dl.ReadHermite(2.25f), 7.75f, 1e-5);
}

/* ── TabulatedFunction / Shapers ────────────────────────────────────── */

static void test_shapers()
{
    section("TabulatedFunction / Shapers");

    static Shapers sh;   // static: 3×4096 float tables
    sh.Init();

    /* Odd mirror + wrap covers all of sin from a [0, π] table. */
    for (float x = -9.5f; x < 9.5f; x += 0.37f)
        CHECK_NEAR(sh.ReadSin(x), std::sin(x), 2e-3);

    /* Odd-mirrored saturators: odd symmetry and clamp past the table edge. */
    CHECK_NEAR(sh.ReadQuake(0.5f), -sh.ReadQuake(-0.5f), 1e-6);
    CHECK_NEAR(sh.ReadSinc(1.3f),  -sh.ReadSinc(-1.3f),  1e-6);
    CHECK_NEAR(sh.ReadQuake(100.0f), sh.ReadQuake(4.0f), 1e-6);
    CHECK_NEAR(sh.ReadQuake(0.0f), 0.0f, 1e-3);
}

/* ── filters ────────────────────────────────────────────────────────── */

static void test_filters()
{
    section("filters");

    OnePole lp;
    lp.Init();
    lp.SetCutoff(0.01f);
    for (int i = 0; i < 20000; i++) lp.Tick(1.0f);
    CHECK_NEAR(lp.GetLowpass(), 1.0f, 1e-3);   // DC gain 1

    StateVariable svf;
    svf.Init();
    svf.SetControls(0.01f, 0.5f);
    float peak = 0.0f;
    svf.Tick(1.0f);
    for (int i = 0; i < 20000; i++) {
        svf.Tick(0.0f);
        const float v = std::fabs(svf.GetLowpass());
        if (v > peak) peak = v;
        CHECK(std::isfinite(svf.GetLowpass()));
        if (g_failures) return;   // don't spam 20k failures
    }
    CHECK(peak < 2.0f);   // impulse response bounded (stable)
}

/* ── SparseLine ─────────────────────────────────────────────────────── */

static void test_sparse_line()
{
    section("SparseLine");

    SparseLine<8> sl;
    sl.Init();

    /* 20 frames at t = 100·i into an 8-deep ring: indices stay monotonic,
     * the live window is [12, 19], and GetSample clamps into it. */
    for (int i = 0; i < 20; i++) sl.Write({(float)i, 100.0 * i});
    CHECK(sl.GetLatestIndex() == 19);
    CHECK(sl.GetOldestIndex() == 12);
    CHECK_NEAR(sl.GetSample(15)->value, 15.0f, 0.0);
    CHECK_NEAR(sl.GetSample(3)->value,  12.0f, 0.0);    // clamped to oldest
    CHECK_NEAR(sl.GetSample(99)->value, 19.0f, 0.0);    // clamped to newest

    /* WindowAt (binary search) brackets a time correctly across the wrap,
     * and StepWindow walked from the latest window agrees with it. */
    for (double t = 1250.0; t < 1900.0; t += 87.0) {
        Window w = sl.WindowAt(t);
        CHECK(w.p2->time <= t && t <= w.p1->time);

        Window s = sl.WindowAtLatest();
        sl.StepWindow(s, t);
        CHECK(s.index == w.index);
    }

    /* Interp endpoints: tau = 0 reads p1 (the later frame), 1 reads p2. */
    Window w = sl.WindowAt(1550.0);
    CHECK_NEAR(sl.Read(w, 0.0f, InterpMode::LINEAR), w.p1->value, 1e-6);
    CHECK_NEAR(sl.Read(w, 1.0f, InterpMode::LINEAR), w.p2->value, 1e-6);
    CHECK_NEAR(sl.Read(w, 0.0f, InterpMode::CATMULLROM), w.p1->value, 1e-6);
    CHECK_NEAR(sl.Read(w, 1.0f, InterpMode::CATMULLROM), w.p2->value, 1e-6);
}

/* ── Analyzer ───────────────────────────────────────────────────────── */

static void test_analyzer()
{
    section("Analyzer");

    /* 10 periods of a 100 Hz sine: one keyframe per extremum (2/period)
     * plus the unconditional seed. */
    static SparseLine<1024> sl;
    static Analyzer<1024>   an;
    sl.Init();
    an.Init(sl, kFs);
    an.SetThreshold(0.001f);

    const float  f = 100.0f;
    const int    n = 4800;
    for (int i = 0; i < n; i++)
        an.Analyze(std::sin(2.0 * M_PI * f * i / kFs));

    const long long frames = (long long)an.KeyframeCount();
    std::printf("   sine keyframes: %lld (expect ~21)\n", frames);
    CHECK(frames >= 19 && frames <= 23);

    /* Keyframe timestamps vs the true extrema of the input (peaks at
     * n = 120 + 240k samples).  Measures the front end's systematic
     * offset; anything beyond ±2 samples would smear reconstruction. */
    double meanOff = 0.0, maxOff = 0.0;
    int    counted = 0;
    for (long long i = 1; i < frames; i++) {           // skip the seed frame
        const double t = sl.GetSample(i)->time;
        const double k = std::round((t - 120.0) / 240.0);
        const double off = t - (120.0 + 240.0 * k);
        meanOff += off;
        if (std::fabs(off) > maxOff) maxOff = std::fabs(off);
        counted++;
    }
    meanOff /= counted;
    std::printf("   extremum timestamp offset: mean %+.3f, max |%.3f| samples\n",
                meanOff, maxOff);
    CHECK(maxOff <= 0.5);

    /* Reconstruction: read the sparse buffer back on the uniform grid and
     * compare to the input.  Extrema + smoothstep should track a sine
     * closely; this is the paper's core claim in miniature. */
    double err2 = 0.0, sig2 = 0.0;
    const double lo = sl.GetSample(1)->time;
    const double hi = sl.GetLatest()->time;
    Window w = sl.WindowAt(lo + 1.0);
    for (double t = lo + 1.0; t < hi; t += 1.0) {
        sl.StepWindow(w, t);
        const float tau = (float)(w.endTime - t) * w.invDuration;
        const float y   = sl.Read(w, tau, InterpMode::CATMULLROM);
        const double x  = std::sin(2.0 * M_PI * f * (t - meanOff) / kFs);
        err2 += (y - x) * (y - x);
        sig2 += x * x;
    }
    const double nrmse = std::sqrt(err2 / sig2);
    std::printf("   sine reconstruction NRMSE: %.4f\n", nrmse);
    CHECK(nrmse < 0.1);

    /* Threshold monotonicity: a coarse ε must store no more keyframes than
     * a fine ε on the same signal (sine + small high-frequency ripple). */
    static SparseLine<1024> slA, slB;
    static Analyzer<1024>   anA, anB;
    slA.Init(); anA.Init(slA, kFs); anA.SetThreshold(0.001f);
    slB.Init(); anB.Init(slB, kFs); anB.SetThreshold(0.1f);
    for (int i = 0; i < n; i++) {
        const float x = std::sin(2.0 * M_PI * 100.0 * i / kFs)
                      + 0.02f * std::sin(2.0 * M_PI * 3000.0 * i / kFs);
        anA.Analyze(x);
        anB.Analyze(x);
    }
    std::printf("   keyframes fine ε: %lld, coarse ε: %lld\n",
                (long long)anA.KeyframeCount(), (long long)anB.KeyframeCount());
    CHECK(anB.KeyframeCount() < anA.KeyframeCount());

    /* GuardKeyframe: on an extremum-quiet signal (DC), coverage frames keep
     * the newest keyframe within maxGap of the write head. */
    static SparseLine<1024> slG;
    static Analyzer<1024>   anG;
    slG.Init();
    anG.Init(slG, kFs);
    for (int i = 0; i < 2000; i++) {
        anG.Analyze(0.5f);
        if (i % 64 == 63) anG.GuardKeyframe(500.0);
    }
    CHECK((double)anG.WriteHead() - anG.GetLatest()->time <= 500.0 + 64.0);
}

/* ── silence boundaries (Analyzer) ──────────────────────────────────── */

template <int N>
static float recon(SparseLine<N>& sl, double t)
{
    const Window w = sl.WindowAt(t);
    return sl.Read(w, (float)((w.endTime - t) * w.invDuration), InterpMode::CATMULLROM);
}

static void test_silence_bounds()
{
    section("silence boundaries");

    /* A tone cut off mid-swing, two seconds of near-silence (a 0.0004 offset
     * and +-0.0003 of noise: under the 0.001 threshold, so no keyframes),
     * then a tone at -0.3. Unbounded (1.2.0), the reconstruction drew the
     * silence as one smoothstep down to -0.3: a 0.3-high wave of 0.25 Hz.
     * Pinned, it is zero from the cut's crossing to the return's. (Digital
     * zero bounds itself: the flattening spline stores a 0 extremum.) */
    static SparseLine<16384> sl;
    static Analyzer<16384>   an;
    sl.Init();
    an.Init(sl, kFs);
    an.SetThreshold(0.001f);
    const int cut = 4800 + 37, back = cut + 96000, total = back + 4800;
    auto x = [&](int i) -> float {
        if (i < cut)  return 0.5f * (float)std::sin(2.0 * M_PI * 330.0 * i / kFs);
        if (i < back) return 0.0004f + 0.0003f * (float)(std::sin(i * 1.7) * std::sin(i * 0.013));
        return 0.3f * (float)std::sin(2.0 * M_PI * 330.0 * (i - back) / kFs + M_PI);
    };
    for (int i = 0; i < total; i++) an.Analyze(x(i));
    float inside = 0.0f, before = 0.0f;
    for (double t = cut + 600.0; t < back - 600.0; t += 7.0)
        inside = std::fmax(inside, std::fabs(recon(sl, t)));
    for (double t = back - 400.0; t < back - 2.0; t += 1.0)
        before = std::fmax(before, std::fabs(recon(sl, t)));
    std::printf("   recorded silence: worst %.2g inside, %.2g in the 400 samples before the return\n",
                (double)inside, (double)before);
    CHECK(inside < 0.001f);   /* (the offset itself where the tone settles, then 0) */
    CHECK(before < 0.01f);

    /* A loud 20 Hz sine is no silence: no zero keyframes in its slopes. */
    static SparseLine<16384> sl2;
    static Analyzer<16384>   an2;
    sl2.Init();
    an2.Init(sl2, kFs);
    an2.SetThreshold(0.001f);
    for (int i = 0; i < 48000; i++)
        an2.Analyze(0.8f * (float)std::sin(2.0 * M_PI * 20.0 * i / kFs + 0.3));
    int zeros = 0;
    for (long long k = sl2.GetOldestIndex() + 1; k <= sl2.GetLatestIndex(); k++)
        if (sl2.GetSample(k)->value == 0.0f) zeros++;
    std::printf("   20 Hz sine: %lld keyframes, %d of them pinned zeros\n",
                sl2.GetLatestIndex() - sl2.GetOldestIndex(), zeros);
    CHECK(zeros == 0);

    /* Live (guard frames every 480 samples): a tone under -60 dB after a loud
     * one reads as silence, not as a wobble between guards. */
    static SparseLine<16384> sl3;
    static Analyzer<16384>   an3;
    auto live = [&](float quietAmp, float eps, int& entries) {
        sl3.Init();
        an3.Init(sl3, kFs);
        an3.SetThreshold(eps);
        entries = 0;
        bool was = false;
        for (int b = 0; b < 300; b++) {
            for (int i = b * 64; i < b * 64 + 64; i++) {
                const float a = (i < 6400) ? 0.5f : quietAmp;
                an3.Analyze(a * (float)std::sin(2.0 * M_PI * 90.0 * i / kFs));
                if (an3.InSilence() && !was) entries++;
                was = an3.InSilence();
            }
            an3.GuardKeyframe(480.0);
        }
        float worst = 0.0f;
        for (double t = 8000.0; t < 18000.0; t += 3.0)
            worst = std::fmax(worst, std::fabs(recon(sl3, t)));
        return worst;
    };
    int entries = 0;
    const float quietTone = live(0.0005f, 0.01f, entries);
    std::printf("   live, a tone under -60 dB: worst %.2g\n", (double)quietTone);
    CHECK(quietTone == 0.0f);

    /* ... but a quiet tone above -60 dB is music, however coarse the
     * keyframe threshold: never gated (a floor tied to it chattered on real
     * material, several silences a second). */
    const float under = live(0.004f, 0.04f, entries);
    std::printf("   live, a -48 dB tone under a 0.04 threshold: %d silences, worst %.2g\n",
                entries, (double)under);
    CHECK(entries == 0);
    CHECK(under > 0.0f);

    /* A tail fading through the floor enters silence once: the return needs
     * twice the floor, so noise around it cannot flicker the gate. */
    sl3.Init();
    an3.Init(sl3, kFs);
    an3.SetThreshold(0.001f);
    rng_reset(3u);
    int flicks = 0;
    bool was = false;
    for (int i = 0; i < 96000; i++) {
        const float env = 0.003f * (1.0f - (float)i / 96000.0f);
        an3.Analyze(env * (float)std::sin(2.0 * M_PI * 90.0 * i / kFs) + 0.0003f * frand2());
        if ((i & 63) == 63) an3.GuardKeyframe(480.0);
        if (an3.InSilence() && !was) flicks++;
        was = an3.InSilence();
    }
    std::printf("   a tail fading through -60 dB: %d silence entries\n", flicks);
    CHECK(flicks == 1);
}

/* ── Detector ───────────────────────────────────────────────────────── */

static void test_detector()
{
    section("Detector");

    static Detector det;
    det.Init(kFs);

    /* Ten 3-sample bursts over a -60 dB noise floor, 0.2 s apart — wider
     * than kMinGapSec, so the hold-off never suppresses one and this stays a
     * test of detection rather than of density. */
    rng_reset(1234u);
    const int kBursts  = 10;
    const int kSpacing = 9600;   // 0.2 s @ 48 kHz
    for (int i = 0; i < kBursts * kSpacing; i++) {
        float x = 0.001f * frand2();
        if (i % kSpacing < 3) x += 1.0f;
        det.Analyze(x);
    }
    std::printf("   bursts: %d, kept events: %u, total events: %u\n",
                kBursts, det.Count(), det.TotalCount());
    CHECK(det.Count() >= 8 && det.Count() <= 14);

    /* Every kept event sits within 5 ms of a burst (envelope group delay). */
    uint32_t misplaced = 0;
    for (long long i = det.events.GetOldestIndex();
         i <= det.events.GetLatestIndex(); i++) {
        if (!det.EventKept(i)) continue;
        const double t   = det.events.GetSample(i)->time;   // seconds
        const double k   = std::round(t / 0.2);
        if (std::fabs(t - 0.2 * k) > 0.005) misplaced++;
    }
    CHECK(misplaced == 0);

    /* Dense hits: the same bursts 0.05 s apart all fire. Each decays to a
     * valley before the next, so the rise test passes them, and they lie
     * further apart than kMinGapSec. */
    static Detector dense;
    dense.Init(kFs);
    rng_reset(1234u);
    const int kTight = 2400;     // 0.05 s @ 48 kHz
    for (int i = 0; i < 30 * kTight; i++) {
        float x = 0.001f * frand2();
        if (i % kTight < 3) x += 1.0f;
        dense.Analyze(x);
    }
    std::printf("   dense: %u kept of %u bursts 50 ms apart\n", dense.Count(), 30u);
    CHECK(dense.Count() >= 29 && dense.Count() <= 30);

    /* The rise test: a tone struck once, then wobbling 10 % at 6 Hz. Each
     * wobble is an envelope peak over the stream average, but none rose from
     * a valley below kRise of its top: one kept event, the strike. */
    static Detector wobble;
    wobble.Init(kFs);
    wobble.SetCutoff(0.0014f);
    for (int i = 0; i < 48000; i++) {
        const double t = (double)i / kFs;
        const float  a = 0.5f * (1.0f + 0.1f * (float)std::sin(2.0 * M_PI * 6.0 * t));
        wobble.Analyze(a * (float)std::sin(2.0 * M_PI * 330.0 * t));
    }
    std::printf("   wobble: %u kept, %u stored\n", wobble.Count(), wobble.TotalCount());
    CHECK(wobble.Count() == 1);
    CHECK(wobble.TotalCount() > 3);

    /* Silence stores nothing (kAbsFloor), and the mute sentinel keeps
     * events but never fires. */
    static Detector quiet;
    quiet.Init(kFs);
    for (int i = 0; i < 48000; i++) quiet.Analyze(0.0f);
    CHECK(quiet.TotalCount() == 0);

    static Detector muted;
    muted.Init(kFs);
    muted.SetThreshold(Detector::kMuteAt);
    for (int i = 0; i < 48000; i++) {
        float x = (i % 4800 < 3) ? 1.0f : 0.0f;
        muted.Analyze(x);
    }
    CHECK(muted.Count() == 0);
    CHECK(muted.TotalCount() > 0);
}

/* ── Granule ────────────────────────────────────────────────────────── */

static void test_granule()
{
    section("Granule");

    /* Hand-built keyframe grid: alternating ±1 every 100 samples.  At
     * pitch = stretch = 1 the read is a smoothstep wave: keyframe values
     * are hit exactly at keyframe times, and the read never overshoots. */
    static SparseLine<1024> sl;
    sl.Init();
    for (int i = 0; i < 200; i++)
        sl.Write({(i & 1) ? -1.0f : 1.0f, 100.0 * i});

    static Granule<1024> g;
    g.Init(sl, 5000.0);
    g.BeginBlock(64);

    double t   = 5000.0;
    float  pre = 0.0f;
    for (int i = 0; i < 4000; i++, t += 1.0) {
        const float y = g.Read();
        CHECK(std::isfinite(y));
        CHECK(std::fabs(y) <= 1.0f + 1e-4f);
        const double phase = std::fmod(t, 100.0);
        if (phase < 1e-9) {   // exactly on a keyframe
            const float expect = ((long long)(t / 100.0) & 1) ? -1.0f : 1.0f;
            CHECK_NEAR(y, expect, 1e-4);
        }
        pre = y;
        (void)pre;
        if (g_failures > 10) return;
    }

    /* Fence invariant under pitch-up against a LIVE writer: the read head
     * must never pass the live edge (runway >= 0 modulo one block of
     * slack).  This is the front guard's whole job. */
    static SparseLine<1024> sl2;
    static Analyzer<1024>   an2;
    sl2.Init();
    an2.Init(sl2, kFs);
    static Granule<1024> g2;

    rng_reset(77u);
    /* Prime with enough live material for the fences to mean something. */
    for (int i = 0; i < 2000; i++)
        an2.Analyze(std::sin(2.0 * M_PI * 220.0 * i / kFs) + 0.1f * frand2());
    g2.Init(sl2, sl2.GetLatest()->time - 500.0);
    g2.SetPitch(2.0f);
    g2.SetStretch(1.0f);
    g2.SetLeash(16);
    g2.SetFade(480.0f);

    int clock = 2000;
    double worstRunway = 1e12;
    for (int blk = 0; blk < 3000; blk++) {
        for (int i = 0; i < 64; i++, clock++)
            an2.Analyze(std::sin(2.0 * M_PI * 220.0 * clock / kFs)
                        + 0.1f * frand2());
        g2.BeginBlock(64);
        for (int i = 0; i < 64; i++) {
            const float y = g2.Read();
            CHECK(std::isfinite(y));
            const double r = g2.GetRunway().uniform;
            if (r < worstRunway) worstRunway = r;
            if (g_failures > 10) return;
        }
    }
    std::printf("   worst runway at pitch 2.0: %.1f samples\n", worstRunway);
    CHECK(worstRunway > -64.0);

    /* ── the sparse leash ──────────────────────────────────────────────────
     * Keyframes 600 samples apart (a low note's extrema, past c = 48): at
     * pitch 2 against stretch 1, K = 128 keyframes alone would let the heads
     * drift 77k samples (1.6 s) apart. The sparse leash holds them at
     * K c^2 / spacing (~490, at least kSparseMin, plus one splice's travel).
     * Keyframes 12 apart (below c): the keyframe leash alone, as before. */
    {
        static SparseLine<16384> sp;
        static Granule<16384>    gs;
        const double spacing[2] = { 600.0, 12.0 };
        double worst[2] = { 0.0, 0.0 };
        for (int c = 0; c < 2; c++) {
            sp.Init();
            for (int i = 0; i < 16000; i++)
                sp.Write({(i & 1) ? -0.5f : 0.5f, spacing[c] * i});
            gs.Init(sp, spacing[c] * 12000.0);   /* (within the back guard's reach) */
            gs.SetPitch(2.0f);
            gs.SetStretch(1.0f);
            gs.SetLeash(128);
            gs.BeginBlock(64);
            for (int i = 0; i < 20000; i++) {
                const float y = gs.Read();
                CHECK(std::isfinite(y));
                const double d = std::fabs(gs.GetDistance().uniform);
                if (d > worst[c]) worst[c] = d;
            }
        }
        std::printf("   sparse leash: heads at most %.0f samples apart at spacing 600, "
                    "%.0f at spacing 12\n", worst[0], worst[1]);
        CHECK(worst[0] < 1500.0);                     /* not 128 x 600 */
        CHECK(worst[1] > 1500.0 && worst[1] < 4000.0);   /* 128 x 12, plus a splice */
    }

    /* ── reverse: the grid walks backward ──────────────────────────────────
     * A one-sided "past the end" bracket test never notices a cursor leaving
     * its window the OTHER way, so the window goes stale and the read sticks
     * at a clamped endpoint while the index keeps moving. Silent — no NaN, no
     * discontinuity — so assert on the thing that actually breaks: the window
     * must keep bracketing the index, and the output must keep moving. */
    static SparseLine<1024> sl3;
    sl3.Init();
    for (int i = 0; i < 400; i++)
        sl3.Write({(i & 1) ? -1.0f : 1.0f, 100.0 * i});

    static Granule<1024> g3;
    g3.Init(sl3, 30000.0);
    g3.SetPitch(1.0f);
    g3.SetStretch(-1.0f);          // grid reverses at realtime
    g3.SetLeash(4);                // splice often, so the grid is what drives
    g3.SetFade(240.0f);
    g3.BeginBlock(64);

    int   distinct = 0;
    float last     = 1e9f;
    double startIdeal = g3.GetIdeal().uniform;
    for (int i = 0; i < 20000; i++) {
        const float y = g3.Read();
        CHECK(std::isfinite(y));
        CHECK(std::fabs(y) <= 1.0f + 1e-3f);
        if (std::fabs(y - last) > 1e-6f) distinct++;
        last = y;
        if (g_failures > 10) return;
    }
    const double endIdeal = g3.GetIdeal().uniform;
    std::printf("   reverse: grid %.0f -> %.0f, %d distinct output steps\n",
                startIdeal, endIdeal, distinct);
    CHECK(endIdeal < startIdeal);          // the grid actually went backward
    CHECK(distinct > 19000);               // and the read never stuck

    /* The back edge is a loop point: reversing onto the oldest live frame
     * re-anchors to the delayed tap rather than clamping there forever. */
    g3.Init(sl3, 400.0);                   // start near the front of the buffer
    g3.SetStretch(-1.0f);
    g3.SetLeash(4);
    g3.BeginBlock(64);
    bool looped = false;
    for (int i = 0; i < 4000; i++) {
        const float y = g3.Read();
        CHECK(std::isfinite(y));
        if (g3.GetIdeal().uniform > 1000.0) looped = true;   // jumped forward
        if (g_failures > 10) return;
    }
    CHECK(looped);

    /* Reverse PITCH: the head walks backward. The splice phase advances with
     * the head's speed; signed, it would run backward and no fade would ever
     * land. Count fades that actually complete. */
    g3.Init(sl3, 12000.0);
    g3.SetPitch(-1.0f);
    g3.SetStretch(1.0f);
    g3.SetLeash(4);
    g3.BeginBlock(64);
    int landed = 0;
    bool wasSplicing = false;
    for (int i = 0; i < 15000; i++) {
        CHECK(std::isfinite(g3.Read()));
        if (wasSplicing && !g3.IsSplicing()) landed++;
        wasSplicing = g3.IsSplicing();
        if (g_failures > 10) return;
    }
    std::printf("   reverse pitch: %d fades landed\n", landed);
    CHECK(landed > 10);
}

/* ── KeyframeRecorder ───────────────────────────────────────────────── */

/* Shared blocks + a bounded-output assertion used by the recorder tests. */
static bool run_blocks(KeyframeRecorder<1024>& kr, int blocks,
                       float (*gen)(int), int& clock, float bound)
{
    float in[64], out[64];
    for (int b = 0; b < blocks; b++) {
        for (int i = 0; i < 64; i++, clock++) in[i] = gen(clock);
        kr.ProcessBlock(in, out, 64);
        for (int i = 0; i < 64; i++)
            if (!std::isfinite(out[i]) || std::fabs(out[i]) > bound)
                return false;
    }
    return true;
}

static float sine220(int n)
{ return std::sin(2.0 * M_PI * 220.0 * n / kFs); }

static float drums(int n)
{   /* 2 Hz clicks over a quiet 110 Hz drone — exercises the trigger path. */
    float x = 0.2f * std::sin(2.0 * M_PI * 110.0 * n / kFs);
    if (n % 24000 < 16) x += 0.9f;
    return x;
}

static void test_keyframe_recorder()
{
    section("KeyframeRecorder");

    static KeyframeRecorder<1024> kr;
    static SparseLine<1024> krRing;
    int clock = 0;

    /* STOPPED is a bit-exact passthrough. */
    kr.Init(krRing);
    float in[64], out[64];
    for (int i = 0; i < 64; i++) in[i] = frand2();
    kr.ProcessBlock(in, out, 64);
    bool same = true;
    for (int i = 0; i < 64; i++) same = same && (in[i] == out[i]);
    CHECK(same);

    /* LIVE_EFFECT at unity pitch/stretch reproduces a sine: after the
     * fade-in settles, output RMS matches input RMS closely and the
     * waveform is click-free (per-sample delta bounded by the sine's own
     * slew plus crossfade headroom). */
    kr.Init(krRing);
    kr.SetGrainFade(960.0f);
    kr.SubmitRequest(Request::LIVE_EFFECT);
    clock = 0;
    CHECK(run_blocks(kr, 150, sine220, clock, 4.0f));   // warmup ~0.2 s

    double rms = 0.0;
    float  prev = 0.0f, maxDelta = 0.0f;
    bool   first = true;
    for (int b = 0; b < 300; b++) {
        for (int i = 0; i < 64; i++, clock++) in[i] = sine220(clock);
        kr.ProcessBlock(in, out, 64);
        for (int i = 0; i < 64; i++) {
            rms += (double)out[i] * out[i];
            if (!first) {
                const float d = std::fabs(out[i] - prev);
                if (d > maxDelta) maxDelta = d;
            }
            prev  = out[i];
            first = false;
        }
    }
    rms = std::sqrt(rms / (300.0 * 64.0));
    const float sineSlew = 2.0f * (float)M_PI * 220.0f / kFs;   // ≈ 0.0288
    std::printf("   unity live: RMS %.3f (sine %.3f), max delta %.4f "
                "(slew %.4f)\n", rms, 1.0 / std::sqrt(2.0), maxDelta, sineSlew);
    CHECK_NEAR(rms, 1.0 / std::sqrt(2.0), 0.08);
    CHECK(maxDelta < 5.0f * sineSlew);

    /* No hit ever (the detector muted): the first grain alone, seated once the
     * ring holds a fade of input. 1.2.0 seated it at once, its tap clamped to
     * the start, and it read at the write head: full-scale jumps, a drone from
     * power-on played garbage until a hit. */
    {
        static KeyframeRecorder<1024> kq;
        static SparseLine<1024> kqRing;
        kq.Init(kqRing);
        kq.SetGrainFade(960.0f);
        kq.SetTransientThreshold(Detector::kMuteAt);
        kq.SubmitRequest(Request::LIVE_EFFECT);
        int qc = 0;
        float qin[64], qout[64], qprev = 0.0f, qd = 0.0f;
        double qrms = 0.0;
        int qn = 0;
        for (int b = 0; b < 450; b++) {
            for (int i = 0; i < 64; i++, qc++) qin[i] = sine220(qc);
            kq.ProcessBlock(qin, qout, 64);
            for (int i = 0; i < 64; i++) {
                if (b >= 150) { qrms += (double)qout[i] * qout[i]; qn++; }
                if (b > 0) qd = std::fmax(qd, std::fabs(qout[i] - qprev));
                qprev = qout[i];
            }
        }
        qrms = std::sqrt(qrms / qn);
        std::printf("   no hit since the start: RMS %.3f (sine 0.707), max delta %.4f (slew %.4f)\n",
                    qrms, qd, sineSlew);
        CHECK_NEAR(qrms, 1.0 / std::sqrt(2.0), 0.08);
        CHECK(qd < 5.0f * sineSlew);
    }

    /* Direction flips must not click. Reversing a moving head in place is a
     * corner — the slope flips sign — which only the second difference sees.
     * Flip every 40 blocks on the same sine and hold the worst corner to the
     * sine's own curvature. */
    float worstD2 = 0.0f, p1 = out[62], p2 = out[63];
    for (int b = 0; b < 400; b++) {
        if (b % 40 == 0) kr.SetGrainPitch((b / 40) & 1 ? 1.0f : -1.0f);
        for (int i = 0; i < 64; i++, clock++) in[i] = sine220(clock);
        kr.ProcessBlock(in, out, 64);
        for (int i = 0; i < 64; i++) {
            const float d2 = std::fabs(out[i] - 2.0f * p2 + p1);
            if (d2 > worstD2) worstD2 = d2;
            p1 = p2; p2 = out[i];
        }
    }
    const float sineD2 = sineSlew * sineSlew;   // ≈ 0.00083
    std::printf("   direction flips: worst corner %.5f (sine %.5f)\n", worstD2, sineD2);
    CHECK(worstD2 < 2.0f * sineD2);
    kr.SetGrainPitch(1.0f);

    /* Stretch + pitch shift on transient material: finite, bounded, and the
     * detector actually fires (the LIVE trigger path is exercised). */
    kr.Init(krRing);
    kr.SetGrainFade(960.0f);
    kr.SetGrainStretch(0.5f);
    kr.SetGrainPitch(1.26f);        // +4 st
    kr.SetGrainLeash(64);
    kr.SetTransientThreshold(2.0f);
    kr.SubmitRequest(Request::LIVE_EFFECT);
    clock = 0;
    CHECK(run_blocks(kr, 4000, drums, clock, 4.0f));    // ~5.3 s
    std::printf("   transients fired: %u\n", kr.TransientCount());
    CHECK(kr.TransientCount() >= 5);

    /* Punches at stretch 0.5, where the outgoing grain plays older material:
     * every hit leaves at the same latency, the write head's less the fade
     * (master, the newest keyframe's, jitters by a keyframe gap), quick and
     * slow strikes alike. */
    for (int slow = 0; slow < 2; slow++) {
        static KeyframeRecorder<16384> ks;
        static SparseLine<16384> ksRing;
        ks.Init(ksRing);
        ks.SetGrainFade(960.0f);
        ks.SetGrainStretch(0.5f);
        ks.SetTkeoCutoff(0.0014f);
        ks.SubmitRequest(Request::LIVE_EFFECT);
        const int kGap = 13000, kFirst = 5000, kHits = 10;
        const double tau = slow ? 0.04 : 0.005;
        const int from = 600;   /* (matched past the ramp: the strike's body) */
        const int total = kFirst + kHits * kGap;
        static std::vector<float> xi, yo;
        xi.assign(total, 0.0f); yo.assign(total, 0.0f);
        rng_reset(7u);
        for (int i = 0; i < total; i++) {
            float x = 0.001f * frand2();
            const int j = i - kFirst;
            if (j >= 0) {
                const double t = (double)(j % kGap) / kFs;
                x += 0.8f * (float)(std::sin(2.0 * M_PI * 180.0 * t) * std::exp(-t / tau));
            }
            xi[i] = x;
        }
        for (int b = 0; b + 64 <= total; b += 64) ks.ProcessBlock(&xi[b], &yo[b], 64);
        int lagLo = 1 << 30, lagHi = -1;
        double worstRatio = 1.0;
        for (int h = 1; h < kHits; h++) {        /* (the first: no grain before it) */
            const int S = kFirst + h * kGap;
            int best = 0; double bestE = 1e30;
            for (int L = 0; L < 3000; L++) {
                double e = 0.0;
                for (int k = from; k < from + 400; k += 4) {
                    const double d = yo[S + L + k] - xi[S + k];
                    e += d * d;
                }
                if (e < bestE) { bestE = e; best = L; }
            }
            double pin = 0.0, pout = 0.0;
            for (int k = 0; k < 400; k++) {
                pin  = std::fmax(pin,  std::fabs(xi[S + k]));
                pout = std::fmax(pout, std::fabs(yo[S + best + k]));
            }
            if (best < lagLo) lagLo = best;
            if (best > lagHi) lagHi = best;
            worstRatio = std::fmin(worstRatio, pout / pin);
        }
        std::printf("   punches, %s strikes at stretch 0.5: latency %d..%d samples (fade 960), "
                    "attack peak %.3f of the input's\n", slow ? "slow" : "quick", lagLo, lagHi, worstRatio);
        /* (the tap: the write head less the fade, read a block behind it) */
        CHECK(lagHi - lagLo <= 16);
        CHECK(lagLo >= 960 - 64 - 16 && lagHi <= 960 - 64 + 16);
    }

    /* RECORD → STOP → PLAYBACK round trip stays finite through state churn. */
    kr.Init(krRing);
    kr.SubmitRequest(Request::RECORD);
    clock = 0;
    CHECK(run_blocks(kr, 500, sine220, clock, 4.0f));
    kr.SubmitRequest(Request::STOP_REC);
    CHECK(run_blocks(kr, 2, sine220, clock, 4.0f));
    CHECK(kr.GetState() == State::STOPPED);
    kr.SubmitRequest(Request::PLAY);
    CHECK(run_blocks(kr, 500, sine220, clock, 4.0f));
    CHECK(kr.GetState() == State::PLAYBACK);
}

/* ── Stereo alignment (lib/StereoAlign.h) ───────────────────────────── */

/* Strikes every 13000 samples; each channel's level and the right channel's
 * delay set per case. */
static float strike(int i, int delay)
{
    const int j = i - 5000 - delay;
    if (j < 0) return 0.0f;
    const double t = (double)(j % 13000) / kFs;
    return 0.8f * (float)(std::sin(2.0 * M_PI * 180.0 * t) * std::exp(-t / 0.04));
}

struct StereoRun { uint32_t firesL, firesR, follows; double worstDrift, driftAfterHits; int longestDrift; };

static StereoRun run_stereo(float gainR, int delayR, float threshR, int samples)
{
    static KeyframeRecorder<16384> l, r;
    static SparseLine<16384> lRing, rRing;
    static StereoAlign<16384> al;
    l.Init(lRing);
    r.Init(rRing);
    for (KeyframeRecorder<16384>* k : { &l, &r }) {
        k->SetGrainFade(960.0f);
        k->SetGrainStretch(0.5f);
        k->SetTkeoCutoff(0.0014f);
        k->SetTransientThreshold(2.0f);
        k->SubmitRequest(Request::LIVE_EFFECT);
    }
    r.SetTransientThreshold(threshR);
    al.Init();
    StereoRun out = { 0, 0, 0, 0.0, 0.0, 0 };
    float il[64], ir[64], ol[64], orr[64];
    rng_reset(5u);
    int driftRun = 0;
    for (int b = 0; b + 64 <= samples; b += 64) {
        for (int i = 0; i < 64; i++) {
            il[i] = strike(b + i, 0) + 0.001f * frand2();
            ir[i] = gainR * strike(b + i, delayR) + 0.001f * frand2();
        }
        l.ProcessBlock(il, ol, 64);
        r.ProcessBlock(ir, orr, 64);
        out.firesL += l.FiredThisBlock();
        out.firesR += r.FiredThisBlock();
        al.Process(l, r, 64);
        const double d = std::fabs(l.GridLag() - r.GridLag());
        if (d > out.worstDrift) out.worstDrift = d;
        driftRun = (d > StereoAlign<16384>::kDriftLong) ? driftRun + 64 : 0;
        if (driftRun > out.longestDrift) out.longestDrift = driftRun;
        /* 300 ms after each strike: both channels have dealt with it */
        if ((b - 5000) % 13000 >= 14400 - 13000 && (b - 5000) % 13000 < 14400 - 13000 + 64 && b > 5000)
            out.driftAfterHits = std::fmax(out.driftAfterHits, d);
    }
    out.follows = al.Follows();
    return out;
}

static void test_stereo_align()
{
    section("stereo alignment (coincidence window, drift)");
    const int n = 5000 + 12 * 13000;

    /* The same hit on both sides, the right one 5 ms later and under its own
     * keep level (a higher threshold) but near it: it never fires on its own,
     * so each left hit is followed and the grids sit together. */
    const StereoRun co = run_stereo(1.0f, 240, 3.6f, n);
    std::printf("   coincident: L fired %u, R fired %u, follows %u, drift after hits %.0f\n",
                co.firesL, co.firesR, co.follows, co.driftAfterHits);
    CHECK(co.firesL >= 11);
    CHECK(co.follows + co.firesR >= co.firesL);
    CHECK(co.driftAfterHits < 600.0);   /* (a keyframe gap of jitter in GridLag) */

    /* A left-only hit: the right channel hears nothing near its keep level, so
     * no coincidence; at stretch 0.5 its grid drifts from the left's, and the
     * drift ends within kDriftPatience (plus a block). */
    const StereoRun solo = run_stereo(0.0f, 0, 2.0f, n);
    std::printf("   one-sided: L fired %u, R fired %u, follows %u, longest drift %d samples\n",
                solo.firesL, solo.firesR, solo.follows, solo.longestDrift);
    CHECK(solo.firesR == 0);
    CHECK(solo.follows >= 1);
    CHECK(solo.longestDrift <= StereoAlign<16384>::kDriftPatience + 64);
}

/* ── Trigger stream (the panel's TRIG 1 / P3 source) ────────────────── */

/* Clicks every 60 ms over a quiet drone: tighter than a long crossfade, so the
 * recorder's refractory thins them; the detector's 23 ms gap does not. */
static float fast_clicks(int n)
{
    float x = 0.2f * std::sin(2.0 * M_PI * 110.0 * n / kFs);
    if (n % 2880 < 16) x += 0.9f;
    return x;
}

/* Block indices where the engine actually punched — TransientCount() edges,
 * which is exactly the stream the trig out pulses on. */
static std::vector<int> punch_blocks(KeyframeRecorder<1024>& kr, int blocks,
                                     int& clock)
{
    std::vector<int> hits;
    float    in[64], out[64];
    uint32_t last = kr.TransientCount();
    for (int b = 0; b < blocks; b++) {
        for (int i = 0; i < 64; i++, clock++) in[i] = fast_clicks(clock);
        kr.ProcessBlock(in, out, 64);
        if (kr.TransientCount() != last) {
            last = kr.TransientCount();
            hits.push_back(b);
        }
    }
    return hits;
}

/* Gaps are measured in whole blocks, so a punch can read one block late. */
static double min_gap_sec(const std::vector<int>& hits)
{
    double lo = 1e9;
    for (size_t i = 1; i < hits.size(); i++) {
        const double g = (double)(hits[i] - hits[i - 1]) * 64.0 / (double)kFs;
        if (g < lo) lo = g;
    }
    return lo;
}

static void test_trigger_stream()
{
    section("trigger stream (TRIG 1 / P3 source)");

    static KeyframeRecorder<1024> kr;
    static SparseLine<1024> krRing;
    const double blockSec = 64.0 / (double)kFs;

    /* Boot fade (20 ms): nothing thins 60 ms clicks, and no two punches — and
     * therefore no two trigger pulses — land closer than kMinGapSec. */
    kr.Init(krRing);
    kr.SetGrainFade(960.0f);          /* 20 ms */
    kr.SetTransientThreshold(2.0f);
    kr.SubmitRequest(Request::LIVE_EFFECT);
    int clock = 0;
    const std::vector<int> shortFade = punch_blocks(kr, 3000, clock);  /* ~4 s */
    const double shortGap = min_gap_sec(shortFade);
    std::printf("   fade 20 ms:  %zu punches, min gap %.3f s (gap %.3f)\n",
                shortFade.size(), shortGap, (double)Detector::kMinGapSec);
    CHECK(shortFade.size() >= 8);
    CHECK(shortGap >= (double)Detector::kMinGapSec - blockSec);

    /* Top-of-range fade (250 ms): the recorder's own refractory thins the
     * stream. This is the case the old
     * follower-gate LED/CV path got wrong: it pulsed on every kept event while
     * the engine was still refusing the punch. */
    kr.Init(krRing);
    kr.SetGrainFade(12000.0f);        /* 250 ms — top of the UI range */
    kr.SetTransientThreshold(2.0f);
    kr.SubmitRequest(Request::LIVE_EFFECT);
    clock = 0;
    const std::vector<int> longFade = punch_blocks(kr, 3000, clock);
    const double longGap = min_gap_sec(longFade);
    std::printf("   fade 250 ms: %zu punches, min gap %.3f s (fade %.3f)\n",
                longFade.size(), longGap, 12000.0 / (double)kFs);
    CHECK(longFade.size() >= 4);
    CHECK(longFade.size() < shortFade.size());          /* genuinely thinned */
    CHECK(longGap >= 12000.0 / (double)kFs - blockSec);
}

/* ── fuzz: random params, hostile input, small ring ─────────────────── */

static void test_fuzz()
{
    section("fuzz (small ring, random params, hostile input)");

    /* A 128-frame ring wraps every few hundred ms: the writer laps the
     * grid constantly, hammering the back-stop guard.  Parameters sweep
     * the full UI ranges every ~80 ms while the input switches between
     * noise, clicks, sines, DC and silence.  Everything must stay finite
     * and bounded — this is the "12h soak in 2 seconds" test. */
    static KeyframeRecorder<128> kr;
    static SparseLine<128> krRing;
    kr.Init(krRing);
    kr.SubmitRequest(Request::LIVE_EFFECT);
    rng_reset(0xBADC0DEu);

    float in[64], out[64];
    int   clock = 0;
    bool  ok = true;
    float worst = 0.0f;
    for (int b = 0; b < 12000 && ok; b++) {             // ~16 s of audio
        if (b % 60 == 0) {
            kr.SetGrainPitch(std::exp2(2.0f * frand2()));      // ±24 st
            kr.SetGrainStretch(0.01f + 0.99f * frand());
            kr.SetGrainLeash(8 + (int)(frand() * 120.0f));
            kr.SetGrainFade(48.0f + frand() * 12000.0f);
            kr.SetThreshold(0.001f + frand() * 0.1f);
            kr.SetTransientThreshold(frand() * 8.0f);
            kr.SetTkeoCutoff(0.0005f + frand() * 0.05f);
            kr.SetTkeoResonance(frand());
            kr.SetDistortDrive(0.1f + frand() * 4.0f);
            kr.SetDistortCharacter(frand());
        }
        if (b % 500 == 250) kr.SubmitRequest(Request::SLICE);

        const int mode = (b / 100) % 5;
        for (int i = 0; i < 64; i++, clock++) {
            switch (mode) {
                case 0: in[i] = frand2(); break;
                case 1: in[i] = (clock % 4800 < 8) ? 0.95f : 0.0f; break;
                case 2: in[i] = std::sin(2.0 * M_PI * 1300.0 * clock / kFs); break;
                case 3: in[i] = 0.7f; break;
                default: in[i] = 0.0f; break;
            }
        }
        kr.ProcessBlock(in, out, 64);
        for (int i = 0; i < 64; i++) {
            if (!std::isfinite(out[i])) { ok = false; break; }
            const float a = std::fabs(out[i]);
            if (a > worst) worst = a;
        }
        if (worst > 8.0f) ok = false;
    }
    std::printf("   fuzz: %s, worst |out| = %.3f\n", ok ? "clean" : "FAILED", worst);
    CHECK(ok);
}

/* ── main ───────────────────────────────────────────────────────────── */

/* ── CV jack routing (src/jack_routing.h) ───────────────────────────────
 * Every option combination against a grid of follower states. Expectations
 * are written out from the spec, not computed with jacks::Volts, so a bug in
 * the shared code cannot vouch for itself. */
static void test_jack_routing()
{
    section("jack routing");
    using namespace jacks;

    /* Factory (1.3): all four jacks are outputs: Trig 1 the triggers, Env 1 TKEO, Trig 2 CONT, Env 2 PULS; the
     * output follower is not among the mod sources. */
    {
        const Options o;
        CHECK(!o.cvIn);
        CHECK(!o.outFollower);
        CHECK(o.trig2 == kInCont);
        CHECK(o.env2 == kInPuls);
    }

    /* What a jack carrying `sig` must read, straight from the spec. */
    /* (the contours ride with fixed values distinct from every envelope: kContIn, kContOut) */
    static constexpr float kContIn = 0.33f, kContOut = 0.77f, kPulsIn = 0.21f, kPulsOut = 0.64f;
    auto expect = [](uint8_t sig, bool inTrig, bool outTrig, float inEnv, float outEnv) {
        auto clamp01 = [](float v) { return v < 0.0f ? 0.0f : v > 1.0f ? 1.0f : v; };
        if (sig == kInTrig)  return inTrig  ? 5.0f : 0.0f;
        if (sig == kOutTrig) return outTrig ? 5.0f : 0.0f;
        if (sig == kInEnv)   return 5.0f * clamp01(inEnv);
        if (sig == kInCont)  return 5.0f * kContIn;
        if (sig == kOutCont) return 5.0f * kContOut;
        if (sig == kInPuls)  return 5.0f * kPulsIn;
        if (sig == kOutPuls) return 5.0f * kPulsOut;
        return 5.0f * clamp01(outEnv);
    };

    /* Distinct in/out values, so a swapped signal can't pass by coincidence;
     * out-of-range envelopes check the clamp. */
    const float inEnvs[]  = {0.0f, 0.25f, 1.0f, 1.4f, -0.2f};
    const float outEnvs[] = {0.6f, 0.05f, 0.9f};

    int combos = 0;
    for (int cv = 0; cv < 2; cv++)
    for (uint8_t t2 = 0; t2 < kNumSignals; t2++)
    for (uint8_t e2 = 0; e2 < kNumSignals; e2++)
    {
        Options o;
        o.cvIn = cv; o.trig2 = t2; o.env2 = e2;
        combos++;
        for (int it = 0; it < 2; it++)
        for (int ot = 0; ot < 2; ot++)
        for (float ie : inEnvs)
        for (float oe : outEnvs)
        {
            Signals sg;
            sg.inTrig = it; sg.outTrig = ot; sg.inEnv = ie; sg.outEnv = oe;
            sg.inCont = kContIn; sg.outCont = kContOut; sg.inPuls = kPulsIn; sg.outPuls = kPulsOut;
            const Plan p = Route(o, sg);
            if (!cv)
            {
                /* Outputs mode: all four are driven; Trig 1 the triggers, Env 1 the input envelope, and Trig 2 /
                 * Env 2 what was picked for each. */
                CHECK(p.output[kTrig1] && p.output[kEnv1] && p.output[kTrig2] && p.output[kEnv2]);
                CHECK_NEAR(p.volts[kTrig1], expect(kInTrig,  it, ot, ie, oe), 1e-6);
                CHECK_NEAR(p.volts[kEnv1],  expect(kInEnv,   it, ot, ie, oe), 1e-6);
                CHECK_NEAR(p.volts[kTrig2], expect(t2, it, ot, ie, oe), 1e-6);
                CHECK_NEAR(p.volts[kEnv2],  expect(e2, it, ot, ie, oe), 1e-6);
            }
            else
            {
                /* CV-input mode: Trig 1 / Env 1 are never driven; Trig 2 and
                 * Env 2 carry exactly what was picked for each. */
                CHECK(!p.output[kTrig1] && !p.output[kEnv1]);
                CHECK(p.output[kTrig2] && p.output[kEnv2]);
                CHECK_NEAR(p.volts[kTrig2], expect(t2, it, ot, ie, oe), 1e-6);
                CHECK_NEAR(p.volts[kEnv2],  expect(e2, it, ot, ie, oe), 1e-6);
            }
        }
    }
    CHECK(combos == 2 * kNumSignals * kNumSignals);

    /* Preset bytes: every option survives a save and a load. */
    for (int cv = 0; cv < 2; cv++)
    for (int of = 0; of < 2; of++)
    for (uint8_t t2 = 0; t2 < kNumSignals; t2++)
    for (uint8_t e2 = 0; e2 < kNumSignals; e2++)
    {
        Options o;
        o.cvIn = cv; o.outFollower = of; o.trig2 = t2; o.env2 = e2;
        uint8_t b[kBytes];
        Encode(o, b);
        const Options r = Decode(b);
        CHECK(r.cvIn == o.cvIn && r.outFollower == o.outFollower && r.trig2 == t2 && r.env2 == e2);
    }

    /* 1.2's bytes load as they were: 0 / 1 in byte 0 are its outputs / inputs, the output follower off. */
    {
        const uint8_t old12[kBytes] = {1, kInTrig, kInEnv};
        const Options r = Decode(old12);
        CHECK(r.cvIn && !r.outFollower && r.trig2 == kInTrig && r.env2 == kInEnv);
    }

    /* Corrupt or foreign bytes: every value of every byte decodes to
     * something legal, and anything unknown means the defaults. */
    for (int v = 0; v < 256; v++)
    {
        const uint8_t b0[kBytes] = {(uint8_t)v, 1, 3};
        CHECK(Decode(b0).cvIn == (v <= 3 && (v & 1)));
        CHECK(Decode(b0).outFollower == (v <= 3 && (v & 2)));
        const uint8_t b1[kBytes] = {1, (uint8_t)v, 3};
        CHECK(Decode(b1).trig2 == (v < kNumSignals ? v : kInCont));
        const uint8_t b2[kBytes] = {1, 0, (uint8_t)v};
        CHECK(Decode(b2).env2 == (v < kNumSignals ? v : kInPuls));
    }
}

static void test_mod_swap()
{
    section("mod swap");
    using modswap::Swap;

    /* The swap: before and after a fade, exactly depth x the source; in between, never a jump (the signals here are
     * still, so every step is the fade's own), and a second switch mid-fade starts from where the blend stood. */
    {
        float sig[3] = {1.0f, -1.0f, 0.5f};
        Swap w;
        CHECK(w.Out(0, 0.8f, sig, 0) == 0.8f);           // (the first source: no fade)
        CHECK(w.Out(0, 0.8f, sig, 16) == 0.8f);
        float prev = 0.8f, worst = 0.0f;
        for (uint32_t t = 100; t <= 100 + Swap::kFadeMs; t += 2)
        {
            const float v = w.Out(1, 0.6f, sig, t);
            const float d = v > prev ? v - prev : prev - v;
            if (d > worst) worst = d;
            prev = v;
        }
        CHECK(prev == -0.6f);                             // (done: the new source at its depth)
        CHECK(worst < 1.4f * 2.0f / Swap::kFadeMs + 1e-4f);   // (0.8 -> -0.6 over the fade, a 2 ms step at a time)

        float mid = 0.0f;
        for (uint32_t t = 200; t <= 220; t += 2) mid = w.Out(2, 1.0f, sig, t);   // (a third of the way to 0.5)
        const float at = w.Out(0, 0.2f, sig, 222);                            // (switch again, mid-fade)
        CHECK((at > mid ? at - mid : mid - at) < 0.03f);
        float end = 0.0f;
        for (uint32_t t = 224; t <= 300; t += 2) end = w.Out(0, 0.2f, sig, t);
        CHECK(end == 0.2f);
    }
}

int main()
{
    test_deluxe_line();
    test_diffuser();
    test_adaptive_comb();
    test_pcm();
    test_contour();
    test_pulse();
    test_shape_level();
    test_shapers();
    test_filters();
    test_sparse_line();
    test_analyzer();
    test_silence_bounds();
    test_detector();
    test_granule();
    test_keyframe_recorder();
    test_stereo_align();
    test_trigger_stream();
    test_fuzz();
    test_jack_routing();
    test_mod_swap();

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
