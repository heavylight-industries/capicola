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
#include "SparseLine.h"
#include "Analyzer.h"
#include "Detector.h"
#include "Granule.h"
#include "KeyframeRecorder.h"
#include "Shapers.h"
#include "filter.h"

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

/* ── DeluxeLine ─────────────────────────────────────────────────────── */

static void test_deluxe_line()
{
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

/* ── Detector ───────────────────────────────────────────────────────── */

static void test_detector()
{
    section("Detector");

    static Detector det;
    det.Init(kFs);

    /* Ten 3-sample bursts over a -60 dB noise floor, 0.1 s apart. */
    rng_reset(1234u);
    const int kBursts = 10;
    for (int i = 0; i < 48000; i++) {
        float x = 0.001f * frand2();
        if (i % 4800 < 3 && i < kBursts * 4800) x += 1.0f;
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
        const double k   = std::round(t / 0.1);
        if (std::fabs(t - 0.1 * k) > 0.005) misplaced++;
    }
    CHECK(misplaced == 0);

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
    int clock = 0;

    /* STOPPED is a bit-exact passthrough. */
    kr.Init();
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
    kr.Init();
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

    /* Stretch + pitch shift on transient material: finite, bounded, and the
     * detector actually fires (the LIVE trigger path is exercised). */
    kr.Init();
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

    /* RECORD → STOP → PLAYBACK round trip stays finite through state churn. */
    kr.Init();
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
    kr.Init();
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

int main()
{
    test_deluxe_line();
    test_shapers();
    test_filters();
    test_sparse_line();
    test_analyzer();
    test_detector();
    test_granule();
    test_keyframe_recorder();
    test_fuzz();

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
