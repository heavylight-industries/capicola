/**
 * @file audio_engine.cpp
 * @brief Capicola audio engine — see audio_engine.h.
 */

#include "audio_engine.h"

#include <cmath>
#include "daisy_seed.h"
#include "KeyframeRecorder.h"
#include "StereoAlign.h"
#ifdef CAPICOLA_FB_DIFFUSER
#include "AdaptiveComb.h"   /* lib/: the feedback path's adaptive comb */
#endif
#include "Shapers.h"

namespace AlchemyLabAudio {

using namespace capicola;

/* Two independent keyframe rings in external SDRAM (one per channel). 2^21
 * frames × 12 B = 24 MB each → ~48 MB total, sharing the 64 MB region with
 * HostLink's staging buffers. Globals because the DSY_SDRAM_BSS section
 * attribute cannot apply to a class member. */
static constexpr int kRingFrames = 2097152;   // 2^21 per channel
static SparseLine<kRingFrames> DSY_SDRAM_BSS ring_l;
static SparseLine<kRingFrames> DSY_SDRAM_BSS ring_r;

/* The input as PCM, int16, in the SDRAM the keyframe rings leave (lib/PcmRing.h): the top of Quality plays from
 * here. 3.6 M samples a channel, 75 s, 7.2 MB each (~1.5 MB of the 64 MB left). Their write heads, 7.2 MB apart,
 * land 104 cache sets from each other. */
static constexpr uint32_t kPcmSamples = 3600000;
static int16_t DSY_SDRAM_BSS pcmBuf_l[kPcmSamples];
static int16_t DSY_SDRAM_BSS pcmBuf_r[kPcmSamples];
static capicola::PcmRing pcm_l, pcm_r;   // (small: internal RAM)

/* The recorders themselves (grains, analyzer, detector: ~60 KB each) in the
 * AXI SRAM, not SDRAM: their per-sample state is hot. NOLOAD and trivially
 * constructible like the rings, set up in Init(). (.axi_bss:
 * cmake/linkers/capicola_sram.lds) */
#define CAPICOLA_AXI_BSS __attribute__((section(".axi_bss")))
static KeyframeRecorder<kRingFrames> CAPICOLA_AXI_BSS sparse_l;
static KeyframeRecorder<kRingFrames> CAPICOLA_AXI_BSS sparse_r;
static StereoAlign<kRingFrames> align;   // (small: internal RAM)

#ifdef CAPICOLA_FB_DIFFUSER
/* The adaptive comb in the feedback loop (lib/AdaptiveComb.h), one per channel, after the feedback bandpass: taps
 * L back, L from the channel's own keyframes, run by the Comb page (P1 Size: 0 = off .. one keyframe period ..
 * the grain's length, its taps spreading as it goes; P2 the feedback bandpass's centre, kept in the Secondary
 * store). In D2 SRAM (.d2_bss, 32 KB a channel). (The
 * nested all-pass diffuser it replaced, lib/Diffuser.h, cost 15-16 % of every block on the unit.) */
static AdaptiveComb __attribute__((section(".d2_bss"))) fbComb[2];
#endif

/* Waveshaper LUTs — 3 × 4096 floats. Kept in SRAM, not SDRAM: read every
 * keyframe. Shared by both channels and the feedback saturator. */
static Shapers shapers;

namespace {
/* OLA crossfade boot value (secondary P2 sets it live): the monitoring latency
 * AND the retrigger floor. */
constexpr float kFadeSamples = 960.0f;    // 20 ms @ 48 kHz

/* Feedback bandpass centre (normalised fc). 0.02 ≈ 480 Hz. What goes round is the bandpass normalised to a unity
 * peak (2R bp): 1.2 fed back the raw bandpass, whose peak at Q 0.54 is 0.54, so its top Feedback was a loop gain
 * of 0.8 and the loop could never hold itself up. */
constexpr float kFbFilterFc = 0.02f;
/* The Feedback knob (0 .. 1.5, as 1.2 stores it) as a loop gain: x4/3, so noon is 1 (the loop holds itself up)
 * and the top 2 (it builds, held by the sinc and the clamp; a level control is the user's to patch: the
 * followers are mod sources). */
constexpr float kFbKnobGain = 4.0f / 3.0f;

/* Cancels the sinc shaper's 2x small-signal gain so the loop matches tanh. */
constexpr float kFbSatGain = 0.5f;

/* Follower SVF cutoff (envelope smoothing), shared by the input detectors and
 * the output follower. */
constexpr float kTkeoFc = 0.002f;   // (~48 Hz; 1.2's 0.0014, ~34 Hz: a few more triggers, heavylight 2026-11-02)

/* Envelope expansion for the normalised follower outputs (UI pip, mod sources,
 * CV outs): the low-passed Teager energy is tiny, so norm = sqrt(clamp01(env ×
 * kEnvGain)). */
constexpr float kEnvGain = 200.0f;

/* A hit's strength for the contours: its envelope peak in dB of energy (10 log10) against the envelope's full scale
 * (1 / kEnvGain, where EnvNorm reaches 1), kHitRangeDb below it 0. */
constexpr float kHitRangeDb = 40.0f;
inline float HitStrength(float peak)
{
    const float e = peak * kEnvGain;
    if (e <= 0.0f) return 0.0f;
    const float s = 1.0f + 10.0f * std::log10(e) / kHitRangeDb;
    return s < 0.0f ? 0.0f : s > 1.0f ? 1.0f : s;
}

inline float EnvNorm(float env)
{
    float e = env * kEnvGain;
    if (e < 0.0f) e = 0.0f; else if (e > 1.0f) e = 1.0f;
    return std::sqrt(e);
}
} // namespace

#ifdef CAPICOLA_PROFILE
/* The core's cycle counter (DWT), for the stage stamps in lib/ and here. */
extern "C" uint32_t capicola_cycles(void) { return DWT->CYCCNT; }
#define PROF_STAMP(v) const uint32_t v = capicola_cycles()
#define PROF_SET(field, value) ctl.field = (value)
#else
#define PROF_STAMP(v)
#define PROF_SET(field, value)
#endif

void AudioEngine::Init()
{
#ifdef CAPICOLA_PROFILE
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR   = 0xC5ACCE55;   // (the M7's lock access register)
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
    /* CPU load meter for the audio callback (48 kHz, 64-sample blocks). */
    cpuMeter.Init(48000.0f, (int)kMaxBlock);

    /* Tabulate the waveshapers before the recorders bind to them. */
    shapers.Init();

    /* Start live capture + granular playback on both channels. */
    pcm_l.Init(pcmBuf_l, kPcmSamples);
    pcm_r.Init(pcmBuf_r, kPcmSamples);
    sparse_l.Init(ring_l, &shapers, &pcm_l);
    sparse_r.Init(ring_r, &shapers, &pcm_r);
    sparse_l.SetThreshold(0.001f);
    sparse_r.SetThreshold(0.001f);
    sparse_l.SetGrainPitch(1.0f);
    sparse_r.SetGrainPitch(1.0f);
    sparse_l.SetGrainLeash(128);
    sparse_r.SetGrainLeash(128);
    sparse_l.SetGrainFade(kFadeSamples);   // fixed fade = the module latency
    sparse_r.SetGrainFade(kFadeSamples);

    fbSvfL.Init();
    fbSvfR.Init();
#ifdef CAPICOLA_FB_DIFFUSER
    for (int c = 0; c < 2; c++) fbComb[c].Init();
#endif
    fbSvfL.SetControls(kFbFilterFc, 0.01f);
    fbSvfR.SetControls(kFbFilterFc, 0.01f);

    /* Output-side follower: same chain as the input detectors, run on the final
     * mono-summed output. Never punches — signal source only. */
    outDet.Init(48000.0f);
    contourIn.Init();
    contourOut.Init();
    pulseIn.Init();
    pulseOut.Init();
    nPending = 0;
    pitchT = pitchS = 1.0f;
    stretchT = stretchS = 1.0f;
    fbAmtT = fbAmt = 0.0f;
    mixT = mix = 1.0f;
    outSeen   = 0;
    sampleNow = 0;
    SetTkeoCutoff(kTkeoFc);
    sliceCount = 0;
    envNormIn  = envNormOut  = 0.0f;
    inPeak     = 0.0f;

    align.Init();
    sparse_l.SubmitRequest(Request::LIVE_EFFECT);
    sparse_r.SubmitRequest(Request::LIVE_EFFECT);
}

void AudioEngine::ProcessBlock(const float* in_l, const float* in_r,
                               float*       out_l, float*       out_r,
                               std::size_t  n)
{
    cpuMeter.OnBlockStart();
    PROF_STAMP(t0);

    const std::size_t m = (n < kMaxBlock) ? n : kMaxBlock;
    float fbFrom, mixFrom;
    GlideControls(m, fbFrom, mixFrom);

    /* Feedback: last block's wet output → sinc → SVF bandpass (normalised to a unity peak) → the comb, scaled
     * (kFbKnobGain) and summed into the recorder input. Hard ±1 clamp on the
     * injection. */
    const float* src_l = in_l;
    const float* src_r = in_r;
    float fin_l[kMaxBlock], fin_r[kMaxBlock];
    if (fbAmt > 0.0f || fbFrom > 0.0f)
    {
        const float dFb = (fbAmt - fbFrom) / (float)m;
#ifdef CAPICOLA_FB_DIFFUSER
        fbComb[0].SetKeyframes(sparse_l.KeyframeSpacing(), sparse_l.GrainLength());   // (last block's: L)
        fbComb[1].SetKeyframes(sparse_r.KeyframeSpacing(), sparse_r.GrainLength());
        fbComb[0].Block((int)m);
        fbComb[1].Block((int)m);
#endif
        for (std::size_t i = 0; i < m; i++)
        {
            fbSvfL.Tick(kFbSatGain * shapers.ReadSinc(fbL[i]));
            fbSvfR.Tick(kFbSatGain * shapers.ReadSinc(fbR[i]));
#if defined(CAPICOLA_FB_DIFFUSER)
            float vl = fbComb[0].Process(fbSvfL.GetNormBandpass());
            float vr = fbComb[1].Process(fbSvfR.GetNormBandpass());
#else
            float vl = fbSvfL.GetNormBandpass();
            float vr = fbSvfR.GetNormBandpass();
#endif

#ifdef CAPICOLA_PROFILE
            if (!std::isfinite(vl)) { vl = 0.0f; prof.nanFb++; }   // (the probe: counted, kept out of the loop)
            if (!std::isfinite(vr)) { vr = 0.0f; prof.nanFb++; }
#endif
            const float g = fbFrom + dFb * (float)(i + 1);   // (the gain ramped through the block)
            float inj_l = vl * g;
            float inj_r = vr * g;
            if (inj_l > 1.0f) inj_l = 1.0f; else if (inj_l < -1.0f) inj_l = -1.0f;
            if (inj_r > 1.0f) inj_r = 1.0f; else if (inj_r < -1.0f) inj_r = -1.0f;

            fin_l[i] = in_l[i] + inj_l;
            fin_r[i] = in_r[i] + inj_r;
        }
        src_l = fin_l;
        src_r = fin_r;
    }

    PROF_STAMP(t1);
    sparse_l.ProcessBlock(src_l, out_l, m);
    sparse_r.ProcessBlock(src_r, out_r, m);
    PROF_STAMP(t2);
#ifdef CAPICOLA_PROFILE
    for (std::size_t i = 0; i < m; i++) {
        if (!std::isfinite(out_l[i])) { out_l[i] = 0.0f; prof.nanOut++; }
        if (!std::isfinite(out_r[i])) { out_r[i] = 0.0f; prof.nanOut++; }
    }
#endif

    /* Stereo alignment (lib/StereoAlign.h): the coincidence window, the drift
     * guardrail, and drift that outlasts its patience. */
    align.Process(sparse_l, sparse_r, (int)m);
    PROF_STAMP(t3);

    /* One slice event per block in which either channel punched. A follow
     * above realigns the quiet channel onto the same hit, not a second musical
     * hit, so it deliberately does not add a tick. */
    if (sparse_l.FiredThisBlock() || sparse_r.FiredThisBlock()) sliceCount++;

    /* Stash this block's wet (pitched) output for next block's feedback, before
     * the dry/wet blend overwrites the out buffers. */
    for (std::size_t i = 0; i < m; i++) { fbL[i] = out_l[i]; fbR[i] = out_r[i]; }

    /* Dry/wet blend (P5): dry = live input, wet = stretched output (lagged). */
    const float dMix = (mix - mixFrom) / (float)m;   // (ramped through the block: GlideControls)
    for (std::size_t i = 0; i < m; i++)
    {
        const float wet = mixFrom + dMix * (float)(i + 1);
        const float dry = 1.0f - wet;
        out_l[i] = wet * out_l[i] + dry * in_l[i];
        out_r[i] = wet * out_r[i] + dry * in_r[i];
    }

    /* Output follower consumes the final (post-mix) mono sum. Fed at 2× (no 0.5
     * halving): TKEO is quadratic, so this lifts the envelope 4× to sit level
     * with the per-channel input followers. Event timing is unaffected — the
     * dynamic threshold is a scale-invariant ratio over its own average. */
    for (std::size_t i = 0; i < m; i++)
        outDet.Analyze(out_l[i] + out_r[i]);

    /* The contours (block rate): a slice this block is an input hit, a new kept output-follower event an output
     * hit; each at this block's end, its strength the hit's envelope peak (the louder channel's). */
    sampleNow += m;
    const double now = (double)sampleNow / 48000.0;
    {
        const bool fl = sparse_l.FiredThisBlock(), fr = sparse_r.FiredThisBlock();
        if (fl || fr) {
            const float vl = fl ? sparse_l.LastHitValue() : 0.0f, vr = fr ? sparse_r.LastHitValue() : 0.0f;
            const float sIn = FoldHit(HitStrength(vl > vr ? vl : vr));
            // (heard a Fade later: the punch seats the head one Fade back; the delay on top, which the pulse may
            // also rise into)
            pulseIn.Hit(sIn, now, fadeSec + modDelay);
            QueueContour(0, sIn, now + modDelay);
        }
        if (outDet.Count() != outSeen) {
            outSeen = outDet.Count();
            const float sOut = FoldHit(HitStrength(outDet.lastKept));
            pulseOut.Hit(sOut, now, modDelay);    // (heard as it is detected: only the delay to rise into)
            QueueContour(1, sOut, now + modDelay);
        }
        // The contours' hits as they come due (in time order: one delay for all).
        int keep = 0;
        for (int i = 0; i < nPending; i++) {
            if (pending[i].due <= now) (pending[i].side ? contourOut : contourIn).Hit(pending[i].s, pending[i].due);
            else pending[keep++] = pending[i];
        }
        nPending = keep;
        contourIn.Tick(now);
        contourOut.Tick(now);
        pulseIn.Tick(now, (float)m / 48000.0f);
        pulseOut.Tick(now, (float)m / 48000.0f);
    }

    const float env_l = sparse_l.TkeoEnvelope();
    const float env_r = sparse_r.TkeoEnvelope();
    envNormIn  = EnvNorm((env_l > env_r) ? env_l : env_r);
    envNormOut = EnvNorm(outDet.Envelope());

    /* Input peak meter (decaying abs max) — the UI clip indicator reads it. */
    constexpr float kPeakDecay = 0.95f;
    float pk = inPeak * kPeakDecay;
    for (std::size_t i = 0; i < m; i++)
    {
        const float al = std::fabs(in_l[i]);
        const float ar = std::fabs(in_r[i]);
        if (al > pk) pk = al;
        if (ar > pk) pk = ar;
    }
    inPeak = pk;

    cpuMeter.OnBlockEnd();
    cpuAvg = cpuMeter.GetAvgCpuLoad();
    cpuMax = cpuMeter.GetMaxCpuLoad();

#ifdef CAPICOLA_PROFILE
    const uint32_t t4 = capicola_cycles();
    const capicola::RecorderStats sl = sparse_l.Stats();
    const capicola::RecorderStats sr = sparse_r.Stats();
    const uint32_t st[kStages] = { t1 - t0, sl.capCycles, sl.renderCycles,
                                   sr.capCycles, sr.renderCycles,
                                   t3 - t2, t4 - t3, t4 - t0 };
    prof.blocks++;
    for (int k = 0; k < kStages; k++) {
        prof.sum[k] += st[k];
        if (st[k] > prof.max[k]) prof.max[k] = st[k];
    }
#ifdef CAPICOLA_FB_DIFFUSER
    /* The probe's breaker: a real stall only (16 blocks running over 98 % of the budget) turns the comb off
     * (the page's knob turned again turns it back on), so the main loop and the stream keep running. (At 8 over
     * 90 % it tripped on busy material that was keeping up.) */
    static int over = 0;
    over = (st[kStTotal] > BlockBudgetCycles() / 100u * 98u) ? over + 1 : 0;
    if (over >= 16) {
        fbComb[0].SetSize(0.0f); fbComb[1].SetSize(0.0f);
        prof.trips++; over = 0;
    }
#endif
#endif
}

#ifdef CAPICOLA_PROFILE
void AudioEngine::TakeProfile(Profile& out)
{
    __disable_irq();
    out  = prof;
    prof = {};
    __enable_irq();
}

capicola::RecorderStats AudioEngine::StatsL() const { return sparse_l.Stats(); }
capicola::RecorderStats AudioEngine::StatsR() const { return sparse_r.Stats(); }

#ifdef CAPICOLA_FB_DIFFUSER
float AudioEngine::DiffAmount() const { return fbComb[0].Amount(); }
float AudioEngine::DiffPeriod(int ch) const { return fbComb[ch ? 1 : 0].TapBase(); }
#else
float AudioEngine::DiffAmount() const { return 0.0f; }
float AudioEngine::DiffPeriod(int) const { return 0.0f; }
#endif

uint32_t AudioEngine::BlockBudgetCycles()
{
    return (uint32_t)((uint64_t)SystemCoreClock * kMaxBlock / 48000u);
}
#endif

void AudioEngine::SetSlicePitch(float speed)
{
    PROF_SET(pitch, speed);
    pitchT = speed;   // (GlideControls takes it there)
}

void AudioEngine::SetSliceStretch(float stretch)
{
    PROF_SET(stretch, stretch);
    stretchT = stretch;
}

/* Once a block, first: each control a step toward its target (kCtlGlide: ~8 ms), the rates handed to the recorders,
 * the gains' starting points returned so the block ramps them sample by sample. */
constexpr float kCtlGlide = 0.153f;   // 1 - exp(-1.33 ms / 8 ms): a block's share

void AudioEngine::GlideControls(std::size_t m, float& fbFrom, float& mixFrom)
{
    (void)m;
    auto glide = [](float& s, float t, bool flipJumps) {
        if (flipJumps && s * t < 0.0f) { s = t; return; }   // (a polarity flip is a jump, as the gesture means)
        s += (t - s) * kCtlGlide;
        if (std::fabs(t - s) < 1e-6f) s = t;
    };
    const float p0 = pitchS, s0 = stretchS;
    glide(pitchS, pitchT, true);
    glide(stretchS, stretchT, true);
    if (pitchS != p0)   { sparse_l.SetGrainPitch(pitchS);     sparse_r.SetGrainPitch(pitchS); }
    if (stretchS != s0) { sparse_l.SetGrainStretch(stretchS); sparse_r.SetGrainStretch(stretchS); }
    fbFrom = fbAmt;
    glide(fbAmt, fbAmtT, false);
    mixFrom = mix;
    glide(mix, mixT, false);
}

void AudioEngine::SetGrainSize(float keyframes)
{
    PROF_SET(grain, keyframes);
    const int d = (int)(keyframes + 0.5f);
    sparse_l.SetGrainLeash(d);
    sparse_r.SetGrainLeash(d);
}

void AudioEngine::SetQuality(float epsilon)
{
    PROF_SET(quality, epsilon);
    sparse_l.SetThreshold(epsilon);
    sparse_r.SetThreshold(epsilon);
}

bool AudioEngine::PcmOn() const { return sparse_l.PcmOn(); }

void AudioEngine::SetPcm(bool on)
{
    sparse_l.SetPcm(on);
    sparse_r.SetPcm(on);
}

void AudioEngine::SetFeedback(float amt)
{
    PROF_SET(feedback, amt);
    fbAmtT = (amt < 0.0f) ? 0.0f : amt * kFbKnobGain;
}

void AudioEngine::SetCombSize(float amt01)
{
#ifdef CAPICOLA_FB_DIFFUSER
    PROF_SET(diffusion, amt01);
    fbComb[0].SetSize(amt01);
    fbComb[1].SetSize(amt01);
#else
    (void)amt01;
#endif
}

void AudioEngine::SetFeedbackCutoff(float fc)
{
    /* P6·shift — open the feedback filter wide for more transient bite; Q stays
     * at the gentle 0.01 (see Init), only fc moves. */
    fbSvfL.SetControls(fc, 0.01f);
    fbSvfR.SetControls(fc, 0.01f);
}

void AudioEngine::SetMix(float wet01)
{
    PROF_SET(mix, wet01);
    mixT = Clamp01(wet01);
}

void AudioEngine::SetTransientThreshold(float t)
{
    PROF_SET(thresh, t);
    sparse_l.SetTransientThreshold(t);
    sparse_r.SetTransientThreshold(t);
    outDet.SetThreshold(t);        // followers share threshold
}

void AudioEngine::SetTkeoCutoff(float fc)
{
    sparse_l.SetTkeoCutoff(fc);
    sparse_r.SetTkeoCutoff(fc);
    outDet.SetCutoff(fc);          // followers share smoothing
}

void AudioEngine::SetTkeoResonance(float q)
{
    /* Finder SVF resonance, UNCAPPED (filter.h maps Q = 0.5 + 4q); large q rings
     * into self-oscillation. Input side only. */
    const float qq = (q < 0.0f) ? 0.0f : q;
    sparse_l.SetTkeoResonance(qq);
    sparse_r.SetTkeoResonance(qq);
}

void AudioEngine::SetFade(float samples)
{
    PROF_SET(fade, samples);
    sparse_l.SetGrainFade(samples);
    sparse_r.SetGrainFade(samples);
    fadeSec = samples / 48000.0f;
}

/* The fold (heavylight, 2026-11-02): each hit's strength through a sine folder before the contours and pulses draw
 * it, so loud hits fold back down, past zero, and the accents of both reorder together: per hit, so nothing aliases.
 * sin(pi/2 (1 + kFoldGain x amount) s) from the waveshapers' sine table; faded in over the knob's first kFoldIn so
 * 0 is the strengths exactly. At the top a full-strength hit folds through four periods. */
constexpr float kFoldGain = 15.0f;
constexpr float kFoldIn   = 0.05f;

float AudioEngine::FoldHit(float s) const
{
    if (hitFold <= 0.0f) return s;
    const float f = shapers.ReadSin(1.5707963f * (1.0f + kFoldGain * hitFold) * s);
    const float w = hitFold < kFoldIn ? hitFold / kFoldIn : 1.0f;
    return s + w * (f - s);
}

void AudioEngine::SetHitFold(float amount01)
{
    hitFold = amount01 < 0.0f ? 0.0f : amount01 > 1.0f ? 1.0f : amount01;
}

/* The delay (heavylight, 2026-11-02): CONT and PULS later than their hits. A pulse's centre moves (Pulse::Hit's lead
 * is the Fade plus the delay, so a delayed pulse also rises into it); a contour's hit waits in a small queue. */
void AudioEngine::SetModDelay(float seconds)
{
    modDelay = seconds < 0.0f ? 0.0f : seconds;
}

void AudioEngine::QueueContour(int side, float s, double due)
{
    if (nPending == kPending) {   // (full: the oldest goes now, early)
        (pending[0].side ? contourOut : contourIn).Hit(pending[0].s, pending[0].due);
        for (int i = 1; i < kPending; i++) pending[i - 1] = pending[i];
        nPending--;
    }
    pending[nPending++] = {due, s, side};
}

void AudioEngine::SetPulseWidth(float seconds)
{
    pulseIn.SetWidth(seconds);
    pulseOut.SetWidth(seconds);
}

void AudioEngine::SetDrive(float d)
{
    sparse_l.SetDistortDrive(d);
    sparse_r.SetDistortDrive(d);
}

void AudioEngine::SetDriveCharacter(float c)
{
    sparse_l.SetDistortCharacter(c);
    sparse_r.SetDistortCharacter(c);
}

void AudioEngine::TriggerSlice()
{
    if (sparse_l.GetState() == State::LIVE_EFFECT)
    {
        sparse_l.SubmitRequest(Request::SLICE);
        sparse_r.SubmitRequest(Request::SLICE);
    }
}

} // namespace AlchemyLabAudio
