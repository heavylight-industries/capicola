/**
 * @file tests/host/diffuse_render.cpp
 * @brief Offline render of the feedback loop with the adaptive comb in it, to stereo wav.
 *
 *   diffuse_render <out.wav> <diffusion 0..1> <spread 0..1> [feedback 0..1.5] [input.wav] [pitch] [stretch] [kfMul]
 *
 * <diffusion> is the comb's Size (0 off .. 1). kfMul scales the grain length it reaches (default 1, the engine's).
 * The audio engine's feedback chain on two live recorders: last block's output -> sinc sat (x0.5) -> SVF bandpass
 * (fc 0.02, q 0.01) -> AdaptiveComb -> x feedback, clamped to +-1, summed into the input. Output 100% wet; 3 s of
 * silence after the input, for the tail. Without an input: drums and a chord (blend_render's source).
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "AdaptiveComb.h"
#include "KeyframeRecorder.h"
#include "Shapers.h"
#include "filter.h"

using namespace capicola;

static constexpr float kFs = 48000.0f;

static bool write_wav(const char* path, const std::vector<float>& l, const std::vector<float>& r)
{
    FILE* f = std::fopen(path, "wb");
    if (!f) return false;
    const uint32_t n = (uint32_t)l.size(), data = n * 4;
    const uint32_t fs = (uint32_t)kFs, bps = fs * 4;
    uint8_t h[44] = {'R','I','F','F',0,0,0,0,'W','A','V','E','f','m','t',' ',
                     16,0,0,0,1,0,2,0,0,0,0,0,0,0,0,0,4,0,16,0,'d','a','t','a',0,0,0,0};
    const uint32_t riff = 36 + data;
    std::memcpy(h + 4,  &riff, 4);
    std::memcpy(h + 24, &fs,   4);
    std::memcpy(h + 28, &bps,  4);
    std::memcpy(h + 40, &data, 4);
    std::fwrite(h, 1, 44, f);
    for (uint32_t i = 0; i < n; i++) {
        for (float v : {l[i], r[i]}) {
            if (v > 1.0f) v = 1.0f; else if (v < -1.0f) v = -1.0f;
            const int16_t s = (int16_t)std::lrintf(v * 32767.0f);
            std::fwrite(&s, 2, 1, f);
        }
    }
    std::fclose(f);
    return true;
}

// PCM 16/24/32 or float 32, any channels (the first two), any rate (linear resample to 48 kHz).
static bool read_wav(const char* path, std::vector<float>& l, std::vector<float>& r)
{
    FILE* f = std::fopen(path, "rb");
    if (!f) return false;
    std::vector<uint8_t> b;
    uint8_t buf[65536];
    size_t got;
    while ((got = std::fread(buf, 1, sizeof buf, f)) > 0) b.insert(b.end(), buf, buf + got);
    std::fclose(f);
    if (b.size() < 12 || std::memcmp(b.data(), "RIFF", 4) || std::memcmp(b.data() + 8, "WAVE", 4)) return false;
    int fmt = 0, ch = 0, bits = 0;
    uint32_t rate = 0;
    const uint8_t* pcm = nullptr;
    uint32_t pcmLen = 0;
    for (size_t p = 12; p + 8 <= b.size();) {
        uint32_t len;
        std::memcpy(&len, b.data() + p + 4, 4);
        const uint8_t* c = b.data() + p + 8;
        if (!std::memcmp(b.data() + p, "fmt ", 4)) {
            fmt = c[0] | c[1] << 8;
            ch = c[2] | c[3] << 8;
            std::memcpy(&rate, c + 4, 4);
            bits = c[14] | c[15] << 8;
            if (fmt == 0xFFFE) fmt = c[24] | c[25] << 8;
        } else if (!std::memcmp(b.data() + p, "data", 4)) {
            pcm = c;
            pcmLen = (uint32_t)std::min<size_t>(len, b.size() - (p + 8));
        }
        p += 8 + len + (len & 1);
    }
    if (!pcm || ch < 1 || rate == 0) return false;
    const int bytes = bits / 8, frame = bytes * ch;
    const size_t n = pcmLen / frame;
    auto sample = [&](size_t i, int c) -> float {
        const uint8_t* s = pcm + i * frame + c * bytes;
        if (fmt == 3 && bits == 32) { float v; std::memcpy(&v, s, 4); return v; }
        if (bits == 16) return (int16_t)(s[0] | s[1] << 8) / 32768.0f;
        if (bits == 24) return (int32_t)((uint32_t)(s[0] << 8 | s[1] << 16 | s[2] << 24)) / 2147483648.0f;
        if (bits == 32) { int32_t v; std::memcpy(&v, s, 4); return v / 2147483648.0f; }
        return 0.0f;
    };
    const double step = (double)rate / kFs;
    for (double t = 0.0; t + 1.0 < (double)n; t += step) {
        const size_t i = (size_t)t;
        const float fr = (float)(t - i);
        const int cr = ch > 1 ? 1 : 0;
        l.push_back(sample(i, 0) + fr * (sample(i + 1, 0) - sample(i, 0)));
        r.push_back(sample(i, cr) + fr * (sample(i + 1, cr) - sample(i, cr)));
    }
    return true;
}

static float source(int n)
{
    const float t = n / kFs;
    float x = 0.12f * (std::sin(2.0 * M_PI * 220.0 * t)
                     + std::sin(2.0 * M_PI * 277.2 * t)
                     + std::sin(2.0 * M_PI * 330.0 * t)) * (std::fmod(t, 2.0f) < 0.4f ? 1.0f : 0.0f);
    const float tk = std::fmod(t, 0.5f);
    if (tk < 0.15f)
        x += 0.8f * std::sin(2.0 * M_PI * 60.0 * tk) * std::exp(-18.0f * tk);
    const float th = std::fmod(t + 0.25f, 0.5f);
    if (th < 0.03f) {
        static uint32_t r = 1u;
        r ^= r << 13; r ^= r >> 17; r ^= r << 5;
        x += 0.35f * ((float)(r >> 8) / 8388608.0f - 1.0f) * std::exp(-120.0f * th);
    }
    return x;
}

int main(int argc, char** argv)
{
    if (argc < 4) {
        std::fprintf(stderr, "usage: diffuse_render <out.wav> <diffusion> <spread> [feedback] [input.wav] [pitch] [stretch] [kfMul]\n");
        return 2;
    }
    const float diffusion = (float)std::atof(argv[2]);
    const float spread    = (float)std::atof(argv[3]);
    const float fbAmt     = argc > 4 ? (float)std::atof(argv[4]) : 0.8f;
    const float pitch     = argc > 6 ? (float)std::atof(argv[6]) : 1.0f;
    const float stretch   = argc > 7 ? (float)std::atof(argv[7]) : 1.0f;
    const float kfMul     = argc > 8 ? (float)std::atof(argv[8]) : 1.0f;   // the taps' grain length x kfMul

    std::vector<float> inL, inR;
    if (argc > 5 && argv[5][0]) {
        if (!read_wav(argv[5], inL, inR)) { std::fprintf(stderr, "can't read %s\n", argv[5]); return 1; }
    } else {
        for (int i = 0; i < (int)kFs * 8; i++) { inL.push_back(source(i)); inR.push_back(inL.back()); }
    }
    const size_t tail = (size_t)kFs * 3;
    inL.resize(inL.size() + tail, 0.0f);
    inR.resize(inR.size() + tail, 0.0f);
    const size_t total = inL.size() / 64 * 64;

    static Shapers shapers;
    shapers.Init();
    static KeyframeRecorder<65536> kr[2];
    static SparseLine<65536> ring[2];
    static AdaptiveComb diff[2];
    StateVariable svf[2];
    for (int c = 0; c < 2; c++) {
        kr[c].Init(ring[c], &shapers);
        kr[c].SetThreshold(0.001f);
        kr[c].SetGrainPitch(pitch);
        kr[c].SetGrainStretch(stretch);
        kr[c].SetGrainLeash(128);
        kr[c].SetGrainFade(960.0f);
        kr[c].SetTransientThreshold(2.0f);
        kr[c].SubmitRequest(Request::LIVE_EFFECT);
        svf[c].Init();
        svf[c].SetControls(0.02f, 0.01f);
        diff[c].Init();
        diff[c].SetSize(diffusion);
        (void)spread;   // (one knob: Size spreads the taps too)
    }

    std::vector<float> outL(total), outR(total);
    std::vector<float> periods;   // the left taps' period, a block
    float fb[2][64] = {}, in[2][64], out[2][64];
    for (size_t b = 0; b < total / 64; b++) {
        for (int c = 0; c < 2; c++) {
            const std::vector<float>& src = c ? inR : inL;
            diff[c].SetKeyframes(kr[c].KeyframeSpacing(), kfMul * kr[c].GrainLength());
            diff[c].Block(64);
            if (c == 0) periods.push_back(diff[0].TapBase());
            for (int i = 0; i < 64; i++) {
                svf[c].Tick(0.5f * shapers.ReadSinc(fb[c][i]));
                float inj = diff[c].Process(svf[c].GetBandpass()) * fbAmt;
                if (inj > 1.0f) inj = 1.0f; else if (inj < -1.0f) inj = -1.0f;
                in[c][i] = src[b * 64 + i] + inj;
            }
            kr[c].ProcessBlock(in[c], out[c], 64);
            std::memcpy(fb[c], out[c], sizeof fb[c]);
            for (int i = 0; i < 64; i++) (c ? outR : outL)[b * 64 + i] = out[c][i];
        }
    }

    double rms = 0.0, tailRms = 0.0;
    float peak = 0.0f;
    bool finite = true;
    for (size_t i = 0; i < total; i++) {
        const float v = 0.5f * (outL[i] + outR[i]);
        if (!std::isfinite(outL[i]) || !std::isfinite(outR[i])) finite = false;
        rms += (double)v * v;
        if (std::fabs(outL[i]) > peak) peak = std::fabs(outL[i]);
        if (std::fabs(outR[i]) > peak) peak = std::fabs(outR[i]);
        if (i >= total - (size_t)kFs) tailRms += (double)v * v;
    }
    std::printf("%s: diffusion %.2f spread %.2f feedback %.2f: RMS %.4f, peak %.3f, last second RMS %.5f%s\n",
                argv[1], diffusion, spread, fbAmt, std::sqrt(rms / total), peak, std::sqrt(tailRms / kFs),
                finite ? "" : "  NOT FINITE");
    std::vector<float> ps(periods.begin(), periods.end() - (long)(tail / 64));
    std::sort(ps.begin(), ps.end());
    if (!ps.empty())
        std::printf("   comb's L, folded (left, while the input plays): 10%% %.0f, median %.0f, 90%% %.0f samples\n",
                    ps[ps.size() / 10], ps[ps.size() / 2], ps[ps.size() * 9 / 10]);
    return write_wav(argv[1], outL, outR) ? 0 : 1;
}
