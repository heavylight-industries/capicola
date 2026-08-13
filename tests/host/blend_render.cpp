/**
 * @file tests/host/blend_render.cpp
 * @brief Offline A/B renderer: drums + pad through the live engine, to wav.
 *
 *   blend_render <out.wav> [input.wav]
 *
 * Same engine settings every run — rebuild with a different crossfade law
 * in the lib and diff the outputs by ear.
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "KeyframeRecorder.h"

using namespace capicola;

static constexpr float kFs = 48000.0f;
static constexpr int   kSeconds = 8;

static bool write_wav(const char* path, const std::vector<float>& x)
{
    FILE* f = std::fopen(path, "wb");
    if (!f) return false;
    const uint32_t n = (uint32_t)x.size(), data = n * 2;
    const uint32_t fs = (uint32_t)kFs, bps = fs * 2;
    uint8_t h[44] = {'R','I','F','F',0,0,0,0,'W','A','V','E','f','m','t',' ',
                     16,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,2,0,16,0,'d','a','t','a',0,0,0,0};
    const uint32_t riff = 36 + data;
    std::memcpy(h + 4,  &riff, 4);
    std::memcpy(h + 24, &fs,   4);
    std::memcpy(h + 28, &bps,  4);
    std::memcpy(h + 40, &data, 4);
    std::fwrite(h, 1, 44, f);
    for (uint32_t i = 0; i < n; i++) {
        float v = x[i];
        if (v > 1.0f) v = 1.0f; else if (v < -1.0f) v = -1.0f;
        const int16_t s = (int16_t)std::lrintf(v * 32767.0f);
        std::fwrite(&s, 2, 1, f);
    }
    std::fclose(f);
    return true;
}

static float source(int n)
{
    const float t = n / kFs;
    float x = 0.12f * (std::sin(2.0 * M_PI * 220.0 * t)
                     + std::sin(2.0 * M_PI * 277.2 * t)
                     + std::sin(2.0 * M_PI * 330.0 * t));

    const float tk = std::fmod(t, 0.5f);                       // kick, 2 Hz
    if (tk < 0.15f)
        x += 0.8f * std::sin(2.0 * M_PI * 60.0 * tk) * std::exp(-18.0f * tk);

    const float th = std::fmod(t + 0.25f, 0.5f);               // noise hat
    if (th < 0.03f) {
        static uint32_t r = 1u;
        r ^= r << 13; r ^= r >> 17; r ^= r << 5;
        x += 0.35f * ((float)(r >> 8) / 8388608.0f - 1.0f) * std::exp(-120.0f * th);
    }
    return x;
}

int main(int argc, char** argv)
{
    if (argc < 2) { std::fprintf(stderr, "usage: blend_render <out.wav> [input.wav]\n"); return 2; }

    static KeyframeRecorder<65536> kr;
    kr.Init();
    kr.SetThreshold(0.001f);
    kr.SetGrainStretch(0.6f);
    kr.SetGrainPitch(std::exp2(3.0f / 12.0f));
    kr.SetGrainLeash(128);
    kr.SetGrainFade(960.0f);
    kr.SetTransientThreshold(2.0f);
    kr.SubmitRequest(Request::LIVE_EFFECT);

    const int total = (int)kFs * kSeconds;
    std::vector<float> dry((size_t)total), wet((size_t)total);
    float in[64], out[64];
    for (int b = 0; b < total / 64; b++) {
        for (int i = 0; i < 64; i++) in[i] = source(b * 64 + i);
        kr.ProcessBlock(in, out, 64);
        for (int i = 0; i < 64; i++) {
            dry[(size_t)(b * 64 + i)] = in[i];
            wet[(size_t)(b * 64 + i)] = out[i];
        }
    }

    double rms = 0.0, hf = 0.0;
    float  maxDelta = 0.0f;
    for (int i = 1; i < total; i++) {
        rms += (double)wet[i] * wet[i];
        const float d = wet[i] - wet[i - 1];
        hf += (double)d * d;
        if (std::fabs(d) > maxDelta) maxDelta = std::fabs(d);
    }
    std::printf("%s: RMS %.4f, first-diff RMS %.5f, max delta %.4f, splices %u\n",
                argv[1], std::sqrt(rms / total), std::sqrt(hf / total),
                maxDelta, kr.TransientCount());

    if (!write_wav(argv[1], wet)) return 1;
    if (argc > 2 && !write_wav(argv[2], dry)) return 1;
    return 0;
}
