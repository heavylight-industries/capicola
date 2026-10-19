#include <atomic>
#include <cmath>
#include <cstdio>
#include <thread>
#include "audio_engine.h"

int main()
{
    AlchemyLabAudio::AudioEngine engine;
    engine.Init();
    engine.SetTransientThreshold(1.0e9f); // Isolate forced slices.
    engine.SetSlicePitch(0.7f);
    engine.SetSliceStretch(0.2f);
    engine.SetGrainSize(256);
    float input[64] = {}, left[64], right[64];
    engine.ProcessBlock(input, input, left, right, 64); // Enter LIVE_EFFECT.
    std::atomic<bool> stop{false};
    std::atomic<unsigned> requests{0};
    std::thread control([&] {
        while (!stop.load(std::memory_order_acquire))
        {
            engine.TriggerSlice();
            requests.fetch_add(1, std::memory_order_relaxed);
            std::this_thread::yield();
        }
    });
    unsigned mismatches = 0;
    for (int block = 0; block < 1200; ++block)
    {
        for (int i = 0; i < 64; ++i)
            input[i] = 0.6f * std::sin((block * 64 + i) * 0.029f);
        engine.ProcessBlock(input, input, left, right, 64);
        for (int i = 0; i < 64; ++i)
            if (left[i] != right[i]) ++mismatches;
    }
    stop.store(true, std::memory_order_release);
    control.join();
    std::printf("%u concurrent slice requests, %u stereo sample mismatches\n",
                requests.load(), mismatches);
    return mismatches || requests.load() == 0 ? 1 : 0;
}
