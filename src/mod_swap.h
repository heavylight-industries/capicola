/**
 * @file mod_swap.h
 * @brief The swap between a knob's mod sources: the pure part, no hardware.
 *
 * Each knob plays one source at a time, and switching it on the routing page (by hand, or all six with control
 * all) crossfades, so the routing page is played live (heavylight, 2026-11-02). ModRouter (main.cpp) runs one Swap
 * per knob; tests/host checks it.
 */

#pragma once

#include <cstdint>

namespace modswap {

/* One knob's modulation, depth × its source's signal, crossfading from the source before over kFadeMs when the
 * source changes: both sides live, the old one at the depth it had. A change in the middle of a fade fades from
 * where the blend stood, held, so nothing ever jumps. */
class Swap
{
  public:
    static constexpr uint32_t kFadeMs = 50;

    float Out(uint8_t src, float depth, const float* sig, uint32_t t_ms)
    {
        const float now = depth * sig[src];
        if (src != cur)
        {
            fromLive  = !fading;
            held      = last;
            from      = cur;
            fromDepth = lastDepth;
            cur       = src;
            t0        = t_ms;
            fading    = started;   // (the first source has nothing to fade from)
        }
        started   = true;
        lastDepth = depth;
        float v = now;
        if (fading)
        {
            const uint32_t e = t_ms - t0;
            if (e >= kFadeMs)
                fading = false;
            else
            {
                const float x = (float)e / (float)kFadeMs;
                const float a = fromLive ? fromDepth * sig[from] : held;
                v = a + (now - a) * x;
            }
        }
        return last = v;
    }

  private:
    uint8_t  cur       = 0;
    uint8_t  from      = 0;
    bool     started   = false;
    bool     fading    = false;
    bool     fromLive  = false;
    float    fromDepth = 0.0f;
    float    lastDepth = 0.0f;
    float    held      = 0.0f;
    float    last      = 0.0f;
    uint32_t t0        = 0;
};

} // namespace modswap
