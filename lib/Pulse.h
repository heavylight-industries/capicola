#pragma once

#include <cmath>

namespace capicola {

// PULS: one Gaussian per hit, as tall as the hit is strong, as wide as the width (the dt2-keyframer's LFO wave,
// DESIGN 7: p(t) = max over hits of s_k exp(-(t - t_k)^2 / 2 sigma^2), the max so pulses never pile up).
//
// Live (heavylight, 2026-11-02: "option 5"): a hit is known `lead` before it is heard (the input side: Capicola's head
// plays a Fade behind the live edge, so lead = the Fade; the output side hears its hits as they come out: lead 0).
// Each pulse peaks on the hit as heard, at detection + lead. It falls with the width; it rises with
// min(width, lead / 3), the same Gaussian squeezed on the left only when the width is wider than the lead allows,
// and lifted so it starts at exactly 0 when the hit is detected. Nothing is ever cut: every hit has its own pulse
// (kSlots of them; a full pool gives the quietest pulse's slot to the new hit) and they join by max (of magnitude:
// a folded strength can be negative, and its pulse then dips).
//
// The width follows the knob live, gliding in log (kWidthGlide) so every pulse in flight reshapes smoothly. A pulse retires
// kRetire widths after its peak, and stays retired if the width later grows: it would come back out of nothing.
// Times in seconds, values -1..1.
class Pulse {
  public:
    static constexpr int kSlots = 16;

    void Init() {
        widthT = width = 0.05f;
        value = 0.0f;
        for (Slot& p : slot) p.on = false;
    }

    void SetWidth(float seconds) { widthT = seconds < 0.001f ? 0.001f : seconds; }

    // `lead`: how long before its peak the hit is known (its rise fits in that), the peak at tDetect + lead.
    void Hit(float strength, double tDetect, float lead) {
        const float s = strength < -1.0f ? -1.0f : strength > 1.0f ? 1.0f : strength;   // (bipolar: a folded strength)
        Slot* use = nullptr;
        float lowest = 2.0f;
        for (Slot& p : slot) {
            if (!p.on) { use = &p; break; }
            const float a = p.v < 0.0f ? -p.v : p.v;
            if (a < lowest) { lowest = a; use = &p; }
        }
        use->on = true;
        use->s = s;
        use->lead = lead < 0.0f ? 0.0f : lead;
        use->c = tDetect + (double)use->lead;
        use->v = 0.0f;
    }

    // The pulses at time t; dt the time since the last Tick (the width's glide).
    float Tick(double t, float dt) {
        // (in log: a constant ratio a step, so a far target doesn't double the width in one block)
        width *= std::exp(std::log(widthT / width) * (1.0f - std::exp(-dt / kWidthGlide)));
        float m = 0.0f;
        for (Slot& p : slot) {
            if (!p.on) continue;
            const float x = (float)(t - p.c);
            float v;
            if (x >= 0.0f) {
                if (x > kRetire * width) { p.on = false; continue; }
                const float u = x / width;
                v = p.s * std::exp(-0.5f * u * u);
            } else if (p.lead <= 0.0f) {
                v = p.s;   // (no lead: it is at its peak from its hit)
            } else {
                // The rise: the Gaussian at min(width, lead / 3), lifted to start at 0 at detection (x = -lead).
                const float sr = (width < p.lead * (1.0f / 3.0f)) ? width : p.lead * (1.0f / 3.0f);
                const float u = x / sr, ub = p.lead / sr;
                const float g0 = std::exp(-0.5f * ub * ub);
                const float g = (std::exp(-0.5f * u * u) - g0) / (1.0f - g0);
                v = p.s * (g > 0.0f ? g : 0.0f);
            }
            p.v = v;
            if ((v < 0.0f ? -v : v) > (m < 0.0f ? -m : m)) m = v;   // (the largest either way: folded strengths go negative)
        }
        return value = m;
    }

    float Value() const { return value; }
    float Width() const { return width; }

    static constexpr float kWidthGlide = 0.03f;   // seconds
    static constexpr float kRetire     = 4.0f;    // widths after the peak (exp(-8): 3e-4 of its height)

  private:
    struct Slot {
        double c;      // the peak: detection + lead
        float  s, lead, v;
        bool   on;
    };
    Slot  slot[kSlots];
    float widthT, width, value;
};

} // namespace capicola
