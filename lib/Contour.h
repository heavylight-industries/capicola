#pragma once

namespace capicola {

// CONT, the accent contour (the dt2-keyframer's LFO wave, DESIGN 7): a smoothstep from one hit's strength to the
// next's, so it never dips between hits; a hit's strength is its own energy, not its rank.
//
// The keyframer knows its next point ahead and arrives at it on the hit. Live, the next hit is not known yet, so
// each hit starts a smoothstep from wherever the contour is to its own strength, lasting the gap since the hit before
// (kMinGap .. kMaxGap; the first, kFirstGap): the same contour, its shape one hit late. Between hits it holds.
// Times in seconds; values -1..1 (strengths are 0..1 until the engine folds them).
class Contour {
  public:
    void Init() {
        from = to = value = 0.0f;
        t0 = 0.0;
        dur = kFirstGap;
        lastHit = -1.0;
    }

    void Hit(float strength, double t) {
        const float s = strength < -1.0f ? -1.0f : strength > 1.0f ? 1.0f : strength;   // (bipolar: a folded strength)
        double gap = (lastHit < 0.0) ? kFirstGap : t - lastHit;
        if (gap < kMinGap) gap = kMinGap;
        if (gap > kMaxGap) gap = kMaxGap;
        from = At(t);
        to = s;
        t0 = t;
        dur = gap;
        lastHit = t;
    }

    // The contour at time t (never before the latest hit's t).
    float At(double t) const {
        double x = (t - t0) / dur;
        if (x <= 0.0) return from;
        if (x >= 1.0) return to;
        const float f = (float)x;
        return from + (to - from) * f * f * (3.0f - 2.0f * f);
    }

    float Tick(double t) { return value = At(t); }
    float Value() const { return value; }

    static constexpr double kMinGap   = 0.023;   // (the detector's hold-off between kept hits)
    static constexpr double kMaxGap   = 2.0;
    static constexpr double kFirstGap = 0.25;

  private:
    float  from, to, value;
    double t0, dur, lastHit;
};

} // namespace capicola
