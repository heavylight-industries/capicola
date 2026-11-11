#pragma once

#include <KeyframeRecorder.h>

namespace capicola {

// Keeps the two channels' grids together while their detectors and grains stay
// their own. Run once per block, after both recorders' ProcessBlock. A follow
// takes effect on the next block: the follower's idle grain takes the leader's
// grid and head and fades in (KeyframeRecorder::Follow).
template <int bufsz>
class StereoAlign {
public:
    // Guardrail: when exactly one channel fires and the grids have drifted
    // further apart than this, the quiet channel follows at once.
    static constexpr double kMaxDriftSamples = 48000.0;   // 1 s @ 48 kHz

    // Coincidence window. When one channel punches on a hit and the other
    // doesn't, the other follows too if its own envelope comes within
    // kNearShare of its own keep level inside this window (unless it punched
    // on a hit of its own within it already): one stereo hit cuts both
    // channels at the same time, a hit on one side only stays one-sided.
    static constexpr int   kCoincideSamples = 12000;     // 250 ms @ 48 kHz
    static constexpr float kNearShare       = 0.5f;

    // Drift can't last: grids further apart than kDriftLong for kDriftPatience
    // straight, and the channel that punched on a hit longer ago follows.
    static constexpr double kDriftLong     = 2400.0;     // 50 ms
    static constexpr int    kDriftPatience = 96000;      // 2 s

    void Init() { armL = armR = 0; driftFor = 0; follows = 0; }

    uint32_t Follows() const { return follows; }   // monotonic: realignments made

    void Process(KeyframeRecorder<bufsz>& l, KeyframeRecorder<bufsz>& r, int blk) {
        const bool fired_l = l.FiredThisBlock();
        const bool fired_r = r.FiredThisBlock();
        const double lag_l = l.GridLag();
        const double lag_r = r.GridLag();
        const double drift = (lag_l > lag_r) ? (lag_l - lag_r) : (lag_r - lag_l);

        // One channel fired: past the guardrail the other follows at once, else
        // it is armed for the coincidence window.
        if (fired_l != fired_r) {
            KeyframeRecorder<bufsz>& quiet = fired_l ? r : l;
            KeyframeRecorder<bufsz>& loud  = fired_l ? l : r;
            int& arm = fired_l ? armR : armL;
            arm = 0;
            if (drift > kMaxDriftSamples) { quiet.Follow(loud, true); follows++; }
            else if (quiet.SinceFire() > kCoincideSamples) arm = kCoincideSamples;
        }
        if (fired_l) armL = 0;   // (a channel that fires on its own needs no follow)
        if (fired_r) armR = 0;
        if (armL > 0) {
            if (l.NearHit(kNearShare)) { l.Follow(r, true); armL = 0; follows++; }
            else armL -= blk;
        }
        if (armR > 0) {
            if (r.NearHit(kNearShare)) { r.Follow(l, true); armR = 0; follows++; }
            else armR -= blk;
        }

        // Drift that outlasts kDriftPatience always ends: the staler channel follows.
        driftFor = (drift > kDriftLong) ? driftFor + blk : 0;
        if (driftFor >= kDriftPatience) {
            if (l.SinceFire() > r.SinceFire()) l.Follow(r, false);
            else                               r.Follow(l, false);
            driftFor = 0;
            follows++;
        }
    }

private:
    int      armL, armR;   // samples left in which that channel may still follow
    int      driftFor;     // how long the grids have stayed further apart than kDriftLong
    uint32_t follows;
};

} // namespace capicola
