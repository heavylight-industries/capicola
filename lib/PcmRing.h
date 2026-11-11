#pragma once

#include <cstdint>
#include <cstring>

namespace capicola {

// The live input as PCM, int16, in lockstep with the Analyzer: sample n of the raw timeline (the keyframes' time
// axis) sits at index n, so a head's position reads the PCM exactly where it reads the keyframes. The top of the
// Quality knob plays from here instead of the keyframes (the dt2-keyframer's way: the keyframes are never heard,
// they only steer the heads, the leash and the splices).
//
// Cheap per sample, after the dt2-keyframer's kf.c: no 64-bit integer and no floor on the way. The ring keeps the
// raw time of its slot 0 for the lap being written (lapT0, a double that changes only when the write position
// wraps), so a position becomes a ring place with one subtraction, one wrap and a hardware convert; the range a
// read may use (Holds) is two doubles kept as the writer goes.
//
// The ring is whatever the SDRAM has left (AudioEngine: 3.6 M samples, 75 s a channel); a head further back than
// that falls back to the keyframes. Trivially constructible on purpose (SDRAM-resident): call Init() with the
// buffer.
class PcmRing {
  public:
    void Init(int16_t* buffer, uint32_t length) {
        buf = buffer;
        n = length;
        std::memset(buf, 0, (size_t)n * sizeof(int16_t));
        Reset();
    }

    // Back to an empty ring (the Analyzer restarted its timeline), in O(1): what it held is no longer Held.
    void Reset() {
        pos = 0;
        lapT0 = 0.0;
        written = 0.0;
        lo = 1.0;
    }

    void Write(float x) {
        // Round to int16 by hand: a hardware convert, not lrintf's call.
        const float c = x > 1.0f ? 1.0f : x < -1.0f ? -1.0f : x;
        const float s = c * 32767.0f;
        buf[pos] = (int16_t)(int32_t)(s >= 0.0f ? s + 0.5f : s - 0.5f);
        written += 1.0;
        if (++pos == n) { pos = 0; lapT0 += (double)n; }
        if (written - (double)n + 2.0 > lo) lo = written - (double)n + 2.0;
    }

    // Can t be read: its four samples (floor(t) - 1 .. + 2) all written and none overwritten.
    bool Holds(double t) const { return t >= lo && t <= written - 3.0; }

    // The PCM at time t (raw samples), Catmull-Rom through the four around it. Holds(t) first.
    float Read(double t) const {
        double rel = t - lapT0;              // the place in the ring, this lap ..
        if (rel < 0.0) rel += (double)n;     // .. or the one before
        const int32_t p = (int32_t)rel;      // (rel >= 0: a truncation is the floor)
        const float   f = (float)(rel - (double)p);
        const int32_t pm = (p == 0) ? (int32_t)n - 1 : p - 1;
        const int32_t p1 = (p + 1 == (int32_t)n) ? 0 : p + 1;
        const int32_t p2 = (p1 + 1 == (int32_t)n) ? 0 : p1 + 1;
        const float xm = buf[pm], x0 = buf[p], x1 = buf[p1], x2 = buf[p2];
        const float c1 = 0.5f * (x1 - xm);
        const float c2 = xm - 2.5f * x0 + 2.0f * x1 - 0.5f * x2;
        const float c3 = 0.5f * (x2 - xm) + 1.5f * (x0 - x1);
        return (((c3 * f + c2) * f + c1) * f + x0) * (1.0f / 32767.0f);
    }

  private:
    int16_t* buf;
    uint32_t n;
    uint32_t pos;
    double   lapT0;     // the raw time of slot 0 in the lap being written
    double   written;   // samples written: the next sample's raw time
    double   lo;        // the oldest time Held
};

} // namespace capicola
