#pragma once

#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace capicola {

// One-pole TPT (zero-delay feedback) filter, low/high/all-pass outputs.
class OnePole {
public:
    // Trivially constructible on purpose: instances live in SDRAM globals, so
    // any ctor code would run pre-main, before the SDRAM controller is up.
    // Call Init() after hardware init, then SetCutoff().
    inline void Init() {
        lp = in = s = 0.0f;
        SetCutoff(1.0f);
    }

    // Cutoff 0..1, normalised to 0 Hz .. fs/2.
    inline void SetCutoff(float fc) {
        g = tanf(M_PI * 0.5f * fc);
        gain = g / (1.0f + g);
    }

    float Tick(float input) {
        in = input;
        float v = gain * (in - s);
        lp = v + s;
        s  = lp + v;
        return lp;
    }

    inline float GetLowpass()  const { return lp; }
    inline float GetHighpass() const { return in - lp; }
    inline float GetAllpass()  const { return lp - GetHighpass(); }  // 2*lp - in
    inline float GetGain()     const { return gain; }
    inline float GetCutoffWarped() const { return g; }
    inline float GetState()    const { return s; }

private:
    float lp;
    float in;
    float s;
    float gain;   // resolved feedback gain, g/(1+g)
    float g;      // warped cutoff, tan(pi/2 * fc)
};

// 2-pole TPT state-variable filter, simultaneous low/band/high/all/notch.
class StateVariable {
public:
    // Trivially constructible on purpose (SDRAM-resident; see OnePole::Init).
    inline void Init() {
        s1 = s2 = hp = lp = bp = in = 0.0f;
        SetControls(1.0f, 0.0f);
    }

    // Q = 0.5 + 4*q, UNCAPPED: q may exceed 1 for extreme resonance. The TPT
    // form stays stable at any Q (den = 1 + 2*R*g + g^2 > 0), so no clamp; as Q
    // climbs the poles approach the unit circle and the filter self-oscillates.
    inline void SetControls(float fc, float q) {
        g = tanf(M_PI * 0.5f * fc);
        this->q = 0.5f + 4.0f * q;
        r   = 1.0f / (2.0f * this->q);
        num = 2.0f * r + g;
        den = 1.0f + 2.0f * r * g + g * g;
    }

    inline void Tick(float input) {
        in = input;
        hp = (in - num * s1 - s2) / den;
        bp = g * hp + s1;
        lp = g * bp + s2;
        s1 = g * hp + bp;
        s2 = g * bp + lp;
    }

    inline float GetLowpass()  const { return lp; }
    inline float GetBandpass() const { return bp; }
    inline float GetHighpass() const { return hp; }
    inline float GetAllpass()  const { return in - 4.0f * r * bp; }
    inline float GetNotch()    const { return lp - hp; }
    inline float GetCutoff()   const { return (2.0f / M_PI) * atanf(g); }
    inline float GetCutoffWarped() const { return g; }
    inline float GetResonance()    const { return q; }

private:
    float s1;
    float s2;
    float q;
    float g;
    float r;
    float hp;
    float lp;
    float bp;
    float in;
    float num;
    float den;
};

} // namespace capicola
