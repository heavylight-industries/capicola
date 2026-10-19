#include <cmath>
#include <cstdio>
#include <limits>
#include <initializer_list>
#include "jack_routing.h"
#include "control_values.h"

static int failures = 0;
static int checks = 0;
#define CHECK(condition) do { ++checks; if (!(condition)) { ++failures; \
    std::printf("FAIL line %d: %s\n", __LINE__, #condition); } } while (0)
#define NEAR(a, b) CHECK(std::fabs((a) - (b)) < 1.e-6f)
using namespace jacks;

static void storage()
{
    const Options defaults;
    const uint8_t expected[] = {6, 0, 1, 5, 9, 7};
    for (uint8_t j = 0; j < kNumJacks; ++j)
    {
        CHECK(defaults.function[j] == expected[j]);
        for (int value = 0; value < 256; ++value)
        {
            uint8_t bytes[kBytes];
            Encode(defaults, bytes);
            bytes[j] = value;
            const Options restored = Decode(bytes);
            CHECK(restored.function[j] == (value < 11 ? value : expected[j]));
            for (uint8_t other = 0; other < kNumJacks; ++other)
                if (other != j) CHECK(restored.function[other] == expected[other]);
            uint8_t encoded[kBytes];
            Encode(restored, encoded);
            CHECK(Decode(encoded) == restored);
        }
    }
}

static void output_signals()
{
    for (uint8_t jack = 0; jack < kNumJacks; ++jack)
    for (uint8_t f = 0; f < kNumFunctions; ++f)
    for (int input_trigger = 0; input_trigger < 2; ++input_trigger)
    for (int output_trigger = 0; output_trigger < 2; ++output_trigger)
    for (float envelope : {-0.2f, 0.0f, 0.3f, 1.0f, 1.4f})
    {
        Options o;
        o.function[jack] = f;
        Signals s;
        s.inTrig = input_trigger;
        s.outTrig = output_trigger;
        s.inEnv = envelope;
        s.outEnv = 0.8f;
        const auto plan = Route(o, s);
        const float expected = f == 7 ? 5.0f * (envelope < 0 ? 0 : envelope > 1 ? 1 : envelope)
            : f == 8 ? 4.0f : f == 9 ? 5.0f * input_trigger
            : f == 10 ? 5.0f * output_trigger : 0.0f;
        CHECK(plan.output[jack] == (f == 7 || f == 8 || f == 9 || f == 10));
        NEAR(plan.volts[jack], expected);
        for (uint8_t other = 0; other < kNumJacks; ++other)
        {
            if (other == jack) continue;
            const auto defaults = Route(Options{}, s);
            CHECK(plan.output[other] == defaults.output[other]);
            NEAR(plan.volts[other], defaults.volts[other]);
        }
    }
    Signals invalid;
    invalid.inEnv = std::numeric_limits<float>::quiet_NaN();
    Options o;
    o.function[0] = kInputEnvelope;
    NEAR(Route(o, invalid).volts[0], 0.0f);
}

static void modulation()
{
    const float depth[kNumDestinations] = {0.5f, -0.5f, 1, -1, 0, 0.75f};
    const uint8_t source[kNumDestinations] = {0, 1, 0, 1, 0, 1};
    Signals s;
    s.inEnv = 0.4f;
    s.outEnv = 0.8f;
    for (uint8_t j = 0; j < kNumJacks; ++j)
    for (uint8_t f = 0; f < kNumFunctions; ++f)
    for (float sample : {-5.0f, -2.5f, 0.0f, 2.5f, 5.0f})
    {
        Options o;
        for (auto& role : o.function) role = kSlice;
        o.function[j] = f;
        float cv[kNumJacks] = {};
        cv[j] = sample;
        float delta[kNumDestinations];
        Modulation(o, cv, depth, source, s, delta);
        for (uint8_t p = 0; p < kNumDestinations; ++p)
        {
            const float internal = depth[p] * (source[p] == 1 ? 0.8f : 0.4f);
            NEAR(delta[p], internal + (f == p ? sample / 5.0f : 0));
        }
    }
    // Multiple CVs cancel before the internal signal and final knob clamp.
    Options o;
    for (auto& role : o.function) role = kPitch;
    float cv[] = {5,5,-5,0,0,0};
    float delta[kNumDestinations];
    float depths[kNumDestinations] = {-1};
    uint8_t sources[kNumDestinations] = {};
    s.inEnv = 0.8f;
    Modulation(o, cv, depths, sources, s, delta);
    NEAR(delta[0], 0.2f);
    cv[2] = 0.0f;
    Modulation(o, cv, depths, sources, s, delta);
    NEAR(delta[0], 1.2f); // No premature clamp of the external sum.
    cv[0] = std::numeric_limits<float>::quiet_NaN();
    cv[1] = 0.0f;
    Modulation(o, cv, depths, sources, s, delta);
    NEAR(delta[0], -0.8f);
}

static void slice_inputs()
{
    for (uint8_t j = 0; j < kNumJacks; ++j)
    {
        Options o;
        for (auto& f : o.function) f = kPitch;
        o.function[j] = kSlice;
        float cv[kNumJacks] = {};
        bool ready[] = {true,true,true,true,true,true};
        SliceInputs edges;
        CHECK(!edges.Poll(o, cv, ready));
        cv[j] = 2.5f;
        CHECK(edges.Poll(o, cv, ready));
        CHECK(!edges.Poll(o, cv, ready));
        cv[j] = 1.5f; // Between +1 and +2 V, so it cannot re-arm.
        CHECK(!edges.Poll(o, cv, ready));
        cv[j] = 2.5f;
        CHECK(!edges.Poll(o, cv, ready));
        cv[j] = 0.5f;
        CHECK(!edges.Poll(o, cv, ready));
        cv[j] = 2.5f;
        CHECK(edges.Poll(o, cv, ready));
        // Role changes and unavailable inputs do not synthesize rising edges.
        o.function[j] = kSliceTrigger;
        CHECK(!edges.Poll(o, cv, ready));
        o.function[j] = kSlice;
        CHECK(!edges.Poll(o, cv, ready));
        ready[j] = false;
        cv[j] = 0.0f;
        CHECK(!edges.Poll(o, cv, ready));
        ready[j] = true;
        cv[j] = 2.5f;
        CHECK(!edges.Poll(o, cv, ready));
        cv[j] = 0.5f;
        CHECK(!edges.Poll(o, cv, ready));
        cv[j] = 2.5f;
        CHECK(edges.Poll(o, cv, ready));
    }
}

struct FakeJack
{
    bool connected = false, stage_ok = true, direction_ok = true;
    float staged = -1, latched = -1;
    float input_volts = 0.0f;
    float Volts() const { return input_volts; }
    float Value() const { return 0.57f; } // Raw idle bias differs from calibrated zero.
    int stages = 0, enables = 0;
    bool IsCvOutput() const { return connected; }
    bool DisableCvOutput() { if (direction_ok) connected = false; return direction_ok; }
    bool StageVolts(float v) { ++stages; if (stage_ok) staged = v; return stage_ok; }
    bool EnableCvOutput()
    {
        CHECK(latched == staged); // The DAC must be latched before closing the switch.
        ++enables;
        if (direction_ok) connected = true;
        return direction_ok;
    }
};
struct FakeBoard
{
    FakeJack cv_jacks[kNumJacks];
    bool flush_ok = true;
    int flushes = 0;
    bool FlushCvOutputs()
    {
        ++flushes;
        if (flush_ok) for (auto& jack : cv_jacks) jack.latched = jack.staged;
        return flush_ok;
    }
};
static void hardware_switching()
{
    Options o;
    for (auto& f : o.function) f = kInputEnvelope;
    Signals s;
    s.inEnv = 0.6f;
    OutputDriver driver;
    FakeBoard hw;
    driver.Apply(hw, Route(o, s));
    CHECK(hw.flushes == 1); // A single shared-DAC latch for all six writes.
    for (uint8_t j = 0; j < kNumJacks; ++j)
    {
        CHECK(hw.cv_jacks[j].connected);
        CHECK(!driver.InputReady(j));
        NEAR(hw.cv_jacks[j].latched, 3.0f);
    }
    driver.Apply(hw, Route(o, s));
    for (auto& jack : hw.cv_jacks) CHECK(jack.stages == 1);
    for (auto& f : o.function) f = kFeedback;
    driver.Apply(hw, Route(o, s));
    for (uint8_t j = 0; j < kNumJacks; ++j)
    {
        CHECK(!hw.cv_jacks[j].connected);
        CHECK(!driver.InputReady(j)); // One poll for a newly disconnected input to settle.
    }
    driver.Apply(hw, Route(o, s));
    for (uint8_t j = 0; j < kNumJacks; ++j) CHECK(driver.InputReady(j));
    o.function[0] = kSliceTrigger;
    s.inTrig = true;
    hw.flush_ok = false;
    driver.Apply(hw, Route(o, s));
    CHECK(!hw.cv_jacks[0].connected);
    hw.flush_ok = true;
    hw.cv_jacks[0].stage_ok = false;
    driver.Apply(hw, Route(o, s));
    CHECK(!hw.cv_jacks[0].connected);
    hw.cv_jacks[0].stage_ok = true;
    hw.cv_jacks[0].direction_ok = false;
    driver.Apply(hw, Route(o, s));
    CHECK(!hw.cv_jacks[0].connected);
    hw.cv_jacks[0].direction_ok = true;
    driver.Apply(hw, Route(o, s));
    CHECK(hw.cv_jacks[0].connected);
    NEAR(hw.cv_jacks[0].latched, 5.0f);
    hw.cv_jacks[0].direction_ok = false;
    o.function[0] = kPitch;
    driver.Apply(hw, Route(o, s));
    CHECK(hw.cv_jacks[0].connected);
    CHECK(!driver.InputReady(0));
    hw.cv_jacks[0].direction_ok = true;
    driver.Apply(hw, Route(o, s));
    CHECK(!hw.cv_jacks[0].connected);
    CHECK(!driver.InputReady(0));
    driver.Apply(hw, Route(o, s));
    CHECK(driver.InputReady(0));
}

static void failed_write_then_return_to_previous_value()
{
    // MCP DAC shadows survive a failed latch. Returning to the old voltage
    // must replace that pending shadow, rather than flush the obsolete pulse.
    for (uint8_t j = 0; j < 4; ++j)
    {
        Options o;
        for (auto& f : o.function) f = kPitch;
        o.function[j] = kSliceTrigger;
        Signals s;
        OutputDriver driver;
        FakeBoard hw;
        driver.Apply(hw, Route(o, s));
        NEAR(hw.cv_jacks[j].latched, 0.0f);
        hw.flush_ok = false;
        s.inTrig = true;
        driver.Apply(hw, Route(o, s));
        hw.flush_ok = true;
        s.inTrig = false;
        driver.Apply(hw, Route(o, s));
        NEAR(hw.cv_jacks[j].latched, 0.0f);
        driver.Apply(hw, Route(o, s));
        NEAR(hw.cv_jacks[j].latched, 0.0f);
    }
}

static void calibrated_inputs_and_live_mix()
{
    Options o;
    for (auto& f : o.function) f = kPitch;
    FakeBoard hw;
    OutputDriver driver;
    driver.Apply(hw, Route(o, {}));
    float volts[kNumJacks];
    bool ready[kNumJacks];
    driver.ReadInputs(hw, volts, ready);
    for (uint8_t j = 0; j < kNumJacks; ++j)
    {
        CHECK(ready[j]);
        NEAR(volts[j], 0.0f); // Do not turn the raw ADC bias into modulation.
    }
    hw.cv_jacks[0].input_volts = 2.5f;
    driver.ReadInputs(hw, volts, ready);
    Signals signals;
    signals.inEnv = 0.4f;
    signals.outEnv = 0.8f;
    ModulationMixer mixer;
    mixer.Update(o, volts, signals);
    NEAR(mixer.Delta(0, 0.0f, 0), 0.5f);
    // Source and depth changes apply without waiting for another sample frame.
    NEAR(mixer.Delta(0, -1.0f, 1), -0.3f);
    NEAR(mixer.Delta(0, 0.5f, 0), 0.7f);
    // A physically connected output cannot read back into the CV sum.
    hw.cv_jacks[0].connected = true;
    driver.ReadInputs(hw, volts, ready);
    CHECK(!ready[0]);
    NEAR(volts[0], 0.0f);
    hw.cv_jacks[0].connected = false;
    hw.cv_jacks[0].input_volts = 10.0f;
    hw.cv_jacks[1].input_volts = -5.0f;
    driver.ReadInputs(hw, volts, ready);
    mixer.Update(o, volts, signals);
    NEAR(mixer.Delta(0, 0.0f, 0), 1.0f); // Cancel before the final parameter clamp.
    signals.inEnv = std::numeric_limits<float>::quiet_NaN();
    mixer.Update(o, volts, signals);
    NEAR(mixer.Delta(0, 1.0f, 0), 1.0f);
    NEAR(mixer.Delta(0, std::numeric_limits<float>::quiet_NaN(), 1), 1.0f);

    SliceInputs slices;
    o.function[0] = kSlice;
    volts[0] = 0.0f;
    CHECK(!slices.Poll(o, volts, ready));
    volts[0] = 0.8f;
    CHECK(!slices.Poll(o, volts, ready));
    volts[0] = 1.9f;
    CHECK(!slices.Poll(o, volts, ready));
    volts[0] = 2.1f;
    CHECK(slices.Poll(o, volts, ready));
}

static void gesture_base_values()
{
    // A flipped target of 0.4 with +0.1 CV must store 0.3, not 0.4.
    NEAR(controls::BaseForEffective(0.4f, 0.1f), 0.3f);
    NEAR(controls::BaseForEffective(0.4f, 0.1f) + 0.1f, 0.4f);
    NEAR(controls::BaseForEffective(0.7f, -0.15f) - 0.15f, 0.7f);
    NEAR(controls::BaseForEffective(0.1f, 0.4f), 0.0f);
    NEAR(controls::BaseForEffective(0.9f, -0.4f), 1.0f);
}

int main()
{
    storage();
    output_signals();
    modulation();
    slice_inputs();
    hardware_switching();
    failed_write_then_return_to_previous_value();
    calibrated_inputs_and_live_mix();
    gesture_base_values();
    std::printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
