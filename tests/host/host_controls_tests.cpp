#include <array>
#include <cstdio>
#include <limits>

#include "host_controls.h"
#include "control_state.h"
#include "preset_history.h"
#include "alchemy/hw/cv_input.h"

using namespace capicola::host_controls;

static int checks = 0;
static int failures = 0;
#define CHECK(condition) do { ++checks; if (!(condition)) { ++failures; \
    std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); } } while (0)

static const uint8_t* Bytes(const float& value)
{
    return reinterpret_cast<const uint8_t*>(&value);
}

static void TestPager()
{
    alchemy::Pager pager(2, NUM_POTS);
    float phys[NUM_POTS] = {};
    float held[NUM_POTS] = {};
    // Prime pickup exactly as the control loop does before runtime edits.
    pager.Update(phys, 0);
    const auto schema = pager.SchemaHash();
    const auto bytes = pager.SerializedSize();
    for (uint8_t page = 0; page < 2; ++page)
        for (uint8_t pot = 0; pot < NUM_POTS; ++pot)
        {
            const float value = 0.2f + 0.05f * (page * NUM_POTS + pot);
            CHECK(SetPager(pager, (page * NUM_POTS + pot) * sizeof(float),
                           Bytes(value), sizeof(value), phys, true, 0, held));
            CHECK(pager.Stored(page, pot) == value);
            CHECK(!pager.State(page, pot).caught);
            if (page == 0) CHECK(held[pot] == value);
        }
    CHECK(pager.SchemaHash() == schema);
    CHECK(pager.SerializedSize() == bytes);

    // A B3 page reasserting its held snapshot must retain the host edit.
    pager.Update(phys, 16);
    for (uint8_t pot = 0; pot < NUM_POTS; ++pot)
    {
        pager.SetStored(0, pot, held[pot], phys);
        CHECK(pager.Stored(0, pot) == held[pot]);
    }
    // Editing Depth while it is held uses Depth's snapshot, too.
    pager.GoToPage(1, phys);
    for (uint8_t pot = 0; pot < NUM_POTS; ++pot) held[pot] = pager.Stored(1, pot);
    const float depth = 0.9f;
    CHECK(SetPager(pager, 8 * sizeof(float), Bytes(depth), sizeof(depth), phys, true, 1, held));
    CHECK(held[2] == depth);
    CHECK(held[1] == pager.Stored(1, 1));
    // Catch stays parked until the physical pot crosses the new target.
    pager.Update(phys, 32);
    CHECK(pager.Stored(1, 2) == depth);
    phys[2] = 1.0f;
    pager.Update(phys, 48);
    CHECK(pager.Caught(2));
    CHECK(pager.Stored(1, 2) == 1.0f);

    std::array<uint8_t, 2 * NUM_POTS * sizeof(float)> before{}, after{};
    pager.Serialize(before.data());
    const float invalid[] = {-0.1f, 1.1f, std::numeric_limits<float>::quiet_NaN(),
                             std::numeric_limits<float>::infinity()};
    for (const float value : invalid)
        CHECK(!SetPager(pager, 0, Bytes(value), sizeof(value), phys, true, 1, held));
    CHECK(!SetPager(pager, 1, Bytes(depth), sizeof(depth), phys, true, 1, held));
    CHECK(!SetPager(pager, 0, Bytes(depth), 1, phys, true, 1, held));
    CHECK(!SetPager(pager, bytes, Bytes(depth), sizeof(depth), phys, true, 1, held));
    pager.Serialize(after.data());
    CHECK(before == after);
}

static void TestRouting()
{
    uint8_t zones[NUM_POTS] = {};
    bool caught[NUM_POTS] = {true, true, true, true, true, true};
    constexpr uint8_t sources = 9;
    for (uint8_t pot = 0; pot < NUM_POTS; ++pot)
    {
        const uint8_t source = pot + 3;
        CHECK(SetRouting(zones, caught, sources, pot, &source, 1));
        CHECK(zones[pot] == source);
        CHECK(!caught[pot]);
        if (pot + 1 < NUM_POTS) CHECK(caught[pot + 1]);
    }
    const uint8_t invalid = sources;
    CHECK(!SetRouting(zones, caught, sources, 0, &invalid, 1));
    CHECK(!SetRouting(zones, caught, sources, NUM_POTS, zones, 1));
    CHECK(!SetRouting(zones, caught, sources, 0, zones, 2));
    CHECK(!SetRouting(zones, caught, sources, 0, zones, 0));
    CHECK(zones[0] == 3);
    const uint8_t source = 0;
    CHECK(SetRouting(zones, caught, sources, 0, &source, 1));
    CHECK(zones[0] == source);
}

static void TestSecondary()
{
    alchemy::PotState states[NUM_POTS];
    alchemy::PotState band;
    const float phys[NUM_POTS] = {0.1f, 0.2f, 0.1f, 0.1f, 0.1f, 0.8f};
    const float value = 0.8f;
    CHECK(SetSecondary(states, band, 0, Bytes(value), sizeof(value), phys));
    CHECK(states[0].stored == value);
    CHECK(!states[0].caught);
    CHECK(states[1].caught);
    CHECK(states[1].stored == 0.5f);
    CHECK(band.stored == 0.5f);
    alchemy::UpdateCatch(states[0], phys[0]);
    CHECK(states[0].stored == value);
    alchemy::UpdateCatch(states[0], 0.9f);
    CHECK(states[0].caught);

    // The band uses feedback P2's position, never the hidden Engine P6 pot.
    CHECK(SetSecondary(states, band, 5 * sizeof(float), Bytes(value), sizeof(value), phys));
    CHECK(states[5].stored == value);
    CHECK(band.stored == value);
    CHECK(!band.caught);
    alchemy::UpdateCatch(band, phys[1]);
    CHECK(band.stored == value);
    alchemy::UpdateCatch(band, 0.9f);
    CHECK(band.caught);

    const float invalid = std::numeric_limits<float>::quiet_NaN();
    CHECK(!SetSecondary(states, band, 0, Bytes(invalid), sizeof(invalid), phys));
    CHECK(!SetSecondary(states, band, 1, Bytes(value), sizeof(value), phys));
    CHECK(!SetSecondary(states, band, 0, Bytes(value), 2, phys));
    CHECK(!SetSecondary(states, band, NUM_POTS * sizeof(float), Bytes(value), sizeof(value), phys));
    CHECK(states[0].stored == 0.9f);
}

static void TestOptions()
{
    uint8_t snap = 0;
    jacks::Options options;
    bool caught[NUM_POTS] = {true, true, true, true, true, true};
    uint8_t value = 1;
    CHECK(SetOptions(snap, options, caught, 0, &value, 1));
    CHECK(snap == 1);
    CHECK(caught[0]);
    for (uint8_t mode = 0; mode <= 3; ++mode)
    {
        CHECK(SetOptions(snap, options, caught, 1, &mode, 1));
        CHECK(options.cvIn == ((mode & 1) != 0));
        CHECK(options.outFollower == ((mode & 2) != 0));
        CHECK(snap == 1);
        for (bool state : caught) CHECK(!state);
    }
    for (uint8_t signal = 0; signal < jacks::kNumSignals; ++signal)
    {
        const uint8_t previous = options.env2;
        CHECK(SetOptions(snap, options, caught, 2, &signal, 1));
        CHECK(options.trig2 == signal);
        CHECK(options.env2 == previous);
        CHECK(SetOptions(snap, options, caught, 3, &signal, 1));
        CHECK(options.env2 == signal);
        CHECK(options.trig2 == signal);
    }
    const uint8_t limits[] = {2, 4, jacks::kNumSignals, jacks::kNumSignals};
    uint8_t before[jacks::kBytes], after[jacks::kBytes];
    jacks::Encode(options, before);
    for (uint16_t field = 0; field < 4; ++field)
        CHECK(!SetOptions(snap, options, caught, field, &limits[field], 1));
    CHECK(!SetOptions(snap, options, caught, 4, &value, 1));
    CHECK(!SetOptions(snap, options, caught, 0, &value, 2));
    CHECK(!SetOptions(snap, options, caught, 0, &value, 0));
    jacks::Encode(options, after);
    CHECK(std::memcmp(before, after, sizeof(before)) == 0);
    CHECK(snap == 1);
}

static void TestTriggerInput()
{
    // Exercise the real SDK voltage conversion at nominal and calibrated spans.
    for (bool calibrated : {false, true})
    {
        const float vdda = calibrated ? 3.20f : 3.30f;
        const uint16_t zero = calibrated ? 33500 : 32768;
        uint16_t raw = zero;
        alchemy::CvInput input;
        input.Init(&raw, 1000.0f, true, false, 0.0f);
        if (calibrated) input.SetCalibration(zero, vdda);
        capicola::TriggerInput trigger;
        auto poll = [&](float volts) {
            raw = static_cast<uint16_t>(zero - volts * 65535.0f * alchemy::kV2CvInGainDesign / vdda + 0.5f);
            input.Process();
            return trigger.Update(input.Volts());
        };
        CHECK(!poll(0.0f));
        CHECK(!poll(1.9f));
        CHECK(poll(2.1f));
        CHECK(!poll(5.0f));
        CHECK(!poll(1.1f));
        CHECK(!poll(3.3f)); // No second edge until rearmed.
        CHECK(!poll(0.9f));
        CHECK(poll(3.3f)); // Previously below the mistaken 4 V threshold.
        CHECK(!poll(3.3f));
        CHECK(!poll(-2.0f));
        CHECK(poll(5.0f));
    }
    capicola::TriggerInput trigger;
    CHECK(!trigger.Update(2.0f));
    CHECK(trigger.Update(2.01f));
    CHECK(!trigger.Update(1.0f));
    CHECK(!trigger.Update(2.01f));
    CHECK(!trigger.Update(0.99f));
    CHECK(trigger.Update(2.01f));
}

static void TestPresetPickup()
{
    alchemy::PotState secondary[NUM_POTS], feedback[NUM_POTS];
    const bool enabled[NUM_POTS] = {true, true, true, true, true, false};
    const float phys[NUM_POTS] = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
    feedback[0].stored = 0.7f; // Unsaved comb size must survive a load.
    feedback[1].stored = 0.2f;
    secondary[5].stored = 0.8f; // A newly loaded Feedback Tone.
    capicola::RearmSettings(secondary, enabled, feedback, enabled, phys, false, true);
    CHECK(feedback[1].stored == 0.8f);
    CHECK(!feedback[1].caught);
    CHECK(secondary[5].stored == 0.8f);
    CHECK(feedback[0].stored == 0.7f);
    // The next frame's actual pickup algorithm must hold the loaded value.
    for (uint8_t p = 0; p < NUM_POTS; ++p)
        if (enabled[p]) alchemy::UpdateCatch(feedback[p], phys[p]);
    CHECK(feedback[1].stored == secondary[5].stored);
    CHECK(feedback[0].stored == 0.7f);
    alchemy::UpdateCatch(feedback[1], 0.9f);
    CHECK(feedback[1].caught);

    // Proximity pickup can snap to the pot; both views must agree afterward.
    secondary[5].stored = 0.202f;
    capicola::RearmSettings(secondary, enabled, feedback, enabled, phys, false, true);
    CHECK(feedback[1].caught);
    CHECK(secondary[5].stored == feedback[1].stored);

    // Loading with Engine settings open only rearms that page's controls.
    feedback[1].caught = false;
    secondary[0].stored = 0.9f;
    secondary[5].stored = 0.6f;
    capicola::RearmSettings(secondary, enabled, feedback, enabled, phys, true, false);
    CHECK(feedback[1].stored == 0.6f);
    CHECK(!secondary[0].caught);
    CHECK(!feedback[1].caught);
    alchemy::UpdateCatch(secondary[0], phys[0]);
    CHECK(secondary[0].stored == 0.9f);
}

struct HistoryStore
{
    static constexpr uint8_t kNumSlots = 16;
    struct Slot
    {
        bool valid = false;
        uint32_t seq = 0, schema = 0;
        std::array<uint8_t, 4> bytes{};
    };
    Slot slots[kNumSlots];
    bool failRead = false, failArchive = false, failSave = false;
    int saves = 0, archives = 0;
    uint8_t archivedSlot = 0;

    Slot InfoFor(uint8_t slot) const { return slots[slot]; }
    int32_t ReadSlotBytes(uint8_t slot, uint32_t, uint8_t* out, uint16_t capacity,
                          uint32_t* schema, uint16_t* total) const
    {
        if (failRead) return -1;
        *schema = slots[slot].schema;
        *total = slots[slot].bytes.size();
        const auto count = capacity < *total ? capacity : *total;
        std::memcpy(out, slots[slot].bytes.data(), count);
        return count;
    }
    bool WriteSlotRaw(uint8_t slot, uint32_t schema, const uint8_t* data, uint16_t size)
    {
        ++archives;
        archivedSlot = slot;
        if (failArchive || size != slots[slot].bytes.size()) return false;
        slots[slot] = {true, 100, schema, {}};
        std::memcpy(slots[slot].bytes.data(), data, size);
        return true;
    }
    bool Save(uint8_t slot)
    {
        ++saves;
        if (failSave) return false;
        slots[slot] = {true, 101, 456, {5, 6, 7, 8}};
        return true;
    }
};

static void TestSaveHistory()
{
    uint8_t scratch[4];
    HistoryStore store;
    CHECK(capicola::SaveWithHistory(store, scratch, sizeof(scratch)));
    CHECK(store.saves == 1 && store.archives == 0);

    auto populated = [] {
        HistoryStore result;
        result.slots[0] = {true, 5, 123, {1, 2, 3, 4}};
        return result;
    };
    store = populated();
    const auto previous = store.slots[0];
    CHECK(capicola::SaveWithHistory(store, scratch, sizeof(scratch)));
    CHECK(store.archivedSlot == 1);
    CHECK(store.slots[1].bytes == previous.bytes);
    CHECK(store.slots[1].schema == previous.schema);
    CHECK(store.slots[0].bytes != previous.bytes);

    // Never overwrite the boot state when reading or archiving it failed.
    store = populated();
    store.failRead = true;
    CHECK(!capicola::SaveWithHistory(store, scratch, sizeof(scratch)));
    CHECK(store.saves == 0 && store.archives == 0);
    CHECK(store.slots[0].bytes == previous.bytes);
    store = populated();
    CHECK(!capicola::SaveWithHistory(store, scratch, 2)); // Truncated read.
    CHECK(store.saves == 0 && store.archives == 0);
    store = populated();
    store.failArchive = true;
    CHECK(!capicola::SaveWithHistory(store, scratch, sizeof(scratch)));
    CHECK(store.saves == 0 && store.archives == 1);
    CHECK(store.slots[0].bytes == previous.bytes);
    store = populated();
    store.failSave = true;
    CHECK(!capicola::SaveWithHistory(store, scratch, sizeof(scratch)));
    CHECK(store.slots[0].bytes == previous.bytes);
    CHECK(store.slots[1].bytes == previous.bytes);

    // A full history replaces its oldest entry, never its boot slot.
    store = populated();
    for (uint8_t slot = 1; slot < store.kNumSlots; ++slot)
        store.slots[slot] = {true, static_cast<uint32_t>(10 + slot), 789, {}};
    store.slots[7].seq = 1;
    CHECK(capicola::SaveWithHistory(store, scratch, sizeof(scratch)));
    CHECK(store.archivedSlot == 7);
    CHECK(store.slots[7].bytes == previous.bytes);
    CHECK(store.slots[6].schema == 789);
}

int main()
{
    TestPager();
    TestRouting();
    TestSecondary();
    TestOptions();
    TestTriggerInput();
    TestPresetPickup();
    TestSaveHistory();
    std::printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
