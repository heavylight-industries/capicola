#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <vector>

#include "save_history.h"

static int checks = 0, failures = 0;
#define CHECK(condition) do { ++checks; if (!(condition)) { ++failures; \
    std::printf("FAIL line %d: %s\n", __LINE__, #condition); } } while (0)

struct Record
{
    bool valid = false;
    uint32_t seq = 0, schema = 0;
    std::vector<uint8_t> bytes;
};

struct FakeStore
{
    static constexpr uint8_t kNumSlots = 4;
    enum Fault { None, ReadFailure, ShortRead, WrongTotal, WrongSchema, HistoryFailure, SaveFailure };
    struct SlotInfo { bool valid; uint32_t seq; uint16_t length; uint32_t schema_hash; };
    std::array<Record, kNumSlots> slots;
    Record live{true, 100, 0xaabbccdd, {8, 6, 4, 2, 0}};
    Fault fault = None;
    std::vector<int> operations;

    SlotInfo InfoFor(uint8_t slot) const
    {
        const auto& r = slots.at(slot);
        return {r.valid, r.seq, static_cast<uint16_t>(r.bytes.size()), r.schema};
    }
    int32_t ReadSlotBytes(uint8_t slot, uint32_t offset, uint8_t* out,
                          uint16_t capacity, uint32_t* schema, uint16_t* total)
    {
        CHECK(slot == 0 && offset == 0);
        operations.push_back(10);
        if (fault == ReadFailure) return -1;
        const auto& r = slots.at(slot);
        *schema = fault == WrongSchema ? r.schema ^ 1u : r.schema;
        *total = r.bytes.size() + (fault == WrongTotal ? 1 : 0);
        size_t size = std::min<size_t>(r.bytes.size(), capacity);
        if (fault == ShortRead && size) --size;
        std::memcpy(out, r.bytes.data(), size);
        return size;
    }
    bool WriteSlotRaw(uint8_t slot, uint32_t schema, const uint8_t* bytes, uint16_t length)
    {
        CHECK(slot > 0 && slot < kNumSlots);
        operations.push_back(100 + slot);
        if (fault == HistoryFailure) return false;
        slots.at(slot) = {true, 99, schema, {bytes, bytes + length}};
        return true;
    }
    bool Save(uint8_t slot)
    {
        CHECK(slot == 0);
        operations.push_back(200 + slot);
        if (fault == SaveFailure) return false;
        slots.at(slot) = live;
        return true;
    }
};

static FakeStore seeded()
{
    FakeStore store;
    // Preserve older/foreign schemas byte-for-byte, including their names.
    store.slots[0] = {true, 42, 0x12345678, {1, 3, 5, 7, 9}};
    return store;
}

static bool sameRecord(const Record& a, const Record& b)
{
    return a.valid == b.valid && a.seq == b.seq && a.schema == b.schema && a.bytes == b.bytes;
}

static void success_paths()
{
    uint8_t buffer[32];
    FakeStore empty;
    CHECK(history::Save(empty, buffer, sizeof buffer));
    CHECK(empty.operations == std::vector<int>({200}));
    CHECK(sameRecord(empty.slots[0], empty.live));

    auto store = seeded();
    const auto boot = store.slots[0];
    CHECK(history::Save(store, buffer, sizeof buffer));
    CHECK(store.operations == std::vector<int>({10, 101, 200}));
    CHECK(store.slots[1].schema == boot.schema && store.slots[1].bytes == boot.bytes);
    CHECK(sameRecord(store.slots[0], store.live));

    store = seeded();
    store.slots[1] = {true, 30, 3, {3}};
    CHECK(history::Save(store, buffer, sizeof buffer));
    CHECK(store.operations == std::vector<int>({10, 102, 200}));

    store = seeded();
    store.slots[1] = {true, 30, 3, {3}};
    store.slots[2] = {true, 10, 2, {2}};
    store.slots[3] = {true, 20, 1, {1}};
    CHECK(history::Save(store, buffer, sizeof buffer));
    CHECK(store.operations == std::vector<int>({10, 102, 200}));

    // Even at the maximum sequence value, the backup destination is never 0.
    store = seeded();
    for (uint8_t i = 1; i < FakeStore::kNumSlots; ++i)
        store.slots[i] = {true, UINT32_MAX, 3, {3}};
    CHECK(history::Save(store, buffer, sizeof buffer));
    CHECK(store.operations == std::vector<int>({10, 101, 200}));
}

static void prerequisite_failures()
{
    uint8_t buffer[32];
    for (auto fault : {FakeStore::ReadFailure, FakeStore::ShortRead,
                       FakeStore::WrongTotal, FakeStore::WrongSchema, FakeStore::HistoryFailure})
    {
        auto store = seeded();
        const auto original = store.slots;
        store.fault = fault;
        CHECK(!history::Save(store, buffer, sizeof buffer));
        CHECK(std::find(store.operations.begin(), store.operations.end(), 200)
              == store.operations.end());
        for (uint8_t slot = 0; slot < FakeStore::kNumSlots; ++slot)
            CHECK(sameRecord(store.slots[slot], original[slot]));
    }

    auto store = seeded();
    CHECK(!history::Save(store, buffer, 2));
    CHECK(store.operations.empty());
    store.slots[0].bytes.clear();
    CHECK(!history::Save(store, buffer, sizeof buffer));
    CHECK(store.operations.empty());
}

static void boot_save_failure()
{
    auto store = seeded();
    const auto boot = store.slots[0];
    store.fault = FakeStore::SaveFailure;
    uint8_t buffer[32];
    CHECK(!history::Save(store, buffer, sizeof buffer));
    CHECK(store.operations == std::vector<int>({10, 101, 200}));
    CHECK(sameRecord(store.slots[0], boot));
    CHECK(store.slots[1].schema == boot.schema && store.slots[1].bytes == boot.bytes);
}

int main()
{
    success_paths();
    prerequisite_failures();
    boot_save_failure();
    std::printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
