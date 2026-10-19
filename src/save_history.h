#pragma once

#include <cstddef>
#include <cstdint>

namespace history {

/** Preserve the complete boot record before replacing it. Store is Presets
 * in firmware and a fault-injecting store in the host tests. */
template <typename Store>
bool Save(Store& store, uint8_t* buffer, size_t capacity)
{
    static_assert(Store::kNumSlots > 1, "History needs a non-boot slot");
    const auto boot = store.InfoFor(0);
    if (boot.valid)
    {
        if (boot.length == 0 || boot.length > capacity) return false;

        uint8_t destination = 1;
        uint32_t oldest = UINT32_MAX;
        for (uint8_t i = 1; i < Store::kNumSlots; ++i)
        {
            const auto slot = store.InfoFor(i);
            if (!slot.valid) { destination = i; break; }
            if (slot.seq < oldest) { oldest = slot.seq; destination = i; }
        }

        uint32_t schema = 0;
        uint16_t total = 0;
        const int32_t read = store.ReadSlotBytes(
            0, 0, buffer, boot.length, &schema, &total);
        if (read != boot.length || total != boot.length || schema != boot.schema_hash)
            return false;
        if (!store.WriteSlotRaw(destination, schema, buffer, total)) return false;
    }
    return store.Save(0);
}

} // namespace history
