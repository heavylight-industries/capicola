#pragma once

#include <cstdint>

namespace capicola {

template<class Store>
bool SaveWithHistory(Store& presets, uint8_t* scratch, uint16_t capacity)
{
    if (presets.InfoFor(0).valid)
    {
        uint8_t dst = 0;
        uint32_t oldest = UINT32_MAX;
        for (uint8_t i = 1; i < Store::kNumSlots; ++i)
        {
            const auto info = presets.InfoFor(i);
            if (!info.valid) { dst = i; break; }
            if (!dst || info.seq < oldest) { oldest = info.seq; dst = i; }
        }
        if (!dst) return false;

        uint32_t schema = 0;
        uint16_t total = 0;
        const int32_t copied = presets.ReadSlotBytes(0, 0, scratch, capacity, &schema, &total);
        if (copied <= 0 || copied != total || total > capacity) return false;
        if (!presets.WriteSlotRaw(dst, schema, scratch, total)) return false;
    }
    return presets.Save(0);
}

} // namespace capicola
