#pragma once

#include <cstdint>
#include <cstring>

#include "alchemy/surface/pager.h"
#include "hardware.h"
#include "jack_routing.h"

namespace capicola::host_controls {

inline bool ReadNorm(uint16_t offset, const uint8_t* data, uint16_t size,
                     unsigned cells, unsigned& cell, float& value)
{
    if (offset % sizeof(float) || size != sizeof(float)) return false;
    cell = offset / sizeof(float);
    if (cell >= cells) return false;
    std::memcpy(&value, data, sizeof(value));
    return value >= 0.0f && value <= 1.0f;
}

inline bool SetPager(alchemy::Pager& pager, uint16_t offset,
                     const uint8_t* data, uint16_t size, const float* phys,
                     bool overlayOpen, uint8_t heldPage, float* heldNorm)
{
    unsigned cell;
    float value;
    if (!ReadNorm(offset, data, size, pager.NumPages() * pager.NumPots(), cell, value))
        return false;
    const uint8_t page = cell / pager.NumPots();
    const uint8_t pot  = cell % pager.NumPots();
    pager.SetStored(page, pot, value, phys);
    // The B3 pages reassert this snapshot every frame.
    if (overlayOpen && page == heldPage) heldNorm[pot] = pager.Stored(page, pot);
    return true;
}

inline bool SetRouting(uint8_t* zones, bool* caught, uint8_t numSources,
                       uint16_t offset, const uint8_t* data, uint16_t size)
{
    if (size != 1 || offset >= NUM_POTS || data[0] >= numSources) return false;
    zones[offset] = data[0];
    caught[offset] = false;
    return true;
}

inline bool SetSecondary(alchemy::PotState* states, alchemy::PotState& band,
                         uint16_t offset, const uint8_t* data, uint16_t size,
                         const float* phys)
{
    unsigned pot;
    float value;
    if (!ReadNorm(offset, data, size, NUM_POTS, pot, value)) return false;
    // Secondary P6 is also the feedback page's P2.
    if (pot == 5)
    {
        band.stored = value;
        alchemy::InitCatch(band, phys[1]);
        states[pot].stored = band.stored;
    }
    else
    {
        states[pot].stored = value;
        alchemy::InitCatch(states[pot], phys[pot]);
    }
    return true;
}

inline bool SetOptions(uint8_t& snap, jacks::Options& options, bool* routeCaught,
                       uint16_t offset, const uint8_t* data, uint16_t size)
{
    if (size != 1 || offset > jacks::kBytes) return false;
    const uint8_t value = data[0];
    switch (offset)
    {
        case 0:
            if (value > 1) return false;
            snap = value;
            break;
        case 1:
            if (value > 3) return false;
            options.cvIn = (value & 1) != 0;
            options.outFollower = (value & 2) != 0;
            // Changing the source wheel also changes its pickup zones.
            for (uint8_t p = 0; p < NUM_POTS; ++p) routeCaught[p] = false;
            break;
        case 2:
        case 3:
            if (value >= jacks::kNumSignals) return false;
            (offset == 2 ? options.trig2 : options.env2) = value;
            break;
    }
    return true;
}

} // namespace capicola::host_controls
