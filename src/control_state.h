#pragma once

#include "alchemy/control/pot_catch.h"
#include "hardware.h"

namespace capicola {

class TriggerInput
{
  public:
    bool Update(float volts)
    {
        if (armed && volts > 2.0f)
        {
            armed = false;
            return true;
        }
        if (!armed && volts < 1.0f) armed = true;
        return false;
    }

  private:
    bool armed = true;
};

inline void RearmSettings(alchemy::PotState* secondary, const bool* secEnabled,
                          alchemy::PotState* feedback, const bool* fbEnabled,
                          const float* phys, bool secondaryOpen, bool feedbackOpen)
{
    // Feedback P2 and the serialized Engine P6 represent the same control.
    feedback[1].stored = secondary[5].stored;
    for (uint8_t p = 0; p < NUM_POTS; ++p)
    {
        if (secondaryOpen && secEnabled[p]) alchemy::InitCatch(secondary[p], phys[p]);
        if (feedbackOpen && fbEnabled[p]) alchemy::InitCatch(feedback[p], phys[p]);
    }
    if (feedbackOpen) secondary[5].stored = feedback[1].stored;
}

} // namespace capicola
