#pragma once

#include <cmath>

namespace controls {

/** A gesture holds an effective (already modulated) value. Store only its
 * base on release, so the still-connected CV is not added a second time. */
inline float BaseForEffective(float effective, float modulation)
{
    const float base = effective - modulation;
    if (!std::isfinite(base)) return 0.5f;
    return base < 0.0f ? 0.0f : base > 1.0f ? 1.0f : base;
}

} // namespace controls
