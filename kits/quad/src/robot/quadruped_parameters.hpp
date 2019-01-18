#pragma once

#include <string>

namespace hebi {

struct QuadrupedParameters
{
    double leg_swing_time_;

    void resetToDefaults();
};

} // namespace hebi