#include "quadruped_parameters.hpp"
#include "xml_util/xml_helpers.hpp"

#include "xml_util/pugixml.hpp"

namespace hebi {

void QuadrupedParameters::resetToDefaults()
{
    leg_swing_time_ = 0.25f;
}

} // namespace hebi
