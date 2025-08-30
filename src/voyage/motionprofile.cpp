#include "voyage/motionprofile.hpp"
#include "units.h"

namespace voyage {

template class TrapezoidalMotionProfile<units::length::meters>;
template class TrapezoidalMotionProfile<units::angle::radians>;

typedef TrapezoidalMotionProfile<units::length::meters>
    TrapezoidalMotionProfile_m;

typedef TrapezoidalMotionProfile<units::angle::radians>
    TrapezoidalMotionProfile_rad;

} // namespace voyage