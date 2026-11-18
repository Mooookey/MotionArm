#pragma once

#include "ArmCalibration.h"

// Uncommissioned units have no allowable pulses. Use generate_profile.py with
// actual bench measurements when a mechanical and electrical review is complete.
inline constexpr arm::Calibration kArmProfile{};
