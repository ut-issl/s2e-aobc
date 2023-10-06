/**
 * @file aobc.cpp
 * @brief Class to emulate AOBC
 */

#include "aobc.hpp"

#include <library/utilities/macros.hpp>

#include "../../simulation/spacecraft/aocs_module_components.hpp"

AOBC::AOBC(ClockGenerator *clock_generator, AocsModuleComponents *components) : Component(100, clock_generator), components_(components) { Initialize(); }

AOBC::~AOBC() { delete components_; }

void AOBC::Initialize() {}

void AOBC::MainRoutine(const int time_count) {
  UNUSED(time_count);
  // Currently, this class is not used.
}
