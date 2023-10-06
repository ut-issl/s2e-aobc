/**
 * @file aobc.cpp
 * @brief Class to emulate AOBC
 */

#include "aobc.hpp"

#include <library/utilities/macros.hpp>

#include "../../simulation/spacecraft/aocs_module_components.hpp"

AOBC::AOBC(ClockGenerator *clock_gen, AocsModuleComponents *components) : Component(100, clock_gen), components_(components) { Initialize(); }

AOBC::~AOBC() { delete components_; }

void AOBC::Initialize() {}

void AOBC::MainRoutine(int count) {
  UNUSED(count);
  // Currently, this class is not used.
}
