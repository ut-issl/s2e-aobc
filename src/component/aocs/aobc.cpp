/**
 * @file aobc.cpp
 * @brief Class to emulate AOBC
 */

#include "aobc.hpp"

#include <utilities/macros.hpp>

#include "../../simulation/spacecraft/aocs_module_components.hpp"

Aobc::Aobc(ClockGenerator *clock_generator, AocsModuleComponents *components) : Component(100, clock_generator), components_(components) {
  Initialize();
}

Aobc::~Aobc() { delete components_; }

void Aobc::Initialize() {}

void Aobc::MainRoutine(const int time_count) {
  UNUSED(time_count);
  // Currently, this class is not used.
}
