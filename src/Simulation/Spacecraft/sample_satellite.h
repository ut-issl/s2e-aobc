#pragma once

#include <simulation/spacecraft/spacecraft.hpp>

#include "aocs_module_components.h"

class SampleSatellite : public Spacecraft {
 public:
  SampleSatellite(const SimulationConfiguration *simulation_configuration, const GlobalEnvironment *global_environment, const unsigned int spacecraft_id);
};
