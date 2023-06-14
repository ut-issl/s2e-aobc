#pragma once

#include <simulation/spacecraft/spacecraft.hpp>

#include "ISSL6U_Components.h"

class SampleSatellite : public Spacecraft {
 public:
  SampleSatellite(const SimulationConfiguration *simulation_configuration, const GlobalEnvironment *global_environment, const unsigned int spacecraft_id);
};
