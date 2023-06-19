#ifndef S2E_AOBC_SIMULATION_SPACECRAFT_SAMPLE_SATELLITE_HPP_
#define S2E_AOBC_SIMULATION_SPACECRAFT_SAMPLE_SATELLITE_HPP_

#include <simulation/spacecraft/spacecraft.hpp>

#include "aocs_module_components.h"

class SampleSatellite : public Spacecraft {
 public:
  SampleSatellite(const SimulationConfiguration *simulation_configuration, const GlobalEnvironment *global_environment,
                  const unsigned int spacecraft_id);
};

#endif  // S2E_AOBC_SIMULATION_SPACECRAFT_SAMPLE_SATELLITE_HPP_
