#include "sample_satellite.h"

#include "aocs_module_components.h"

SampleSatellite::SampleSatellite(const SimulationConfiguration *simulation_configuration, const GlobalEnvironment *global_environment,
                     const unsigned int spacecraft_id)
    : Spacecraft(simulation_configuration, global_environment, spacecraft_id) {
  components_ =
      new AocsModuleComponents(dynamics_, structure_, local_environment_, global_environment, simulation_configuration, &clock_generator_, spacecraft_id);
}
