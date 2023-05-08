#include "ISSL6Usat.h"
#include "ISSL6U_Components.h"

ISSL6USat::ISSL6USat(const SimulationConfiguration *simulation_configuration, const GlobalEnvironment *global_environment,
                     const unsigned int spacecraft_id)
    : Spacecraft(simulation_configuration, global_environment, spacecraft_id)
{
  components_ = new ISSL6UComponents(dynamics_, structure_, local_environment_, global_environment, simulation_configuration, &clock_generator_, spacecraft_id);
}
