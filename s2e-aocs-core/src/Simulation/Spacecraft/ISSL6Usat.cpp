#include "ISSL6Usat.h"
#include "ISSL6U_Components.h"
#include "ClockGenerator.h"

ISSL6USat::ISSL6USat(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id)
  :Spacecraft(sim_config, glo_env, sat_id)
{
  components_ = new ISSL6UComponents(dynamics_, structure_, local_env_, glo_env, sim_config, &clock_gen_, sat_id);
}
