#pragma once

#include "Spacecraft.h"
#include "ISSL6U_Components.h"

class ISSL6USat : public Spacecraft
{
public:
  ISSL6USat(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id);
};
