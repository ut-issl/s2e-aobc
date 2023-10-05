/**
 * @file sample_satellite.hpp
 * @brief An example of user side spacecraft class
 */

#ifndef S2E_AOBC_SIMULATION_SPACECRAFT_SAMPLE_SATELLITE_HPP_
#define S2E_AOBC_SIMULATION_SPACECRAFT_SAMPLE_SATELLITE_HPP_

#include <simulation/spacecraft/spacecraft.hpp>

#include "aocs_module_components.h"

/**
 * @class SampleSatellite
 * @brief An example of user side spacecraft class
 */
class SampleSatellite : public Spacecraft {
 public:
  /**
   * @fn SampleSatellite
   * @brief Constructor
   * @param [in] simulation_configuration: Simulation configuration
   * @param [in] global_environment: Global environment information
   * @param [in] spacecraft_id: Spacecraft ID number
   */
  SampleSatellite(const SimulationConfiguration *simulation_configuration, const GlobalEnvironment *global_environment,
                  const unsigned int spacecraft_id);
};

#endif  // S2E_AOBC_SIMULATION_SPACECRAFT_SAMPLE_SATELLITE_HPP_
