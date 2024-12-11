/**
 * @file sample_case.hpp
 * @brief Example of user defined simulation case
 */

#ifndef S2E_AOBC_SIMULATION_CASE_SAMPLE_CASE_HPP_
#define S2E_AOBC_SIMULATION_CASE_SAMPLE_CASE_HPP_

#include <simulation/case/simulation_case.hpp>
#include <simulation/monte_carlo_simulation/monte_carlo_simulation_executor.hpp>

#include "../spacecraft/sample_satellite.hpp"

/**
 * @class SampleCase
 * @brief An example of user defined simulation class
 */
class SampleCase : public s2e::simulation::SimulationCase {
 public:
  /**
   * @fn SampleCase
   * @brief Constructor
   * @param [in] initialize_base_file: Initialize base file path
   * @param [in] monte_carlo_simulator: Monte Carlo simulator
   * @param [in] log_path: Log file path
   */
  SampleCase(const std::string initialize_base_file, s2e::simulation::MonteCarloSimulationExecutor &monte_carlo_simulator,
             const std::string log_path);

  /**
   * @fn ~SampleCase
   * @brief Destructor
   */
  virtual ~SampleCase();

  /**
   * @fn GetLogHeader
   * @brief Override function of GetLogHeader
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override function of GetLogValue
   */
  virtual std::string GetLogValue() const;

 private:
  SampleSatellite *spacecraft_;                                           //!< Instance of spacecraft
  s2e::simulation::MonteCarloSimulationExecutor &monte_carlo_simulator_;  //!< Instance of ground station

  /**
   * @fn InitializeTargetObjects
   * @brief Override function of InitializeTargetObjects in SimulationCase
   */
  void InitializeTargetObjects();

  /**
   * @fn UpdateTargetObjects
   * @brief Override function of Main in SimulationCase
   */
  void UpdateTargetObjects();
};

#endif  // S2E_AOBC_SIMULATION_CASE_SAMPLE_CASE_HPP_
