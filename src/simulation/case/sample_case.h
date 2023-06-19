#ifndef S2E_AOBC_SIMULATION_CASE_SAMPLE_CASE_HPP_
#define S2E_AOBC_SIMULATION_CASE_SAMPLE_CASE_HPP_

#include <simulation/case/simulation_case.hpp>
#include <simulation/monte_carlo_simulation/monte_carlo_simulation_executor.hpp>

#include "../spacecraft/sample_satellite.h"

class SampleCase : public SimulationCase {
 public:
  SampleCase(const std::string initialize_base_file, MonteCarloSimulationExecutor &monte_carlo_simulator, const std::string log_path);
  virtual ~SampleCase();

  // Log for Monte Carlo Simulation
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

 private:
  SampleSatellite *spacecraft_;
  MonteCarloSimulationExecutor &monte_carlo_simulator_;

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
