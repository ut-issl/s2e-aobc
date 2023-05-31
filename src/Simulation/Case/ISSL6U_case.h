#pragma once

#include <simulation/case/simulation_case.hpp>
#include <simulation/monte_carlo_simulation/monte_carlo_simulation_executor.hpp>

#include "../Spacecraft/ISSL6Usat.h"

class ISSL6UCase : public SimulationCase {
public:
  ISSL6UCase(const std::string initialize_base_file,
             MonteCarloSimulationExecutor &monte_carlo_simulator,
             const std::string log_path);
  virtual ~ISSL6UCase();

  // Log for Monte Carlo Simulation
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

private:
  ISSL6USat *spacecraft_;
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
