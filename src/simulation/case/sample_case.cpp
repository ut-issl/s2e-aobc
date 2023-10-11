/**
 * @file sample_case.cpp
 * @brief Example of user defined simulation case
 */

#include "sample_case.hpp"

#include <./simulation/monte_carlo_simulation/simulation_object.hpp>

SampleCase::SampleCase(const std::string initialize_base_file, MonteCarloSimulationExecutor &monte_carlo_simulator, const std::string log_path)
    : SimulationCase(initialize_base_file, monte_carlo_simulator, log_path), monte_carlo_simulator_(monte_carlo_simulator) {}

SampleCase::~SampleCase() { delete spacecraft_; }

void SampleCase::InitializeTargetObjects() {
  // Instantiate the target of the simulation
  // `spacecraft_id` corresponds to the index of `spacecraft_file` in simulation_base.ini
  const int spacecraft_id = 0;
  spacecraft_ = new SampleSatellite(&simulation_configuration_, global_environment_, spacecraft_id);

  // Register the log output
  spacecraft_->LogSetup(*(simulation_configuration_.main_logger_));

  // Monte Carlo Simulation
  monte_carlo_simulator_.SetSeed();
  monte_carlo_simulator_.RandomizeAllParameters();
  SimulationObject::SetAllParameters(monte_carlo_simulator_);
  monte_carlo_simulator_.AtTheBeginningOfEachCase();
}

void SampleCase::UpdateTargetObjects() {
  // Spacecraft Update
  spacecraft_->Update(&(global_environment_->GetSimulationTime()));
}

// Log for Monte Carlo Simulation
std::string SampleCase::GetLogHeader() const {
  std::string str_tmp = "";
  str_tmp += WriteScalar("time", "s");
  str_tmp += WriteVector("spacecraft_angular_velocity", "b", "rad/s", 3);

  return str_tmp;
}

std::string SampleCase::GetLogValue() const {
  std::string str_tmp = "";
  str_tmp += WriteScalar(global_environment_->GetSimulationTime().GetElapsedTime_s());
  str_tmp += WriteVector(spacecraft_->GetDynamics().GetAttitude().GetAngularVelocity_b_rad_s(), 3);

  return str_tmp;
}
