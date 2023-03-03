#pragma once

#include "SimulationCase.h"
#include "MCSimExecutor.h"
#include "../Spacecraft/ISSL6Usat.h"

class ISSL6UCase : public SimulationCase
{
public:
  ISSL6UCase(std::string ini_fname, MCSimExecutor& mc_sim, const std::string log_path);
  virtual ~ISSL6UCase();
  void Initialize();
  void Main();

  //Log for Monte Carlo Simulation
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;
private:
  ISSL6USat* spacecraft_;
  MCSimExecutor& mc_sim_;
};
