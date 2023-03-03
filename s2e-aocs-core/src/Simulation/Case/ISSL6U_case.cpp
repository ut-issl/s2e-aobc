#include "ISSL6U_case.h"
#include "SimulationConfig.h"
#include "SimulationObject.h"
#include "../Spacecraft/ISSL6Usat.h"

ISSL6UCase::ISSL6UCase(std::string ini_fname, MCSimExecutor& mc_sim, const std::string log_path)
  : SimulationCase(ini_fname,mc_sim,log_path),mc_sim_(mc_sim)
{
}

ISSL6UCase::~ISSL6UCase()
{
  delete spacecraft_;
}

void ISSL6UCase::Initialize()
{
  //インスタンス化
  const int sat_id = 0;
  spacecraft_ = new ISSL6USat(&sim_config_, glo_env_, sat_id);

  //Monte Carlo Simulation
  mc_sim_.SetSeed();
  mc_sim_.RandomizeAllParameters();
  SimulationObject::SetAllParameters(mc_sim_);
  mc_sim_.AtTheBeginningOfEachCase();
  
  //Register log output
  glo_env_->LogSetup(*(sim_config_.main_logger_));
  spacecraft_->LogSetup(*(sim_config_.main_logger_));

  //Write Log headers
  sim_config_.main_logger_->WriteHeaders();

  //Start simulation
  std::cout << "\nSimulationDateTime \n";
  glo_env_->GetSimTime().PrintStartDateTime();
}

void ISSL6UCase::Main()
{
  glo_env_->Reset(); //for MonteCarlo Sim
  while (!glo_env_->GetSimTime().GetState().finish)
  {
    //logging
     if (glo_env_->GetSimTime().GetState().log_output)  sim_config_.main_logger_->WriteValues();
    // Global Environment Update
    glo_env_->Update();
    // Spacecraft Update
    spacecraft_->Update(&(glo_env_->GetSimTime()));
    // Debug output
    if (glo_env_->GetSimTime().GetState().disp_output)
    {
      std::cout << "Progresss: " << glo_env_->GetSimTime().GetProgressionRate() << "%\r";
    }
  }
  //MC finish
  mc_sim_.AtTheEndOfEachCase();
}

//Log for Monte Carlo Simulation
std::string ISSL6UCase::GetLogHeader() const
{
  std::string str_tmp = "";
  str_tmp += WriteScalar("time", "s");
  str_tmp += WriteVector("Omega", "b", "rad/s", 3);

  return str_tmp;
}

std::string ISSL6UCase::GetLogValue() const
{
  std::string str_tmp = "";
  str_tmp += WriteScalar(glo_env_->GetSimTime().GetElapsedSec());
  str_tmp += WriteVector(spacecraft_->GetDynamics().GetAttitude().GetOmega_b(), 3);

  return str_tmp;
}
