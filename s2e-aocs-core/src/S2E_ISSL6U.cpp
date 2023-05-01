/**
 * @file S2E_ISSL6U.cpp
 * @brief The main file of S2E-USER
 */

#ifdef WIN32
#define _WINSOCKAPI_ // stops windows.h including winsock.h
#include <tchar.h>
#include <windows.h>
#endif

#include <cstdio>
#include <iostream>
#include <string>
#include <cstdlib>

// Simulator includes
#include <library/utilities/macros.hpp>
#include <simulation/monte_carlo_simulation/initialize_monte_carlo_simulation.hpp>
#include <library/logger/logger.hpp>

// Add custom include files
#include "./Simulation/Case/ISSL6U_case.h"

void print_path(std::string path)
{
#ifdef WIN32
  std::cout << path << std::endl;
#else
  const char *rpath = realpath(path.c_str(), NULL);
  if (rpath)
  {
    std::cout << rpath << std::endl;
    free((void *)rpath);
  }
#endif
}

int main(int argc, char *argv[])
{
  UNUSED(argc);
  UNUSED(argv);

  std::string ini_file = "../../data/initialize_files/simulation_base.ini";
  MCSimExecutor *mc_sim = InitMCSim(ini_file);
  Logger *log_mc_sim = InitLogMC(ini_file, mc_sim->IsEnabled());

  std::cout << "Starting simulation..." << std::endl;
  std::cout << "\tIni file: ";
  print_path(ini_file);

  while (mc_sim->WillExecuteNextCase())
  {
    std::chrono::system_clock::time_point start, end;
    start = std::chrono::system_clock::now();

    auto simcase = ISSL6UCase(ini_file, *mc_sim, log_mc_sim->GetLogPath());
    // Initialize
    log_mc_sim->AddLogList(&simcase);
    if (mc_sim->GetNumOfExecutionsDone() == 0)
      log_mc_sim->WriteHeaders();
    simcase.Initialize();

    // Main
    log_mc_sim->WriteValues(); // log initial value
    simcase.Main();
    mc_sim->AtTheEndOfEachCase();
    log_mc_sim->WriteValues(); // log final value
    log_mc_sim->ClearLogList();

    end = std::chrono::system_clock::now();
    double time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000000.0);
    std::cout << std::endl
              << "Simulation execution time: " << time << "sec" << std::endl
              << std::endl;
  }
  return EXIT_SUCCESS;
}
