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
#include <library/logger/initialize_log.hpp>
#include <simulation/monte_carlo_simulation/initialize_monte_carlo_simulation.hpp>

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
  MonteCarloSimulationExecutor *mc_simulator = InitMonteCarloSimulation(ini_file);
  Logger *log_mc_simulator = InitMonteCarloLog(ini_file, mc_simulator->IsEnabled());

  std::cout << "Starting simulation..." << std::endl;
  std::cout << "\tIni file: ";
  print_path(ini_file);

  while (mc_simulator->WillExecuteNextCase())
  {
    std::chrono::system_clock::time_point start, end;
    start = std::chrono::system_clock::now();

    auto simcase = ISSL6UCase(ini_file, *mc_simulator, log_mc_simulator->GetLogPath());
    // Initialize
    log_mc_simulator->AddLogList(&simcase);
    if (mc_simulator->GetNumberOfExecutionsDone() == 0)
    {
      log_mc_simulator->WriteHeaders();
    }
    simcase.Initialize();

    // Main
    log_mc_simulator->WriteValues(); // log initial value
    simcase.Main();
    mc_simulator->AtTheEndOfEachCase();
    log_mc_simulator->WriteValues(); // log final value
    log_mc_simulator->ClearLogList();

    end = std::chrono::system_clock::now();
    double time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000000.0);
    std::cout << std::endl
              << "Simulation execution time: " << time << "sec" << std::endl
              << std::endl;
  }
  return EXIT_SUCCESS;
}
