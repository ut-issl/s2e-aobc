/**
 * @file s2e_aobc.cpp
 * @brief The main file of S2E-AOBC (Attitude On Board Computer)
 */

#ifdef WIN32
#define _WINSOCKAPI_  // stops windows.h including winsock.h
#include <tchar.h>
#include <windows.h>
#endif

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

// Simulator includes
#include <logger/initialize_log.hpp>
#include <simulation/monte_carlo_simulation/initialize_monte_carlo_simulation.hpp>
#include <utilities/macros.hpp>

// Add custom include files
#include "./simulation/case/sample_case.hpp"

void print_path(std::string path) {
#ifdef WIN32
  std::cout << path << std::endl;
#else
  const char *rpath = realpath(path.c_str(), NULL);
  if (rpath) {
    std::cout << rpath << std::endl;
    free((void *)rpath);
  }
#endif
}

int main(int argc, char *argv[]) {
  UNUSED(argc);
  UNUSED(argv);

  std::string ini_file = "../../data/initialize_files/simulation_base.ini";
  s2e::simulation::MonteCarloSimulationExecutor *monte_carlo_simulator = s2e::simulation::InitMonteCarloSimulation(ini_file);
  s2e::logger::Logger *log_monte_carlo_simulator = s2e::logger::InitMonteCarloLog(ini_file, monte_carlo_simulator->IsEnabled());

  std::cout << "Starting simulation..." << std::endl;
  std::cout << "\tIni file: ";
  print_path(ini_file);

  while (monte_carlo_simulator->WillExecuteNextCase()) {
    std::chrono::system_clock::time_point start, end;
    start = std::chrono::system_clock::now();

    SampleCase simulation_case = SampleCase(ini_file, *monte_carlo_simulator, log_monte_carlo_simulator->GetLogPath());
    // Initialize
    log_monte_carlo_simulator->AddLogList(&simulation_case);
    if (monte_carlo_simulator->GetNumberOfExecutionsDone() == 0) {
      log_monte_carlo_simulator->WriteHeaders();
    }
    simulation_case.Initialize();

    // Main
    log_monte_carlo_simulator->WriteValues();  // log initial value
    simulation_case.Main();
    monte_carlo_simulator->AtTheEndOfEachCase();
    log_monte_carlo_simulator->WriteValues();  // log final value
    log_monte_carlo_simulator->ClearLogList();

    end = std::chrono::system_clock::now();
    double time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000000.0);
    std::cout << std::endl << "Simulation execution time: " << time << "sec" << std::endl << std::endl;
  }
  return EXIT_SUCCESS;
}
