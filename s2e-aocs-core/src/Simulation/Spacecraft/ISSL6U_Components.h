#pragma once

#include <dynamics/dynamics.hpp>
#include <library/math/vector.hpp>
#include <simulation/spacecraft/installed_components.hpp>

// CDH
#include <components/real/cdh/on_board_computer_with_c2a.hpp>
// Power
#include "../../Component/Power/PowerController.h"
#include "../../Component/Power/INA260.h"
// AOCS
#include "../../Component/AOCS/Sagitta.h"
#include "../../Component/AOCS/STIM210.h"
#include "../../Component/AOCS/MPU9250_GYRO.h"
#include "../../Component/AOCS/MPU9250_MAG.h"
#include "../../Component/AOCS/RM3100.h"
#include "../../Component/AOCS/NanoSSOCD60.h"
#include "../../Component/AOCS/MTQseiren.h"
#include "../../Component/AOCS/OEM7600.h"
#include "../../Component/AOCS/RW0003.h"
// Propulsion
#include <components/real/propulsion/initialize_simple_thruster.hpp>
// Mission
#include <component/real/mission/initialize_telescope.hpp>
// HILS IF
#include "../../Interface/HILS/HilsIfDriver.h"

using libra::Vector;

class ISSL6UComponents : public InstalledComponents
{
public:
  ISSL6UComponents(const Dynamics *dynamics, Structure *structure, const LocalEnvironment *local_environment,
                   const GlobalEnvironment *global_environment, const SimulationConfiguration *configuration, ClockGenerator *clock_generator,
                   const unsigned int spacecraft_id);
  ~ISSL6UComponents();
  libra::Vector<3> GenerateForce_b_N();
  libra::Vector<3> GenerateTorque_b_Nm();
  void LogSetup(Logger &logger);

  // Getter
  // コンポ操作のためconstにしない(privateの意味？)
  inline MagSensor &GetMagH() { return *rm3100_aobc_; }
  inline MagTorquer &GetMTQ() { return *mtq_seiren_; }
  inline SimpleThruster &GetThruster() { return *thruster_; }
  inline const LocalEnvironment &GetLocalEnv() { return *local_env_; }
  inline GNSSReceiver &GetGNSSR() { return *oem7600_; }

private:
  // CDH
  ObcWithC2a *aobc_;
  // Power
  PowerController *power_controller_;
  vector<INA260> ina260s_;
  // AOCS
  MPU9250_GYRO *mpu9250_gyro_;
  MPU9250_MAG *mpu9250_mag_;
  RM3100 *rm3100_aobc_;
  RM3100 *rm3100_ext_;
  NanoSSOCD60 *nanoSSOC_D60_pz_;
  NanoSSOCD60 *nanoSSOC_D60_py_;
  NanoSSOCD60 *nanoSSOC_D60_mz_;
  NanoSSOCD60 *nanoSSOC_D60_my_;
  MTQseiren *mtq_seiren_;

  OEM7600 *oem7600_; // GNSS Receiver
  Sagitta *sagitta_;
  STIM210 *stim210_;

  RW0003 *rw0003_x_;
  RW0003 *rw0003_y_;
  RW0003 *rw0003_z_;
  // Thruster
  SimpleThruster *thruster_;
  // Mission
  Telescope *telescope_;
  // HILS
  HilsPortManager *hils_port_manager_;
  HilsIfDriver *hils_if_driver_;

  // States
  const SimulationConfiguration *configuration_; //!< Simulation settings
  const Dynamics *dynamics_;                     //!< Dynamics information of the spacecraft
  Structure *structure_;                         //!< Structure information of the spacecraft
  const LocalEnvironment *local_environment_;    //!< Local environment information around the spacecraft
  const GlobalEnvironment *global_environment_;  //!< Global environment information
};
