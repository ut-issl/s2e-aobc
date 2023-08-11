#ifndef S2E_AOBC_SIMULATION_SPACECRAFT_AOCS_MODULE_COMPONENTS_HPP_
#define S2E_AOBC_SIMULATION_SPACECRAFT_AOCS_MODULE_COMPONENTS_HPP_

#include <dynamics/dynamics.hpp>
#include <library/math/vector.hpp>
#include <simulation/spacecraft/installed_components.hpp>

// CDH
#include <components/real/cdh/on_board_computer_with_c2a.hpp>
// Power
#include "../../component/power/ina260.hpp"
#include "../../component/power/power_controller.hpp"
// AOCS
#include "../../component/aocs/mpu9250_gyro.hpp"
#include "../../component/aocs/mpu9250_mag.hpp"
#include "../../component/aocs/mtq_seiren.hpp"
#include "../../component/aocs/nanossoc_d60.hpp"
#include "../../component/aocs/oem7600.hpp"
#include "../../component/aocs/rm3100.hpp"
#include "../../component/aocs/rw0003.hpp"
#include "../../component/aocs/sagitta.hpp"
#include "../../component/aocs/stim210.hpp"
// Propulsion
#include <components/real/propulsion/initialize_simple_thruster.hpp>
// Mission
#include <components/real/mission/initialize_telescope.hpp>
// HILS IF
#include "../../interface/hils/hils_if_driver.hpp"

using libra::Vector;

class AocsModuleComponents : public InstalledComponents {
 public:
  AocsModuleComponents(const Dynamics *dynamics, Structure *structure, const LocalEnvironment *local_environment,
                       const GlobalEnvironment *global_environment, const SimulationConfiguration *configuration, ClockGenerator *clock_generator,
                       const unsigned int spacecraft_id);
  ~AocsModuleComponents();
  libra::Vector<3> GenerateForce_b_N();
  libra::Vector<3> GenerateTorque_b_Nm();
  void LogSetup(Logger &logger);

  // Getter
  // コンポ操作のためconstにしない(privateの意味？)
  inline Magnetometer &GetMagH() { return *rm3100_aobc_; }
  inline Magnetorquer &GetMTQ() { return *mtq_seiren_; }
  inline SimpleThruster &GetThruster() { return *thruster_; }
  inline const LocalEnvironment &GetLocalEnv() { return *local_environment_; }
  inline GnssReceiver &GetGNSSR() { return *oem7600_; }

 private:
  // CDH
  ObcWithC2a *aobc_;
  // Power
  PowerController *power_controller_;
  std::vector<INA260> ina260s_;
  // AOCS
  MPU9250_GYRO *mpu9250_gyro_;
  MPU9250_MAG *mpu9250_mag_;
  RM3100 *rm3100_aobc_;
  RM3100 *rm3100_ext_;
  std::vector<NanoSSOCD60*> nano_ssoc_d60_;
  MTQseiren *mtq_seiren_;

  OEM7600 *oem7600_;  // GNSS Receiver
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
  const Dynamics *dynamics_;                      //!< Dynamics information of the spacecraft
  Structure *structure_;                          //!< Structure information of the spacecraft
  const LocalEnvironment *local_environment_;     //!< Local environment information around the spacecraft
  const GlobalEnvironment *global_environment_;   //!< Global environment information
  const SimulationConfiguration *configuration_;  //!< Simulation settings
};

#endif  // S2E_AOBC_SIMULATION_SPACECRAFT_AOCS_MODULE_COMPONENTS_HPP_
