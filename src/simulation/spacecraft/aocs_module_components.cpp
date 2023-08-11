#include "aocs_module_components.h"

#include <components/real/aocs/initialize_gnss_receiver.hpp>
#include <components/real/aocs/initialize_gyro_sensor.hpp>
#include <components/real/aocs/initialize_magnetometer.hpp>
#include <components/real/aocs/initialize_magnetorquer.hpp>
#include <components/real/aocs/initialize_reaction_wheel.hpp>
#include <components/real/aocs/initialize_star_sensor.hpp>
#include <components/real/aocs/initialize_sun_sensor.hpp>
#include <components/real/power/csv_scenario_interface.hpp>
#include <library/initialize/initialize_file_access.hpp>
#include <vector>

#include "../../component/aocs/aobc.hpp"
#include "aocs_module_port_config.h"

AocsModuleComponents::AocsModuleComponents(const Dynamics *dynamics, Structure *structure, const LocalEnvironment *local_environment,
                                           const GlobalEnvironment *global_environment, const SimulationConfiguration *configuration,
                                           ClockGenerator *clock_generator, const unsigned int spacecraft_id)
    : dynamics_(dynamics),
      structure_(structure),
      local_environment_(local_environment),
      global_environment_(global_environment),
      configuration_(configuration) {
  // General
  IniAccess iniAccess = IniAccess(configuration_->spacecraft_file_list_[spacecraft_id]);
  // Scenario
  const std::string scenario_file_path = iniAccess.ReadString("SCENARIO", "scenario_file_path");
  CsvScenarioInterface::Initialize(scenario_file_path);

  // CDH
  double compo_step_sec = global_environment_->GetSimulationTime().GetComponentStepTime_s();
  aobc_ = new ObcWithC2a(clock_generator, 10);
  hils_port_manager_ = new HilsPortManager();

  // Power
  // PowerController
  std::vector<int> power_gpio_ports = {38, 60, 61, 73, 91, 6, 7, 8, 71, 79, 80};                            // TODO iniファイルに移す？
  std::vector<double> power_outout_voltage_list = {3.3, 3.3, 3.3, 3.3, 5.0, 5.0, 5.0, 3.3, 7.6, 7.6, 7.6};  // TODO iniファイルに移す？
  power_controller_ = new PowerController(PowerControlUnit(1, clock_generator), power_gpio_ports, power_outout_voltage_list, aobc_);

  // INA
  int ina_prescaler = 1;
  PowerPort *ina_power_port = power_controller_->GetPowerPort((int)PowerPortIdx::INA);
  double ina_min_voltage = 3.3,
         ina_power_consumption = 0.001;  // TODO 初期化ファイルに移動させる？
  unsigned char ina_i2c_port = 4;
  unsigned char ina_i2c_addr = 0x40;
  // PICのみ特別
  ina260s_.push_back(INA260(ina_prescaler, clock_generator, ina_power_port, ina_min_voltage, ina_power_consumption,
                            power_controller_->GetPowerPort((int)PowerPortIdx::PIC), ina_i2c_port, ina_i2c_addr, aobc_));
  // RM,SS,MTQ,STIM,STT,OEM,RWX,RWY,RWZ
  std::vector<unsigned char> ina_i2c_addr_list = {0x44, 0x45, 0x46, 0x41, 0x42, 0x43, 0x47, 0x48, 0x49};
  for (size_t i = 0; i < ina_i2c_addr_list.size(); i++) {
    ina260s_.push_back(INA260(ina_prescaler, clock_generator, ina_power_port, ina_min_voltage, ina_power_consumption,
                              power_controller_->GetPowerPort(i + 2), ina_i2c_port, ina_i2c_addr_list[i], aobc_));  // INAとMPUは観測対象でない
  }

  // AOCS
  const std::string mpu9250_gyro_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "gyro_l_file");
  const unsigned int mpu9250_gyro_hils_port_id = iniAccess.ReadInt("COM_PORT", "mpu9250_gyro_hils_port_id");
  mpu9250_gyro_ = new MPU9250_GYRO(
      InitGyroSensor(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::MPU), 2, mpu9250_gyro_ini_path, compo_step_sec, dynamics_),
      0, mpu9250_gyro_hils_port_id, 0x68, aobc_, hils_port_manager_);
  const std::string mpu9250_mag_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "magsensor_l_file");
  const unsigned int mpu9250_mag_hils_port_id = iniAccess.ReadInt("COM_PORT", "mpu9250_mag_hils_port_id");
  mpu9250_mag_ = new MPU9250_MAG(InitMagnetometer(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::MPU), 2, mpu9250_mag_ini_path,
                                                 compo_step_sec, &(local_environment_->GetGeomagneticField())),
                                 0, mpu9250_mag_hils_port_id, 0x0c, aobc_, hils_port_manager_, mpu9250_gyro_->GetIsMagOn());

  const std::string rm3100_aobc_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "magsensor_h_aobc_file");
  const unsigned int rm3100_aobc_hils_port_id = iniAccess.ReadInt("COM_PORT", "rm3100_aobc_hils_port_id");
  rm3100_aobc_ = new RM3100(Magnetometer(InitMagnetometer(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::RM), 1,
                                                         rm3100_aobc_ini_path, compo_step_sec, &local_environment_->GetGeomagneticField())),
                            0, rm3100_aobc_hils_port_id, 0x20, aobc_, hils_port_manager_);
  const std::string rm3100_ext_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "magsensor_h_ext_file");
  const unsigned int rm3100_ext_hils_port_id = iniAccess.ReadInt("COM_PORT", "rm3100_ext_hils_port_id");
  rm3100_ext_ = new RM3100(Magnetometer(InitMagnetometer(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::RM), 2,
                                                        rm3100_ext_ini_path, compo_step_sec, &local_environment_->GetGeomagneticField())),
                           0, rm3100_ext_hils_port_id, 0x23, aobc_, hils_port_manager_);

  // Sun sensors
  const std::string nanoSSOC_D60_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "ss_file");
  IniAccess ss_ini_file = IniAccess(nanoSSOC_D60_ini_path);
  const size_t number_of_mounted_ss = ss_ini_file.ReadInt("GENERAL", "number_of_mounted_sensors");
  for (size_t ss_idx = 0; ss_idx < number_of_mounted_ss; ss_idx++){
    const unsigned int nanoSSOC_D60_hils_port_id = iniAccess.ReadInt("COM_PORT", "nanoSSOC_D60_pz_hils_port_id");
    const std::string ss_section_name = "I2C_PORT_" + std::to_string(static_cast<long long>(ss_idx));
    const uint8_t i2c_address = ss_ini_file.ReadInt(ss_section_name.c_str(), "i2c_address");
    NanoSSOCD60* ss =
      new NanoSSOCD60(InitSunSensor(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::SS), ss_idx, nanoSSOC_D60_ini_path,
                                    &(local_environment_->GetSolarRadiationPressure()), &(local_environment_->GetCelestialInformation())),
                      0, nanoSSOC_D60_hils_port_id, i2c_address, aobc_, hils_port_manager_);
    nano_ssoc_d60_.push_back(ss);
  }

  const std::string mtq_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "mtq_file");
  std::vector<int> mtq_gpio_ports{78, 81, 82, 83, 84, 68};
  mtq_seiren_ = new MTQseiren(InitMagnetorquer(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::MTQ), 1, mtq_ini_path,
                                               compo_step_sec, &(local_environment_->GetGeomagneticField())),
                              mtq_gpio_ports, aobc_);

  const std::string oem7600_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "gnssr_file");
  const unsigned int oem7600_hils_port_id = iniAccess.ReadInt("COM_PORT", "oem7600_hils_port_id");
  const unsigned int oem7600_baud_rate = iniAccess.ReadInt("COM_PORT", "oem7600_baud_rate");
  oem7600_ =
      new OEM7600(GnssReceiver(InitGnssReceiver(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::OEM), 1, oem7600_ini_path,
                                                dynamics_, &(global_environment_->GetGnssSatellites()), &(global_environment_->GetSimulationTime()))),
                  0x02, aobc_, 0x01, oem7600_hils_port_id, oem7600_baud_rate, hils_port_manager_);

  const std::string sagitta_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "stt_file");
  const unsigned int sagitta_hils_port_id = iniAccess.ReadInt("COM_PORT", "sagitta_hils_port_id");
  const unsigned int sagitta_baud_rate = iniAccess.ReadInt("COM_PORT", "sagitta_baud_rate");
  sagitta_ = new Sagitta(InitStarSensor(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::STT), 0, sagitta_ini_path,
                                        global_environment_->GetSimulationTime().GetSimulationStep_s(), dynamics_, local_environment_),
                         0x05, aobc_, sagitta_hils_port_id, sagitta_baud_rate, hils_port_manager_);

  const std::string stim210_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "gyro_h_file");
  const unsigned int stim210_hils_port_id = iniAccess.ReadInt("COM_PORT", "stim210_hils_port_id");
  const unsigned int stim210_baud_rate = iniAccess.ReadInt("COM_PORT", "stim210_baud_rate");
  stim210_ = new STIM210(
      InitGyroSensor(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::STIM), 1, stim210_ini_path, compo_step_sec, dynamics_),
      compo_step_sec, 0x04, aobc_, stim210_hils_port_id, stim210_baud_rate, hils_port_manager_);

  const std::string rw_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "rw_file");
  const unsigned int rw0003_x_hils_port_id = iniAccess.ReadInt("COM_PORT", "rw0003_x_hils_port_id");
  rw0003_x_ = new RW0003(InitReactionWheel(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::RWX), 1, rw_ini_path,
                                           dynamics_->GetAttitude().GetPropStep_s(), compo_step_sec),
                         1, rw0003_x_hils_port_id, 0x11, aobc_, hils_port_manager_);
  const unsigned int rw0003_y_hils_port_id = iniAccess.ReadInt("COM_PORT", "rw0003_y_hils_port_id");
  rw0003_y_ = new RW0003(InitReactionWheel(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::RWY), 2, rw_ini_path,
                                           dynamics_->GetAttitude().GetPropStep_s(), compo_step_sec),
                         1, rw0003_y_hils_port_id, 0x12, aobc_, hils_port_manager_);
  const unsigned int rw0003_z_hils_port_id = iniAccess.ReadInt("COM_PORT", "rw0003_z_hils_port_id");
  rw0003_z_ = new RW0003(InitReactionWheel(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::RWZ), 3, rw_ini_path,
                                           dynamics_->GetAttitude().GetPropStep_s(), compo_step_sec),
                         1, rw0003_z_hils_port_id, 0x13, aobc_, hils_port_manager_);

  // Thruster
  const std::string thruster_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "thruster_file");
  thruster_ = new SimpleThruster(InitSimpleThruster(clock_generator, 1, thruster_ini_path, structure_, dynamics_));

  // Mission
  const std::string telescope_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "telescope_file");
  telescope_ = new Telescope(InitTelescope(clock_generator, 1, telescope_ini_path, &(dynamics_->GetAttitude()),
                                           &(global_environment_->GetHipparcosCatalog()), &(local_environment_->GetCelestialInformation())));

// HILS IF Board
#ifdef USE_HILS
  const unsigned int hils_if_hils_port_id = iniAccess.ReadInt("COM_PORT", "hils_if_hils_port_id");
  const unsigned int hils_if_baud_rate = iniAccess.ReadInt("COM_PORT", "hils_if_baud_rate");
  hils_if_driver_ = new HilsIfDriver(1, clock_generator, hils_if_hils_port_id, hils_if_baud_rate, hils_port_manager_, mtq_gpio_ports, aobc_);
#endif
}

AocsModuleComponents::~AocsModuleComponents() {
  delete telescope_;
  delete sagitta_;
  delete stim210_;
  delete mpu9250_gyro_;
  delete mpu9250_mag_;
  delete rm3100_aobc_;
  delete rm3100_ext_;
  for (auto nano_ssoc_d60 : nano_ssoc_d60_) {
    delete nano_ssoc_d60;
  }
  delete mtq_seiren_;
  delete rw0003_x_;
  delete rw0003_y_;
  delete rw0003_z_;
  delete oem7600_;
  delete thruster_;
  std::vector<INA260>().swap(ina260s_);
  delete power_controller_;
#ifdef USE_HILS
  delete hils_if_driver_;
#endif
  delete aobc_;
  delete hils_port_manager_;
}

Vector<3> AocsModuleComponents::GenerateForce_b_N() {
  // There is no orbit control component, so it remains 0
  Vector<3> force_b_N_(0.0);
  return force_b_N_;
}

Vector<3> AocsModuleComponents::GenerateTorque_b_Nm() {
  Vector<3> torque_b_Nm_(0.0);
  torque_b_Nm_ += mtq_seiren_->GetOutputTorque_b_Nm();
  torque_b_Nm_ += rw0003_x_->GetOutputTorque_b_Nm();
  torque_b_Nm_ += rw0003_y_->GetOutputTorque_b_Nm();
  torque_b_Nm_ += rw0003_z_->GetOutputTorque_b_Nm();

  return torque_b_Nm_;
}

void AocsModuleComponents::LogSetup(Logger &logger) {
  logger.AddLogList(telescope_);
  logger.AddLogList(sagitta_);
  logger.AddLogList(stim210_);
  logger.AddLogList(mpu9250_gyro_);
  logger.AddLogList(mpu9250_mag_);
  logger.AddLogList(rm3100_aobc_);
  logger.AddLogList(rm3100_ext_);
  for (auto nano_ssoc_d60 : nano_ssoc_d60_) {
    logger.AddLogList(nano_ssoc_d60);
  }
  logger.AddLogList(mtq_seiren_);
  logger.AddLogList(rw0003_x_);
  logger.AddLogList(rw0003_y_);
  logger.AddLogList(rw0003_z_);
  logger.AddLogList(oem7600_);
  logger.AddLogList(thruster_);
}
