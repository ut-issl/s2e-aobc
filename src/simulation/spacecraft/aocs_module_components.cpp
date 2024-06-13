/**
 * @file aocs_module_components.cpp
 * @brief Components installed on a spacecraft
 */

#include "aocs_module_components.hpp"

#include <components/real/power/csv_scenario_interface.hpp>
#include <library/initialize/initialize_file_access.hpp>
#include <vector>

#include "../../component/aocs/aobc.hpp"
#include "aocs_module_port_config.hpp"

AocsModuleComponents::AocsModuleComponents(const Dynamics *dynamics, Structure *structure, const LocalEnvironment *local_environment,
                                           const GlobalEnvironment *global_environment, const SimulationConfiguration *configuration,
                                           ClockGenerator *clock_generator, const unsigned int spacecraft_id)
    : dynamics_(dynamics),
      structure_(structure),
      local_environment_(local_environment),
      global_environment_(global_environment),
      configuration_(configuration) {
  // General
  IniAccess ini_access = IniAccess(configuration_->spacecraft_file_list_[spacecraft_id]);
  // Scenario
  const std::string scenario_file_path = ini_access.ReadString("SCENARIO", "scenario_file_path");
  CsvScenarioInterface::Initialize(scenario_file_path);

  // CDH
  double compo_step_sec = global_environment_->GetSimulationTime().GetComponentStepTime_s();
  aobc_ = new ObcWithC2a(clock_generator, 10);
  hils_port_manager_ = new HilsPortManager();

  // Power
  // PowerController
  const std::string power_controller_ini_path = ini_access.ReadString("COMPONENTS_FILE", "power_controller_file");
  IniAccess power_controller_access = IniAccess(power_controller_ini_path);
  int number_of_ports = power_controller_access.ReadInt("POWER_CONTROLLER", "number_of_ports");
  std::vector<int> power_gpio_ports = power_controller_access.ReadVectorInt("POWER_CONTROLLER", "gpio_port", number_of_ports);
  std::vector<double> power_output_voltage_list = power_controller_access.ReadVectorDouble("POWER_CONTROLLER", "output_voltage_V", number_of_ports);
  power_controller_ = new PowerController(PowerControlUnit(1, clock_generator), power_gpio_ports, power_output_voltage_list, aobc_);

  // INA
  PowerPort *ina_power_port = power_controller_->GetPowerPort((int)PowerPortIdx::INA);
  double ina_min_voltage = power_controller_access.ReadDouble("POWER_SENSOR", "ina_minimum_voltage_V");
  double ina_power_consumption = power_controller_access.ReadDouble("POWER_SENSOR", "ina_power_consumption_W");
  int ina_prescaler = power_controller_access.ReadInt("POWER_SENSOR", "ina_prescaler");
  unsigned char ina_i2c_port = (unsigned char)power_controller_access.ReadInt("POWER_SENSOR", "i2c_port_id");
  unsigned char ina_i2c_address_pic = (unsigned char)power_controller_access.ReadInt("POWER_SENSOR", "i2c_address_pic");

  // PICのみ特別
  ina260s_.push_back(Ina260(ina_prescaler, clock_generator, ina_power_port, ina_min_voltage, ina_power_consumption,
                            power_controller_->GetPowerPort((int)PowerPortIdx::PIC), ina_i2c_port, ina_i2c_address_pic, aobc_));
  // RM, SS, MTQ, STIM, STT, OEM, RWX, RWY, RWZ
  int number_of_sensors = power_controller_access.ReadInt("POWER_SENSOR", "number_of_sensors");
  std::vector<unsigned char> ina_i2c_address_list = power_controller_access.ReadVectorUnsignedChar("POWER_SENSOR", "i2c_address", number_of_sensors);
  for (size_t i = 0; i < ina_i2c_address_list.size(); i++) {
    ina260s_.push_back(Ina260(ina_prescaler, clock_generator, ina_power_port, ina_min_voltage, ina_power_consumption,
                              power_controller_->GetPowerPort(i + 2), ina_i2c_port, ina_i2c_address_list[i], aobc_));  // INAとMPUは観測対象でない
  }

  // MPU9250
  const std::string mpu9250_gyro_ini_path = ini_access.ReadString("COMPONENTS_FILE", "gyro_l_file");
  const unsigned int mpu9250_gyro_sensor_hils_port_id = ini_access.ReadInt("COM_PORT", "mpu9250_gyro_sensor_hils_port_id");
  IniAccess mpu9250_gyro_ini_access = IniAccess(mpu9250_gyro_ini_path);
  const uint8_t mpu9250_gyro_i2c_address = (uint8_t)mpu9250_gyro_ini_access.ReadInt("I2C_PORT_GYRO", "i2c_address");
  mpu9250_gyro_sensor_ = new Mpu9250GyroSensor(
      InitGyroSensor(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::MPU), 2, mpu9250_gyro_ini_path, compo_step_sec, dynamics_),
      0, mpu9250_gyro_sensor_hils_port_id, mpu9250_gyro_i2c_address, aobc_, hils_port_manager_);

  const std::string mpu9250_mag_ini_path = ini_access.ReadString("COMPONENTS_FILE", "coarse_magnetometer_file");
  const unsigned int mpu9250_magnetometer_hils_port_id = ini_access.ReadInt("COM_PORT", "mpu9250_magnetometer_hils_port_id");
  IniAccess mpu9250_mag_ini_access = IniAccess(mpu9250_mag_ini_path);
  const uint8_t mpu9250_mag_i2c_address = (uint8_t)mpu9250_mag_ini_access.ReadInt("I2C_PORT_MAG", "i2c_address");
  mpu9250_magnetometer_ = new Mpu9250Magnetometer(
      InitMagnetometer(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::MPU), 2, mpu9250_mag_ini_path, compo_step_sec,
                       &(local_environment_->GetGeomagneticField())),
      0, mpu9250_magnetometer_hils_port_id, mpu9250_mag_i2c_address, aobc_, hils_port_manager_, mpu9250_gyro_sensor_->GetIsMagOn());

  // RM3100
  const std::string rm3100_aobc_ini_path = ini_access.ReadString("COMPONENTS_FILE", "fine_magnetometer_file");
  const unsigned int rm3100_aobc_hils_port_id = ini_access.ReadInt("COM_PORT", "rm3100_aobc_hils_port_id");
  IniAccess rm3100_aobc_ini_access = IniAccess(rm3100_aobc_ini_path);
  const uint8_t rm3100_aobc_i2c_address = (uint8_t)rm3100_aobc_ini_access.ReadInt("I2C_PORT_1", "i2c_address");
  rm3100_aobc_ = new Rm3100(Magnetometer(InitMagnetometer(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::RM), 1,
                                                          rm3100_aobc_ini_path, compo_step_sec, &local_environment_->GetGeomagneticField())),
                            0, rm3100_aobc_hils_port_id, rm3100_aobc_i2c_address, aobc_, hils_port_manager_);

  const std::string rm3100_ext_ini_path = ini_access.ReadString("COMPONENTS_FILE", "external_fine_magnetometer_file");
  const unsigned int rm3100_ext_hils_port_id = ini_access.ReadInt("COM_PORT", "rm3100_ext_hils_port_id");
  IniAccess rm3100_ext_ini_access = IniAccess(rm3100_ext_ini_path);
  const uint8_t rm3100_ext_i2c_address = (uint8_t)rm3100_ext_ini_access.ReadInt("I2C_PORT_2", "i2c_address");
  rm3100_external_ = new Rm3100(Magnetometer(InitMagnetometer(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::RM), 2,
                                                              rm3100_ext_ini_path, compo_step_sec, &local_environment_->GetGeomagneticField())),
                                0, rm3100_ext_hils_port_id, rm3100_ext_i2c_address, aobc_, hils_port_manager_);

  // Sun sensors
  const std::string nanoSSOC_D60_ini_path = ini_access.ReadString("COMPONENTS_FILE", "sun_sensor_file");
  IniAccess ss_ini_file = IniAccess(nanoSSOC_D60_ini_path);
  const size_t number_of_mounted_ss = ss_ini_file.ReadInt("GENERAL", "number_of_mounted_sensors");
  for (size_t ss_idx = 0; ss_idx < number_of_mounted_ss; ss_idx++) {
    const std::string hils_port = "nanoSSOC_d60_idx_" + std::to_string(static_cast<long long>(ss_idx)) + "_hils_port_id";
    const unsigned int nanoSSOC_D60_hils_port_id = ini_access.ReadInt("COM_PORT", hils_port.c_str());

    const std::string ss_section_name = "I2C_PORT_" + std::to_string(static_cast<long long>(ss_idx));
    const uint8_t i2c_address = (uint8_t)ss_ini_file.ReadInt(ss_section_name.c_str(), "i2c_address");
    NanoSsocD60 *ss =
        new NanoSsocD60(InitSunSensor(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::SS), ss_idx, nanoSSOC_D60_ini_path,
                                      &(local_environment_->GetSolarRadiationPressure()), &(local_environment_->GetCelestialInformation())),
                        0, nanoSSOC_D60_hils_port_id, i2c_address, aobc_, hils_port_manager_);
    nano_ssoc_d60_.push_back(ss);
  }

  // MTQ
  const std::string mtq_ini_path = ini_access.ReadString("COMPONENTS_FILE", "magnetorquer_file");
  IniAccess mtq_ini_file = IniAccess(mtq_ini_path);
  int number_of_mtq_ports = mtq_ini_file.ReadInt("MTQ_GPIO_PORT", "number_of_ports");
  std::vector<int> mtq_gpio_ports = mtq_ini_file.ReadVectorInt("MTQ_GPIO_PORT", "gpio_port", number_of_mtq_ports);
  mtq_seiren_ = new MtqSeiren(InitMagnetorquer(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::MTQ), 1, mtq_ini_path,
                                               compo_step_sec, &(local_environment_->GetGeomagneticField())),
                              mtq_gpio_ports, aobc_);

  // GPS-R
  const std::string oem7600_ini_path = ini_access.ReadString("COMPONENTS_FILE", "gnss_receiver_file");
  IniAccess oem7600_ini_file = IniAccess(oem7600_ini_path);
  const unsigned char oem7600_uart_sils_port = (unsigned char)oem7600_ini_file.ReadInt("UART_PORT", "uart_port_sils");
  const unsigned char oem7600_com_port = (unsigned char)oem7600_ini_file.ReadInt("UART_PORT", "oem_com_port");
  const unsigned int oem7600_hils_port_id = ini_access.ReadInt("COM_PORT", "oem7600_hils_port_id");
  const unsigned int oem7600_baud_rate = ini_access.ReadInt("COM_PORT", "oem7600_baud_rate");
  oem7600_ =
      new Oem7600(GnssReceiver(InitGnssReceiver(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::OEM), 1, oem7600_ini_path,
                                                dynamics_, &(global_environment_->GetGnssSatellites()), &(global_environment_->GetSimulationTime()))),
                  oem7600_uart_sils_port, aobc_, oem7600_com_port, oem7600_hils_port_id, oem7600_baud_rate, hils_port_manager_);

  // STT
  const std::string sagitta_ini_path = ini_access.ReadString("COMPONENTS_FILE", "star_sensor_file");
  IniAccess sagitta_ini_file = IniAccess(sagitta_ini_path);
  const unsigned char sagitta_uart_sils_port = (unsigned char)sagitta_ini_file.ReadInt("UART_PORT", "uart_port_sils");
  const unsigned int sagitta_hils_port_id = ini_access.ReadInt("COM_PORT", "sagitta_hils_port_id");
  const unsigned int sagitta_baud_rate = ini_access.ReadInt("COM_PORT", "sagitta_baud_rate");
  sagitta_ = new Sagitta(InitStarSensor(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::STT), 1, sagitta_ini_path,
                                        global_environment_->GetSimulationTime().GetSimulationStep_s(), dynamics_, local_environment_),
                         sagitta_uart_sils_port, aobc_, sagitta_hils_port_id, sagitta_baud_rate, hils_port_manager_);

  // Accurate gyro sensor
  const std::string stim210_ini_path = ini_access.ReadString("COMPONENTS_FILE", "fine_gyro_sensor_file");
  IniAccess stim210_ini_file = IniAccess(stim210_ini_path);
  const unsigned char stim210_uart_sils_port = (unsigned char)stim210_ini_file.ReadInt("UART_PORT", "uart_port_sils");
  const unsigned int stim210_hils_port_id = ini_access.ReadInt("COM_PORT", "stim210_hils_port_id");
  const unsigned int stim210_baud_rate = ini_access.ReadInt("COM_PORT", "stim210_baud_rate");
  stim210_ = new Stim210(
      InitGyroSensor(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::STIM), 1, stim210_ini_path, compo_step_sec, dynamics_),
      compo_step_sec, stim210_uart_sils_port, aobc_, stim210_hils_port_id, stim210_baud_rate, hils_port_manager_);

  // Reaction Wheel
  const std::string rw_ini_path = ini_access.ReadString("COMPONENTS_FILE", "reaction_wheel_file");
  IniAccess rw_ini_access = IniAccess(rw_ini_path);
  const unsigned int rw0003_x_hils_port_id = ini_access.ReadInt("COM_PORT", "rw0003_x_hils_port_id");
  const uint8_t i2c_address_x = (uint8_t)rw_ini_access.ReadInt("I2C_PORT_1", "i2c_address");
  rw0003_x_ = new Rw0003(InitReactionWheel(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::RWX), 1, rw_ini_path, compo_step_sec),
                         1, rw0003_x_hils_port_id, i2c_address_x, aobc_, hils_port_manager_);

  const unsigned int rw0003_y_hils_port_id = ini_access.ReadInt("COM_PORT", "rw0003_y_hils_port_id");
  const uint8_t i2c_address_y = (uint8_t)rw_ini_access.ReadInt("I2C_PORT_2", "i2c_address");
  rw0003_y_ = new Rw0003(InitReactionWheel(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::RWY), 2, rw_ini_path, compo_step_sec),
                         1, rw0003_y_hils_port_id, i2c_address_y, aobc_, hils_port_manager_);

  const unsigned int rw0003_z_hils_port_id = ini_access.ReadInt("COM_PORT", "rw0003_z_hils_port_id");
  const uint8_t i2c_address_z = (uint8_t)rw_ini_access.ReadInt("I2C_PORT_3", "i2c_address");
  rw0003_z_ = new Rw0003(InitReactionWheel(clock_generator, power_controller_->GetPowerPort((int)PowerPortIdx::RWZ), 3, rw_ini_path, compo_step_sec),
                         1, rw0003_z_hils_port_id, i2c_address_z, aobc_, hils_port_manager_);

  // Component interference
  const std::string interference_file_path = ini_access.ReadString("COMPONENTS_FILE", "component_interference_file");
  configuration_->main_logger_->CopyFileToLogDirectory(interference_file_path);
  mtq_mpu9250_magnetometer_interference_ = new MtqMagnetometerInterference(interference_file_path, *mpu9250_magnetometer_, *mtq_seiren_, 0);
  mtq_rm3100_aobc_interference_ = new MtqMagnetometerInterference(interference_file_path, *rm3100_aobc_, *mtq_seiren_, 1);
  mtq_rm3100_external_interference_ = new MtqMagnetometerInterference(interference_file_path, *rm3100_external_, *mtq_seiren_, 2);

  // Thruster
  const std::string thruster_ini_path = ini_access.ReadString("COMPONENTS_FILE", "thruster_file");
  thruster_ = new SimpleThruster(InitSimpleThruster(clock_generator, 1, thruster_ini_path, structure_, dynamics_));

  // Mission
  const std::string telescope_ini_path = ini_access.ReadString("COMPONENTS_FILE", "telescope_file");
  telescope_ = new Telescope(InitTelescope(clock_generator, 1, telescope_ini_path, &(dynamics_->GetAttitude()),
                                           &(global_environment_->GetHipparcosCatalog()), &(local_environment_->GetCelestialInformation()),
                                           &(global_environment_->GetSimulationTime()), &(dynamics_->GetOrbit())));

  // Communication
  const std::string command_sender_ini_path = ini_access.ReadString("COMPONENTS_FILE", "command_sender_file");
  wings_command_sender_to_c2a_ = new WingsCommandSenderToC2a(InitWingsCommandSenderToC2a(clock_generator, compo_step_sec, command_sender_ini_path));

// HILS IF Board
#ifdef USE_HILS
  const unsigned int hils_if_hils_port_id = ini_access.ReadInt("COM_PORT", "hils_if_hils_port_id");
  const unsigned int hils_if_baud_rate = ini_access.ReadInt("COM_PORT", "hils_if_baud_rate");
  hils_if_driver_ = new HilsIfDriver(1, clock_generator, hils_if_hils_port_id, hils_if_baud_rate, hils_port_manager_, mtq_gpio_ports, aobc_);
#endif
}

AocsModuleComponents::~AocsModuleComponents() {
  delete wings_command_sender_to_c2a_;
  delete telescope_;
  delete sagitta_;
  delete stim210_;
  delete mpu9250_gyro_sensor_;
  delete mpu9250_magnetometer_;
  delete rm3100_aobc_;
  delete rm3100_external_;
  for (auto nano_ssoc_d60 : nano_ssoc_d60_) {
    delete nano_ssoc_d60;
  }
  delete mtq_seiren_;
  delete rw0003_x_;
  delete rw0003_y_;
  delete rw0003_z_;
  delete oem7600_;
  delete thruster_;
  std::vector<Ina260>().swap(ina260s_);
  delete power_controller_;
#ifdef USE_HILS
  delete hils_if_driver_;
#endif
  delete aobc_;
  delete hils_port_manager_;
}

Vector<3> AocsModuleComponents::GenerateForce_b_N() {
  Vector<3> force_b_N_(0.0);
  // TODO: Add thruster force
  return force_b_N_;
}

Vector<3> AocsModuleComponents::GenerateTorque_b_Nm() {
  Vector<3> torque_b_Nm_(0.0);
  torque_b_Nm_ += mtq_seiren_->GetOutputTorque_b_Nm();
  torque_b_Nm_ += rw0003_x_->GetOutputTorque_b_Nm();
  torque_b_Nm_ += rw0003_y_->GetOutputTorque_b_Nm();
  torque_b_Nm_ += rw0003_z_->GetOutputTorque_b_Nm();
  // TODO: Add thruster torque

  return torque_b_Nm_;
}

void AocsModuleComponents::LogSetup(Logger &logger) {
  logger.AddLogList(telescope_);
  logger.AddLogList(sagitta_);
  logger.AddLogList(stim210_);
  logger.AddLogList(mpu9250_gyro_sensor_);
  logger.AddLogList(mpu9250_magnetometer_);
  logger.AddLogList(rm3100_aobc_);
  logger.AddLogList(rm3100_external_);
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
