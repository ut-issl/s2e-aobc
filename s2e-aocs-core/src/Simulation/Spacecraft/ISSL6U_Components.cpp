#include "ISSL6U_Components.h"
#include "ISSL6U_PortConfig.h"
#include "../../Component/AOCS/AOBC.h"

#include <Component/AOCS/InitGyro.hpp>
#include <Component/AOCS/InitSunSensor.hpp>
#include <Component/AOCS/InitMagSensor.hpp>
#include <Component/AOCS/InitStt.hpp>
#include <Component/AOCS/InitGnssReceiver.hpp>
#include <Component/AOCS/InitMagTorquer.hpp>
#include <Component/AOCS/InitRwModel.hpp>
#include <Component/Propulsion/InitSimpleThruster.hpp>
#include <Component/Mission/Telescope/InitTelescope.hpp>
#include <Component/Power/CsvScenarioInterface.h>

#include <Interface/InitInput/IniAccess.h>

#include <vector>

ISSL6UComponents::ISSL6UComponents(
  const Dynamics* dynamics,
  const Structure* structure,
  const LocalEnvironment* local_env,
  const GlobalEnvironment* glo_env,
  const SimulationConfig* config,
  ClockGenerator* clock_gen,
  const int sat_id)
  :dynamics_(dynamics), structure_(structure), local_env_(local_env), glo_env_(glo_env),config_(config)
{
  //General
  IniAccess iniAccess = IniAccess(config->sat_file_[sat_id]);
  //Scenario
  const std::string scenario_file_path = iniAccess.ReadString("SCENARIO", "scenario_file_path");
  CsvScenarioInterface::Initialize(scenario_file_path);

  // CDH
  double compo_step_sec = glo_env_->GetSimTime().GetCompoStepSec();
  aobc_ = new OBC_C2A(clock_gen,10);
  hils_port_manager_ = new HilsPortManager();

  // Power
  // PowerController
  vector<int> power_gpio_ports = {38, 60, 61, 73, 91, 6, 7, 8, 71, 79, 80}; // TODO iniファイルに移す？
  vector<double> power_outout_voltage_list = {3.3, 3.3, 3.3, 3.3, 5.0, 5.0, 5.0, 3.3, 7.6, 7.6, 7.6}; // TODO iniファイルに移す？
  power_controller_ = new PowerController(PCU(1, clock_gen), power_gpio_ports, power_outout_voltage_list, aobc_);

  // INA
  int ina_prescaler = 1;
  PowerPort* ina_power_port = power_controller_->GetPowerPort((int)PowerPortIdx::INA);
  double ina_min_voltage = 3.3, ina_power_consumption = 0.001;  // TODO 初期化ファイルに移動させる？
  unsigned char ina_i2c_port = 4;
  unsigned char ina_i2c_addr = 0x40;
  // PICのみ特別
  ina260s_.push_back(INA260(ina_prescaler, clock_gen, ina_power_port, ina_min_voltage, ina_power_consumption,
                            power_controller_->GetPowerPort((int)PowerPortIdx::PIC), ina_i2c_port, ina_i2c_addr, aobc_));
  // RM,SS,MTQ,STIM,STT,OEM,RWX,RWY,RWZ
  vector<unsigned char> ina_i2c_addr_list = {0x44, 0x45, 0x46, 0x41, 0x42, 0x43, 0x47, 0x48, 0x49};
  for(size_t i = 0; i < ina_i2c_addr_list.size(); i++)
  {
    ina260s_.push_back(INA260(ina_prescaler, clock_gen, ina_power_port, ina_min_voltage, ina_power_consumption,
                              power_controller_->GetPowerPort(i+2), ina_i2c_port, ina_i2c_addr_list[i], aobc_));  //INAとMPUは観測対象でない
  }

  // AOCS
  const std::string mpu9250_gyro_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "gyro_l_file");
  const unsigned int mpu9250_gyro_hils_port_id = iniAccess.ReadInt("COM_PORT", "mpu9250_gyro_hils_port_id");
  mpu9250_gyro_ = new MPU9250_GYRO(InitGyro(clock_gen, power_controller_->GetPowerPort((int)PowerPortIdx::MPU), 2, mpu9250_gyro_ini_path, compo_step_sec, dynamics_),
                                   0, mpu9250_gyro_hils_port_id, 0x68, aobc_, hils_port_manager_);
  const std::string mpu9250_mag_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "magsensor_l_file");
  const unsigned int mpu9250_mag_hils_port_id = iniAccess.ReadInt("COM_PORT", "mpu9250_mag_hils_port_id");
  mpu9250_mag_ = new MPU9250_MAG(InitMagSensor(clock_gen, power_controller_->GetPowerPort((int)PowerPortIdx::MPU), 2, mpu9250_mag_ini_path, compo_step_sec, &(local_env_->GetMag())),
                                 0, mpu9250_mag_hils_port_id, 0x0c, aobc_, hils_port_manager_, mpu9250_gyro_->GetIsMagOn());

  const std::string rm3100_aobc_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "magsensor_h_aobc_file");
  const unsigned int rm3100_aobc_hils_port_id = iniAccess.ReadInt("COM_PORT", "rm3100_aobc_hils_port_id");
  rm3100_aobc_ = new RM3100(MagSensor(InitMagSensor(clock_gen, power_controller_->GetPowerPort((int)PowerPortIdx::RM), 1, rm3100_aobc_ini_path, compo_step_sec, &local_env_->GetMag())), 0, rm3100_aobc_hils_port_id, 0x20, aobc_, hils_port_manager_);
  const std::string rm3100_ext_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "magsensor_h_ext_file");
  const unsigned int rm3100_ext_hils_port_id = iniAccess.ReadInt("COM_PORT", "rm3100_ext_hils_port_id");
  rm3100_ext_  = new RM3100(MagSensor(InitMagSensor(clock_gen, power_controller_->GetPowerPort((int)PowerPortIdx::RM), 2, rm3100_ext_ini_path, compo_step_sec, &local_env_->GetMag())), 0, rm3100_ext_hils_port_id, 0x23, aobc_, hils_port_manager_);

  const std::string nanoSSOC_D60_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "ss_file");
  const unsigned int nanoSSOC_D60_pz_hils_port_id = iniAccess.ReadInt("COM_PORT", "nanoSSOC_D60_pz_hils_port_id");
  nanoSSOC_D60_pz_ = new NanoSSOCD60(InitSunSensor(clock_gen, power_controller_->GetPowerPort((int)PowerPortIdx::SS), 1, nanoSSOC_D60_ini_path, &(local_env_->GetSrp()), &(local_env_->GetCelesInfo())), 0, nanoSSOC_D60_pz_hils_port_id, 0x6B, aobc_, hils_port_manager_);
  const unsigned int nanoSSOC_D60_py_hils_port_id = iniAccess.ReadInt("COM_PORT", "nanoSSOC_D60_py_hils_port_id");
  nanoSSOC_D60_py_ = new NanoSSOCD60(InitSunSensor(clock_gen, power_controller_->GetPowerPort((int)PowerPortIdx::SS), 2, nanoSSOC_D60_ini_path, &(local_env_->GetSrp()), &(local_env_->GetCelesInfo())), 0, nanoSSOC_D60_py_hils_port_id, 0x69, aobc_, hils_port_manager_);
  const unsigned int nanoSSOC_D60_mz_hils_port_id = iniAccess.ReadInt("COM_PORT", "nanoSSOC_D60_mz_hils_port_id");
  nanoSSOC_D60_mz_ = new NanoSSOCD60(InitSunSensor(clock_gen, power_controller_->GetPowerPort((int)PowerPortIdx::SS), 3, nanoSSOC_D60_ini_path, &(local_env_->GetSrp()), &(local_env_->GetCelesInfo())), 0, nanoSSOC_D60_mz_hils_port_id, 0x63, aobc_, hils_port_manager_);
  const unsigned int nanoSSOC_D60_my_hils_port_id = iniAccess.ReadInt("COM_PORT", "nanoSSOC_D60_my_hils_port_id");
  nanoSSOC_D60_my_ = new NanoSSOCD60(InitSunSensor(clock_gen, power_controller_->GetPowerPort((int)PowerPortIdx::SS), 4, nanoSSOC_D60_ini_path, &(local_env_->GetSrp()), &(local_env_->GetCelesInfo())), 0, nanoSSOC_D60_my_hils_port_id, 0x6A, aobc_, hils_port_manager_);

  const std::string mtq_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "mtq_file");
  vector<int> mtq_gpio_ports{78, 81, 82, 83, 84, 68};
  mtq_seiren_ = new MTQseiren(InitMagTorquer(clock_gen, power_controller_->GetPowerPort((int)PowerPortIdx::MTQ), 1, mtq_ini_path, compo_step_sec, &(local_env_->GetMag())), mtq_gpio_ports, aobc_);

  const std::string oem7600_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "gnssr_file");
  const unsigned int oem7600_hils_port_id = iniAccess.ReadInt("COM_PORT", "oem7600_hils_port_id");
  const unsigned int oem7600_baud_rate = iniAccess.ReadInt("COM_PORT", "oem7600_baud_rate");
  oem7600_ = new OEM7600(GNSSReceiver(InitGNSSReceiver(clock_gen, power_controller_->GetPowerPort((int)PowerPortIdx::OEM), 1, oem7600_ini_path, dynamics_, &(glo_env_->GetGnssSatellites()), &(glo_env_->GetSimTime()))), 0x02, aobc_, 0x01, oem7600_hils_port_id, oem7600_baud_rate, hils_port_manager_);

  const std::string sagitta_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "stt_file");
  const unsigned int sagitta_hils_port_id = iniAccess.ReadInt("COM_PORT", "sagitta_hils_port_id");
  const unsigned int sagitta_baud_rate = iniAccess.ReadInt("COM_PORT", "sagitta_baud_rate");
  sagitta_ = new Sagitta(InitSTT(clock_gen, power_controller_->GetPowerPort((int)PowerPortIdx::STT), 0, sagitta_ini_path, glo_env_->GetSimTime().GetStepSec(), dynamics_,local_env_), 0x05, aobc_, sagitta_hils_port_id, sagitta_baud_rate, hils_port_manager_);

  const std::string stim210_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "gyro_h_file");
  const unsigned int stim210_hils_port_id = iniAccess.ReadInt("COM_PORT", "stim210_hils_port_id");
  const unsigned int stim210_baud_rate = iniAccess.ReadInt("COM_PORT", "stim210_baud_rate");
  stim210_ = new STIM210(InitGyro(clock_gen, power_controller_->GetPowerPort((int)PowerPortIdx::STIM), 1, stim210_ini_path, compo_step_sec, dynamics_), compo_step_sec, 0x04, aobc_, stim210_hils_port_id, stim210_baud_rate, hils_port_manager_);

  const std::string rw_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "rw_file");
  const unsigned int rw0003_x_hils_port_id = iniAccess.ReadInt("COM_PORT", "rw0003_x_hils_port_id");
  rw0003_x_ = new RW0003(InitRWModel(clock_gen, power_controller_->GetPowerPort((int)PowerPortIdx::RWX), 1,rw_ini_path,dynamics_->GetAttitude().GetPropStep(),compo_step_sec), 1, rw0003_x_hils_port_id, 0x11, aobc_, hils_port_manager_);
  const unsigned int rw0003_y_hils_port_id = iniAccess.ReadInt("COM_PORT", "rw0003_y_hils_port_id");
  rw0003_y_ = new RW0003(InitRWModel(clock_gen, power_controller_->GetPowerPort((int)PowerPortIdx::RWY), 2,rw_ini_path,dynamics_->GetAttitude().GetPropStep(),compo_step_sec), 1, rw0003_y_hils_port_id, 0x12, aobc_, hils_port_manager_);
  const unsigned int rw0003_z_hils_port_id = iniAccess.ReadInt("COM_PORT", "rw0003_z_hils_port_id");
  rw0003_z_ = new RW0003(InitRWModel(clock_gen, power_controller_->GetPowerPort((int)PowerPortIdx::RWZ), 3,rw_ini_path,dynamics_->GetAttitude().GetPropStep(),compo_step_sec), 1, rw0003_z_hils_port_id, 0x13, aobc_, hils_port_manager_);

  //Thruster
  const std::string thruster_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "thruster_file");
  thruster_ = new SimpleThruster(InitSimpleThruster(clock_gen, 1, thruster_ini_path, structure_, dynamics_));

  //Mission
  const std::string telescope_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "telescope_file");
  telescope_ = new Telescope(InitTelescope(clock_gen, 1, telescope_ini_path, &(dynamics_->GetAttitude()), &(glo_env_->GetHippCatalog()), &(local_env_->GetCelesInfo())));

  // HILS IF Board
  #ifdef USE_HILS
    const unsigned int hils_if_hils_port_id = iniAccess.ReadInt("COM_PORT", "hils_if_hils_port_id");
    const unsigned int hils_if_baud_rate = iniAccess.ReadInt("COM_PORT", "hils_if_baud_rate");
    hils_if_driver_ = new HilsIfDriver(1, clock_gen, hils_if_hils_port_id, hils_if_baud_rate, hils_port_manager_, mtq_gpio_ports, aobc_);
  #endif
}

ISSL6UComponents::~ISSL6UComponents()
{
  delete telescope_;
  delete sagitta_;
  delete stim210_;
  delete mpu9250_gyro_;
  delete mpu9250_mag_;
  delete rm3100_aobc_;
  delete rm3100_ext_;
  delete nanoSSOC_D60_pz_;
  delete nanoSSOC_D60_py_;
  delete nanoSSOC_D60_mz_;
  delete nanoSSOC_D60_my_;
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


Vector<3> ISSL6UComponents::GenerateForce_N_b()
{
  //There is no orbit control component, so it remains 0
  Vector<3> force_N_b_(0.0);
  return force_N_b_;
}

Vector<3> ISSL6UComponents::GenerateTorque_Nm_b()
{
  Vector<3> torque_Nm_b_(0.0);
  torque_Nm_b_ += mtq_seiren_->GetTorque_b();
  torque_Nm_b_ += rw0003_x_->GetOutputTorqueB();
  torque_Nm_b_ += rw0003_y_->GetOutputTorqueB();
  torque_Nm_b_ += rw0003_z_->GetOutputTorqueB();

  return torque_Nm_b_;
}

void ISSL6UComponents::LogSetup(Logger& logger)
{
  logger.AddLoggable(telescope_);
  logger.AddLoggable(sagitta_);
  logger.AddLoggable(stim210_);
  logger.AddLoggable(mpu9250_gyro_);
  logger.AddLoggable(mpu9250_mag_);
  logger.AddLoggable(rm3100_aobc_);
  logger.AddLoggable(rm3100_ext_);
  logger.AddLoggable(nanoSSOC_D60_pz_);
  logger.AddLoggable(nanoSSOC_D60_py_);
  logger.AddLoggable(nanoSSOC_D60_mz_);
  logger.AddLoggable(nanoSSOC_D60_my_);
  logger.AddLoggable(mtq_seiren_);
  logger.AddLoggable(rw0003_x_);
  logger.AddLoggable(rw0003_y_);
  logger.AddLoggable(rw0003_z_);
  logger.AddLoggable(oem7600_);
  logger.AddLoggable(thruster_);
}
