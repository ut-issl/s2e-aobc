#include "MPU9250_GYRO.h"
#include <library/math/constants.hpp>
#include <library/utilities/macros.hpp>

MPU9250_GYRO::MPU9250_GYRO(
  GyroSensor gyro,
  const int sils_port_id,
  const unsigned int hils_port_id,
  const unsigned char i2c_addr,
  OnBoardComputer* obc,
  HilsPortManager* hils_port_manager
):GyroSensor(gyro), I2cTargetCommunicationWithObc(sils_port_id, hils_port_id, i2c_addr, obc, hils_port_manager)
{
  unsigned char tmp = 0xff;
  WriteRegister(kCmdGyroEnable_, &tmp, 1); // 初期値としてはGyro OFF

  SetConvertCoefficients();
}

void MPU9250_GYRO::MainRoutine(int count)
{
  UNUSED(count);
  // Read Registers
  ReadCmdGyroEnable();
  ReadCmdMagEnable();

  // Generate TLM
  if (is_gyro_on_ == true)
  {
    angular_velocity_c_rad_s_ = quaternion_b2c_.FrameConversion(dynamics_->GetAttitude().GetAngularVelocity_b_rad_s()); //Convert frame
    angular_velocity_c_rad_s_ = Measure(angular_velocity_c_rad_s_); //Add noises
    WriteGyroTlm();
  }

  int cmd_size = ReceiveCommand();
  if (cmd_size != 1) return; // length == 1 means setting of read register address
  // これ以降はHILS用に事前にテレメトリを溜めておく
  const int kTlmSize = 14;
  StoreTelemetry(kStoredFrameSize, kTlmSize);
  return;
}

void MPU9250_GYRO::ReadCmdGyroEnable()
{
  unsigned char tmp = 0xff;
  ReadRegister(kCmdGyroEnable_, &tmp, 1);
  if (tmp == 0x00) is_gyro_on_ = true;

  return;
}

void MPU9250_GYRO::ReadCmdMagEnable()
{
  unsigned char tmp[2] = {0xff, 0xff};
  ReadCommand(tmp, 2);
  if (tmp[0] != kCmdMagEnable_) return;

  if (tmp[1] == 0x02) is_mag_on_ = true; // 0x02 means turn on mag

  return;
}

void MPU9250_GYRO::ReadCmdGyroLpf()
{
  // TODO 6Uでの利用では固定値なので、中身の実装は優先度低
  return;
}

void MPU9250_GYRO::ReadCmdGyroRange()
{
  // TODO 6Uでの利用では固定値なので、中身の実装は優先度低
  return;
}

void MPU9250_GYRO::ReadCmdAccLpf()
{
  // TODO 6Uでの利用では固定値なので、中身の実装は優先度低
  return;
}

void MPU9250_GYRO::ReadCmdAccRange()
{
  // TODO 6Uでの利用では固定値なので、中身の実装は優先度低
  return;
}

void MPU9250_GYRO::WriteGyroTlm()
{
  unsigned char tlm[kMpuTlmSize_] = {0, 0}; 
  unsigned char reg_id = kRegObsGyro_;

  // Acc
  for (size_t i=0; i<kGyroDimension; i++)
  {
    Convert2Tlm(tlm, acc_c_G_[i] * acc_convert_G_to_raw_);
    WriteRegister(reg_id, tlm, kMpuTlmSize_);
    reg_id += kMpuTlmSize_;
  }
  // Temperature
  Convert2Tlm(tlm, (temperature_degC_ - temp_offset_degC_) * temp_convert_degC_to_raw_);
  WriteRegister(reg_id, tlm, kMpuTlmSize_);
  reg_id += kMpuTlmSize_;
  // Gyro
  for (size_t i=0; i<kGyroDimension; i++)
  {
    double omega_c_deg_s = angular_velocity_c_rad_s_[i] * libra::rad_to_deg;
    Convert2Tlm(tlm, omega_c_deg_s * omega_convert_deg_s_to_raw_);
    WriteRegister(reg_id, tlm, kMpuTlmSize_);
    reg_id += kMpuTlmSize_;
  }

  return;
}

void MPU9250_GYRO::Convert2Tlm(unsigned char tlm[kMpuTlmSize_], const double value)
{
  signed short tlm_s = (signed short)(value);

  for (int i=0; i < kMpuTlmSize_; i++)
  {
    tlm[i] = (unsigned char)(tlm_s >> 8 * (1-i));
  }
 
  return;
}

void MPU9250_GYRO::SetConvertCoefficients()
{
  omega_convert_deg_s_to_raw_ = raw_max_/omega_max_deg_s_;
  acc_convert_G_to_raw_ = raw_max_/acc_max_G_;
  return;
}
