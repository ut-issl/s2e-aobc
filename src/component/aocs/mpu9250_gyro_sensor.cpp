/**
 * @file mpu9250_gyro_sensor.cpp
 * @brief Class to emulate gyro sensor in MPU9250 9 axis sensor
 */

#include "mpu9250_gyro_sensor.hpp"

#include <math_physics/math/constants.hpp>
#include <utilities/macros.hpp>

Mpu9250GyroSensor::Mpu9250GyroSensor(GyroSensor gyro, const int sils_port_id, const unsigned int hils_port_id, const unsigned char i2c_address,
                                     OnBoardComputer *obc, HilsPortManager *hils_port_manager)
    : GyroSensor(gyro), 
    I2cTargetCommunicationWithObc(sils_port_id, hils_port_id, i2c_address, obc, hils_port_manager),
    i2c_address_(i2c_address){
        picompo_handle_ = PICOMPO_open("FAKE_MPU9250_GYRO");
  printf("i2c_address %02x handle: %d\n", i2c_address, picompo_handle_);

  SetConvertCoefficients();
}

void Mpu9250GyroSensor::MainRoutine(const int time_count) {
  UNUSED(time_count);
  // Read Registers
  ReadCmdGyroEnable();
  ReadCmdMagEnable();

  // Generate TLM
  // if (is_gyro_on_ == true) {
    angular_velocity_c_rad_s_ = quaternion_b2c_.FrameConversion(dynamics_->GetAttitude().GetAngularVelocity_b_rad_s());  // Convert frame
    angular_velocity_c_rad_s_ = Measure(angular_velocity_c_rad_s_);                                                      // Add
                                                                                                                         // noises
    WriteGyroTlm();
  // }

  // int command_size = ReceiveCommand();
  // if (command_size != 1) return;  // length == 1 means setting of read register address
  // // これ以降はHILS用に事前にテレメトリを溜めておく
  // const int kTlmSize = 14;
  // StoreTelemetry(kStoredFrameSize, kTlmSize);
  return;
}

void Mpu9250GyroSensor::ReadCmdGyroEnable() {
  unsigned char tmp = 0xff;
  ReadRegister(kCmdGyroEnable_, &tmp, 1);
  if (tmp == 0x00) is_gyro_on_ = true;

  return;
}

void Mpu9250GyroSensor::ReadCmdMagEnable() {
  unsigned char tmp[2] = {0xff, 0xff};
  ReadCommand(tmp, 2);
  if (tmp[0] != kCmdMagEnable_) return;

  if (tmp[1] == 0x02) is_magnetometer_on_ = true;  // 0x02 means turn on mag

  return;
}

void Mpu9250GyroSensor::ReadCmdGyroLpf() {
  // TODO 6Uでの利用では固定値なので、中身の実装は優先度低
  return;
}

void Mpu9250GyroSensor::ReadCmdGyroRange() {
  // TODO 6Uでの利用では固定値なので、中身の実装は優先度低
  return;
}

void Mpu9250GyroSensor::ReadCmdAccelerometerLpf() {
  // TODO 6Uでの利用では固定値なので、中身の実装は優先度低
  return;
}

void Mpu9250GyroSensor::ReadCmdAccelerometerRange() {
  // TODO 6Uでの利用では固定値なので、中身の実装は優先度低
  return;
}

void Mpu9250GyroSensor::WriteGyroTlm() {
  unsigned char tlm[kMpuTlmSize_] = {0, 0};
  unsigned char reg_id = kRegObsGyro_;
  
  Mpu9250GyroOutputReport output = {0};

  // Accelerometer
  for (size_t i = 0; i < kGyroDimension; i++) {
    Convert2Tlm(tlm, accelerometer_c_G_[i] * accelerometer_convert_G_to_raw_);
    if (i == 0) {
      output.accel_x_h = tlm[0];
      output.accel_x_l = tlm[1];
    }else if (i == 1) {
      output.accel_y_h = tlm[0];
      output.accel_y_l = tlm[1];
    }else if (i == 2) {
      output.accel_z_h = tlm[0];
      output.accel_z_l = tlm[1];
    }
    // WriteRegister(reg_id, tlm, kMpuTlmSize_);
    // reg_id += kMpuTlmSize_;
  }

  // Temperature
  Convert2Tlm(tlm, (temperature_degC_ - temperature_offset_degC_) * temperature_convert_degC_to_raw_);
  // WriteRegister(reg_id, tlm, kMpuTlmSize_);
  // reg_id += kMpuTlmSize_;
  output.temp_h = tlm[0];
  output.temp_l = tlm[1];

  // Gyro
  for (size_t i = 0; i < kGyroDimension; i++) {
    double angular_velocity_c_deg_s = angular_velocity_c_rad_s_[i] * libra::rad_to_deg;
    Convert2Tlm(tlm, angular_velocity_c_deg_s * angular_velocity_convert_deg_s_to_raw_);
    if (i==0) {
      output.gyro_x_h = tlm[0];
      output.gyro_x_l = tlm[1];
    }else if (i==1) {
      output.gyro_y_h = tlm[0];
      output.gyro_y_l = tlm[1];
    }else if (i==2) {
      output.gyro_z_h = tlm[0];
      output.gyro_z_l = tlm[1];
    }
    // WriteRegister(reg_id, tlm, kMpuTlmSize_);
    // reg_id += kMpuTlmSize_;
  }
  PICOMPO_send(picompo_handle_, reinterpret_cast<uint8_t *>(&output), sizeof(output));



  return;
}

void Mpu9250GyroSensor::Convert2Tlm(unsigned char tlm[kMpuTlmSize_], const double value) {
  signed short tlm_s = (signed short)(value);

  for (int i = 0; i < kMpuTlmSize_; i++) {
    tlm[i] = (unsigned char)(tlm_s >> 8 * (1 - i));
  }

  return;
}

void Mpu9250GyroSensor::SetConvertCoefficients() {
  angular_velocity_convert_deg_s_to_raw_ = raw_max_ / angular_velocity_max_deg_s_;
  accelerometer_convert_G_to_raw_ = raw_max_ / accelerometer_max_G_;
  return;
}
