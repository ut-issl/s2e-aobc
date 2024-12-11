/**
 * @file mpu9250_mag.cpp
 * @brief Class to emulate magnetometer in MPU9250 9 axis sensor
 */

#include "mpu9250_magnetometer.hpp"

#include <utilities/macros.hpp>

Mpu9250Magnetometer::Mpu9250Magnetometer(s2e::components::Magnetometer magnetometer, const int sils_port_id, const unsigned int hils_port_id,
                                         const unsigned char i2c_address, s2e::components::OnBoardComputer *obc, s2e::simulation::HilsPortManager *hils_port_manager,
                                         const bool *is_mag_on)
    : s2e::components::Magnetometer(magnetometer),
      s2e::components::I2cTargetCommunicationWithObc(sils_port_id, hils_port_id, i2c_address, obc, hils_port_manager),
      is_magnetometer_on_(is_mag_on) {}

void Mpu9250Magnetometer::MainRoutine(const int time_count) {
  UNUSED(time_count);
  // Read Registers
  ReadCmdConfig();

  // Generate TLM
  if (*is_magnetometer_on_ == true && configuration_ == 0x16)  // Power ON and 100Hz Continuous Measurement Mode
  {
    magnetic_field_c_nT_ = quaternion_b2c_.FrameConversion(geomagnetic_field_->GetGeomagneticField_b_nT());  // Convert frame
    magnetic_field_c_nT_ = Measure(magnetic_field_c_nT_);                                                    // Add noises
    WriteMagTlm();
  }

  int command_size = ReceiveCommand();
  if (command_size != 1) return;  // length == 1 means setting of read register address
  // これ以降はHILS用に事前にテレメトリを溜めておく
  const int kTlmSize = 8;
  StoreTelemetry(kStoredFrameSize, kTlmSize);
  return;
}

void Mpu9250Magnetometer::ReadCmdConfig() {
  unsigned char tmp[2] = {0xff, 0xff};
  ReadCommand(tmp, 2);
  if (tmp[0] != kCmdMagConfig_) return;

  configuration_ = tmp[1];

  return;
}

void Mpu9250Magnetometer::WriteMagTlm() {
  unsigned char tlm[] = {0, 0};
  unsigned char reg_id = kRegObsMag_;

  // Status
  WriteRegister(reg_id, &status_, 1);
  reg_id++;

  // MAG
  for (size_t i = 0; i < s2e::components::kMagnetometerDimension; i++) {
    double mag_c_uT = magnetic_field_c_nT_[i] / 1000.0;
    Convert2Tlm(tlm, mag_c_uT * magnetometer_convert_uT_to_raw_);
    WriteRegister(reg_id, tlm, kTlmSize_);
    reg_id += kTlmSize_;
  }

  return;
}

void Mpu9250Magnetometer::Convert2Tlm(unsigned char tlm[2], const double value) {
  signed short tlm_s = (signed short)(value);
  memcpy(tlm, &tlm_s, 2);

  return;
}
