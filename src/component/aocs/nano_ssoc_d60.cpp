/**
 * @file nano_ssoc_d60.cpp
 * @brief Class to emulate NanoSSOC D60 sun sensor
 * @note Manual: NA
 */

#include "nano_ssoc_d60.hpp"
#define _USE_MATH_DEFINES
#include <string.h>  // for memcpy

#include <algorithm>
#include <math_physics/math/constants.hpp>
#include <utilities/macros.hpp>

NanoSsocD60::NanoSsocD60(SunSensor sun_sensor, const int sils_port_id, const unsigned int hils_port_id, const unsigned char i2c_address,
                         OnBoardComputer *obc, HilsPortManager *hils_port_manager)
    : SunSensor(sun_sensor),
      I2cTargetCommunicationWithObc(sils_port_id, hils_port_id, i2c_address, obc, hils_port_manager),
      i2c_address_(i2c_address) {}

NanoSsocD60::~NanoSsocD60() {}

void NanoSsocD60::MainRoutine(const int time_count) {
  UNUSED(time_count);
  Measure();
  sun_intensity_percent_ = solar_illuminance_W_m2_ / srp_environment_->GetSolarConstant_W_m2() * 100.0;

  GenerateTelemetry();

  int command_size = ReceiveCommand();
  if (command_size != 1) return;  // length == 1 means setting of read register address
  // これ以降はHILS用に事前にテレメトリを溜めておく
  const int kTlmSize = 15;
  StoreTelemetry(kStoredFrameSize, kTlmSize);
  return;
}

int NanoSsocD60::GenerateTelemetry() {
  const int kByte2Bit = 8;

  const int kTlmSize = 15;
  unsigned char tlm[kTlmSize] = {};
  tlm[0] = 0x0E;

  int32_t alpha_tlm = ConvertAngle2Tlm(alpha_rad_);
  for (int i = 0; i < 4; i++) {
    tlm[1 + i] = (alpha_tlm >> kByte2Bit * i) & 0xFF;
  }
  int32_t beta_tlm = ConvertAngle2Tlm(beta_rad_);
  for (int i = 0; i < 4; i++) {
    tlm[5 + i] = (beta_tlm >> kByte2Bit * i) & 0xFF;
  }
  int32_t sun_detection_rate_tlm = ConvertFloat2FloatingPoint(float(sun_intensity_percent_));
  for (int i = 0; i < 4; i++) {
    tlm[9 + i] = (sun_detection_rate_tlm >> kByte2Bit * i) & 0xFF;
  }

  tlm[13] = GenerateErrorCode();
  unsigned char check_sum = 0x00;
  for (int i = 0; i < 14; i++) {
    check_sum += tlm[i];
  }
  tlm[14] = check_sum & 0xff;

  WriteRegister(0x04, tlm, kTlmSize);
  return kTlmSize;
}

int32_t NanoSsocD60::ConvertFloat2FloatingPoint(float data) {
  int32_t internal_representation = *((int32_t *)&data);  // The internal representation of "data" in decimal notation.
  return internal_representation;
}

int32_t NanoSsocD60::ConvertAngle2Tlm(double angle_rad) {
  double angle_deg = angle_rad * libra::rad_to_deg;

  int32_t angle_tlm_data = ConvertFloat2FloatingPoint((float)angle_deg);
  return angle_tlm_data;
}

unsigned char NanoSsocD60::GenerateErrorCode() {
  unsigned char error_code;

  if (!sun_detected_flag_) {
    if (sun_intensity_percent_ < 80.0) {
      error_code = 0x0A;
    } else {
      error_code = 0x0D;
    }
  } else {
    if (sun_intensity_percent_ < 80.0) {
      error_code = 0x0B;
    } else if (sun_intensity_percent_ > 120.0) {
      error_code = 0x0C;
    } else {
      error_code = 0x00;
    }
  }

  return error_code;
}

std::string NanoSsocD60::GetLogHeader() const {
  std::string str_tmp = "";
  const std::string st_id = std::to_string(static_cast<long long>(component_id_));

  str_tmp += WriteVector("NanoSSOC_D60" + st_id, "c", "-", 3);
  str_tmp += WriteScalar("sun_detected_flag" + st_id, "-");

  return str_tmp;
}