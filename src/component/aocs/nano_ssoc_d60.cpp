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
      i2c_address_(i2c_address) {
        char buffer[20];
        sprintf(buffer, "FAKE_NANOSSOC_0X%02X", i2c_address);
        picompo_handle_ = PICOMPO_open(buffer);
  printf("i2c_address %02x handle: %d\n", i2c_address, picompo_handle_);
}

NanoSsocD60::~NanoSsocD60() {}

void NanoSsocD60::MainRoutine(const int time_count) {
  UNUSED(time_count);
  Measure();
  sun_intensity_percent_ = solar_illuminance_W_m2_ / srp_environment_->GetSolarConstant_W_m2() * 100.0;

  GenerateTelemetry();

  /*
  int command_size = ReceiveCommand();
  if (command_size != 1) return;  // length == 1 means setting of read register address
  // これ以降はHILS用に事前にテレメトリを溜めておく
  const int kTlmSize = 15;
  StoreTelemetry(kStoredFrameSize, kTlmSize);
  */
  return;
}

int NanoSsocD60::GenerateTelemetry() {
  const int kTlmSize = 15;

  NanoSsocOutputReport output = {0};

  output.angular_position_angle_x = ConvertAngle2Tlm(alpha_rad_);
  output.angular_position_angle_y = ConvertAngle2Tlm(beta_rad_);
  output.angular_position_sun_detection = ConvertFloat2FloatingPoint(float(sun_intensity_percent_));
  output.angular_position_error_code = GenerateErrorCode();
  PICOMPO_send(picompo_handle_, 1, reinterpret_cast<uint8_t *>(&output), sizeof(output));

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