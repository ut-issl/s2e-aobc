/**
 * @file mtq_seiren.cpp
 * @brief Class to emulate MTQ developed by Seiren
 * @note Manual: NA
 */

#include "mtq_seiren.hpp"

#include <utilities/macros.hpp>

MtqSeiren::MtqSeiren(Magnetorquer magnetorquer, std::vector<int> gpio_ports, OnBoardComputer *obc)
    : Magnetorquer(magnetorquer), GpioConnectionWithObc(gpio_ports, obc) {}

MtqSeiren::~MtqSeiren() {}

void MtqSeiren::MainRoutine(const int time_count) {
  UNUSED(time_count);
  // Read GPIO state
  polarity_[0] = ConvertGPIOState2Polarity(GPIO_X_POSITIVE, GPIO_X_NEGATIVE);
  polarity_[1] = ConvertGPIOState2Polarity(GPIO_Y_POSITIVE, GPIO_Y_NEGATIVE);
  polarity_[2] = ConvertGPIOState2Polarity(GPIO_Z_POSITIVE, GPIO_Z_NEGATIVE);

  // Set the output magnetic moment to the PWM on/off
  ConvertPolarity2OutputMag();

  // Calculate magnetic moment and torque
  CalcOutputTorque();
}

int MtqSeiren::ConvertGPIOState2Polarity(int positive_gpio_pin_idx, int negative_gpio_pin_idx) {
  int polarity;

  bool positive_gpio_pin_state;  // true = HIGH, false = LOW
  bool negative_gpio_pin_state;  // true = HIGH, false = LOW

  positive_gpio_pin_state = Read(positive_gpio_pin_idx);
  negative_gpio_pin_state = Read(negative_gpio_pin_idx);

  if (positive_gpio_pin_state == true && negative_gpio_pin_state == false) {
    polarity = 1;
  } else if (positive_gpio_pin_state == false && negative_gpio_pin_state == true) {
    polarity = -1;
  } else {
    // (positive_gpio_pin_state == false && negative_gpio_pin_state == false :
    // MTQ output is off (positive_gpio_pin_state == true  &&
    // negative_gpio_pin_state == true  : inhibit input
    polarity = 0;
  }

  return polarity;
}

void MtqSeiren::ConvertPolarity2OutputMag() {
  for (size_t i = 0; i < kMtqDimension; i++) {
    if (polarity_[i] == 1) {
      output_magnetic_moment_c_Am2_[i] = max_magnetic_moment_c_Am2_[i];
    } else if (polarity_[i] == -1) {
      output_magnetic_moment_c_Am2_[i] = min_magnetic_moment_c_Am2_[i];
    } else {
      output_magnetic_moment_c_Am2_[i] = 0.0;
    }
  }
}

std::string MtqSeiren::GetLogHeader() const {
  std::string str_tmp = "";
  const std::string st_sensor_id = std::to_string(static_cast<long long>(component_id_));
  const char *cs = st_sensor_id.data();
  std::string section = "MTQ_seiren";

  str_tmp += WriteVector(section + cs, "b", "Am2", kMtqDimension);
  str_tmp += WriteVector(section + cs, "b", "Nm", kMtqDimension);

  return str_tmp;
}
