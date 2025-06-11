/**
 * @file rw0003.cpp
 * @brief Class to emulate RW0.003 reaction wheel
 */

#include "rw0003.hpp"

#include <math.h>

#include <utilities/macros.hpp>
#include <utilities/slip.hpp>

Rw0003::Rw0003(ReactionWheel rw, const int sils_port_id, const unsigned int hils_port_id, const unsigned char i2c_address, OnBoardComputer *obc,
               HilsPortManager *hils_port_manager)
    : ReactionWheel(rw), I2cTargetCommunicationWithObc(sils_port_id, hils_port_id, i2c_address, obc, hils_port_manager) {
  char buffer[20];
  sprintf(buffer, "FAKE_RW-0.003_0X%02X", i2c_address);
  picompo_handle_ = PICOMPO_open(buffer);
  printf("i2c_address %02x (%s) handle: %d\n", i2c_address, buffer, picompo_handle_);
  Initialize();
}void Rw0003::MainRoutine(const int time_count) {
  UNUSED(time_count);

  InputReport input = {0};
  PICOMPO_recv(picompo_handle_, 1, reinterpret_cast<uint8_t*>(&input), sizeof(input));
  is_rw_initialized_ = input.is_initialized;
  printf("initialized(%d), handle(%d)\n", is_rw_initialized_, picompo_handle_);
  SetDriveFlag(input.mode != 0);
  printf("input_touque(%d), handle(%d)\n", input.torque, picompo_handle_);
  SetTargetTorque_rw_Nm((double)input.torque);

  // Generate TLM
  if (is_rw_initialized_ == true) {
    CalcTorque();
    OutputReport output;
    output.temperature = (float)temperature_degC_;
    output.speed = (float)angular_velocity_rad_s_;
    int32_t ret = PICOMPO_send(picompo_handle_, 1, reinterpret_cast<uint8_t*>(&output), sizeof(output));
    printf("speed(%f), handle(%d) = %d\n", output.speed, picompo_handle_, ret);
  }

  return;
}

std::string Rw0003::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar("RW0003_angular_velocity", "rad/s");
  str_tmp += WriteScalar("RW0003_angular_velocity", "rpm");
  str_tmp += WriteScalar("RW0003_angular_velocity_upper_limit", "rpm");
  str_tmp += WriteScalar("RW0003_target_angular_acceleration", "rad/s2");
  str_tmp += WriteScalar("RW0003_angular_acceleration", "rad/s^2");

  if (is_logged_jitter_) {
    str_tmp += WriteVector("RW0003_jitter_force", "c", "N", 3);
    str_tmp += WriteVector("RW0003_jitter_torque", "c", "Nm", 3);
  }

  return str_tmp;
}

void Rw0003::Initialize() {
  // HILS用の初期値を設定
  OutputReport output;
  output.temperature = 30;
  output.speed = 0;
  PICOMPO_send(picompo_handle_, 1, reinterpret_cast<uint8_t*>(&output), sizeof(output));
}
