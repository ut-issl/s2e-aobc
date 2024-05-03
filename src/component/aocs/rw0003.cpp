/**
 * @file rw0003.cpp
 * @brief Class to emulate RW0.003 reaction wheel
 */

#include "rw0003.hpp"

#include <math.h>

#include <utilities/macros.hpp>
#include <utilities/slip.hpp>

typedef struct {
  uint8_t is_initialized;
  uint8_t mode;
  uint8_t _padding[2];
  float torque;
} InputReport;

extern "C" {
  uint32_t PICOCOMPO_rw0003_open(const char *name);
  int32_t PICOCOMPO_rw0003_send(uint32_t handle, float temperature, float speed);
  int32_t PICOCOMPO_rw0003_recv(uint32_t handle, InputReport *input);
}

Rw0003::Rw0003(ReactionWheel rw, const int sils_port_id, const unsigned int hils_port_id, const unsigned char i2c_address, OnBoardComputer *obc,
               HilsPortManager *hils_port_manager)
    : ReactionWheel(rw), I2cTargetCommunicationWithObc(sils_port_id, hils_port_id, i2c_address, obc, hils_port_manager) {
  char buffer[20];
  sprintf(buffer, "Fake RW-0.003 0x%02X", i2c_address);
  picocompo_handle_ = PICOCOMPO_rw0003_open(buffer);
  printf("i2c_address %02x (%s) handle: %d\n", i2c_address, buffer, picocompo_handle_);
  Initialize();
}

void Rw0003::MainRoutine(const int time_count) {
  UNUSED(time_count);

  InputReport input;
  PICOCOMPO_rw0003_recv(picocompo_handle_, &input);
  is_rw_initialized_ = input.is_initialized;
  SetDriveFlag(input.mode != 0);
  SetTargetTorque_rw_Nm(input.torque);

  // Generate TLM
  if (is_rw_initialized_ == true) {
    CalcTorque();
    PICOCOMPO_rw0003_send(picocompo_handle_, (float)temperature_degC_, (float)angular_velocity_rad_s_);
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
  PICOCOMPO_rw0003_send(picocompo_handle_, 0, 0);
}
