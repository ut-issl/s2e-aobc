#include "cube_wheel.h"

CubeWheel::CubeWheel(RWModel rw_model, const int port_id, const unsigned char i2c_address, OBC* obc)
    : RWModel(rw_model), ObcI2cCommunicationBase(port_id, i2c_address, obc) {
  port_id_ = i2c_address - 0x67;  // port_idは1固定のため。

  unsigned char data[] = {0x08};

  if (i2c_address == 0x68) {
    WriteRegister(0x80, data, 1);
    data[0] = 0xd0;
    WriteRegister(0x83, data, 1);
  } else if (i2c_address == 0x69) {
    WriteRegister(0x80, data, 1);
    data[0] = 0xd2;
    WriteRegister(0x83, data, 1);
  } else if (i2c_address == 0x6A) {
    WriteRegister(0x80, data, 1);
    data[0] = 0xd4;
    WriteRegister(0x83, data, 1);
  }
}

void CubeWheel::MainRoutine(const int time_count) {
  // Control Modeの取得と処理
  ReadRegister(kWriteCmdControlMode_, &control_mode_, 1);

  if (control_mode_ == 2) {  // Duty cycle input mode
    SetDriveFlag(true);

    // 指令dutyを取得。
    unsigned char dutyH = 0;
    unsigned char dutyL = 0;
    ReadRegister(0x03, &dutyH, 1);  // 実際はアドレス3に2byte書かれるが、SILS上はC2A側でアドレス3,4に書く。
    ReadRegister(0x04, &dutyL, 1);
    double duty = (double)((int16_t)((uint16_t)dutyH + ((uint16_t)dutyL << 8)));

    // 指令トルクをセット。
    double torque_Nm = 0.00023 * duty / 100.0;

    SetTargetTorqueRw(torque_Nm);
  } else if (control_mode_ == 3) {  // Speed controller mode
    SetDriveFlag(true);

    // 指令speedを取得。
    unsigned char speedH = 0;
    unsigned char speedL = 0;
    ReadRegister(0x05, &speedH, 1);  // 実際はアドレス2に2byte書かれるが、SILS上はC2A側でアドレス5,6に書く。
    ReadRegister(0x06, &speedL, 1);

    if (speedH != 0xff && speedL != 0xff) {
      double speed_raw = (double)((int16_t)((uint16_t)speedH + ((uint16_t)speedL << 8)));
      double commanded_speed_rpm = speed_raw / 2;
      double current_speed_rpm = GetVelocityRpm();
      double to_add_rpm = abs(commanded_speed_rpm - current_speed_rpm) / 2;
      double to_add_rad_s = to_add_rpm * 0.1047;
      double sign = 1;
      if (commanded_speed_rpm < current_speed_rpm) {
        sign = -1;
      }
      double speed_command_cycle_in_sec = 0.5;
      double target_omega_dash = to_add_rad_s / speed_command_cycle_in_sec * sign;

      // 指令トルクをセット。
      double torque_Nm = target_omega_dash * 0.00000211;
      SetTargetTorqueRw(torque_Nm);

      // dummy値を書き込む。次にspeed cmdを受信するまで処理をしないため。
      unsigned char dummy[] = {0xff, 0xff};
      WriteRegister(0x05, dummy, 2);
      ReadRegister(0x05, &speedH, 1);  // 実際はアドレス2に2byte書かれるが、SILS上はC2A側でアドレス5,6に書く。
      ReadRegister(0x06, &speedL, 1);
    }
  }

  // Backup Modeの取得と処理。
  ReadRegister(kWriteCmdBackupMode_, &backup_mode_state_, 1);

  GenerateTelemetry();

  CalcTorque();

  return;
}

std::string CubeWheel::GetLogHeader() const {
  std::string str_tmp = "";
  const std::string st_id = std::to_string(static_cast<long long>(port_id_));

  str_tmp += WriteScalar("cubewheel_angular_velocity" + st_id, "rad/s");
  str_tmp += WriteScalar("cubewheel_angular_velocity_rpm" + st_id, "rpm");
  str_tmp += WriteScalar("cubewheel_angular_velocity_upperlimit" + st_id, "rpm");
  str_tmp += WriteScalar("cubewheel_angular_acceleration" + st_id, "rad/s^2");

  if (is_logged_jitter_) {
    str_tmp += WriteVector("cubewheel_jitter_force" + st_id, "c", "N", 3);
    str_tmp += WriteVector("cubewheel_jitter_torque" + st_id, "c", "Nm", 3);
  }

  return str_tmp;
}

int CubeWheel::GenerateTelemetry() {
  unsigned char data[] = {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  };

  // state
  unsigned char state = 0;
  if (backup_mode_state_) {
    state += (1 << 0);
  }
  if (motor_state_) {
    state += (1 << 1);
  }
  if (hall_sensor_state_) {
    state += (1 << 2);
  }
  if (encoder_state_) {
    state += (1 << 3);
  }
  if (error_flag_) {
    state += (1 << 4);
  }

  data[6] = control_mode_;
  data[7] = state;
  WriteRegister(kReadAddressWheelStatus_, data, 8);

  // speed in rpm
  double rpm = GetVelocityRpm();
  int16_t val_speed_rpm = (int16_t)(rpm * 2);
  data[1] = (unsigned char)(val_speed_rpm >> 8);
  data[0] = (unsigned char)(val_speed_rpm);
  WriteRegister(kReadAddressWheelData_, data, 6);

  return 0;
}
