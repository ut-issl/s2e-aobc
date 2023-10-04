#ifndef S2E_AOBC_COMPONENT_AOCS_CUBEWHEEL_HPP_
#define S2E_AOBC_COMPONENT_AOCS_CUBEWHEEL_HPP_

#include "../../../s2e_core_oss/src/Component/AOCS/RWModel.h"
#include "../../../s2e_core_oss/src/Component/Abstract/ObcI2cCommunicationBase.h"

class CubeWheel : public RWModel, public ObcI2cCommunicationBase {
 public:
  CubeWheel(RWModel rw_model, const int port_id, const unsigned char i2c_addr, OBC* obc);

  // Override: RWModel functions
  void MainRoutine(int count) override;
  std::string GetLogHeader() const override;

 private:
  // RWレジスタ
  unsigned char control_mode_ = 0;
  unsigned char backup_mode_state_ = 0;
  unsigned char motor_state_ = 0;
  unsigned char hall_sensor_state_ = 0;
  unsigned char encoder_state_ = 0;
  unsigned char error_flag_ = 0;

  double lsb2nT_ = 13.0;
  int port_id_ = 0;

  // Write Command
  static const uint8_t kWriteCmdControlMode_ = 10;
  static const uint8_t kWriteCmdBackupMode_ = 12;

  // Register address
  static const uint8_t kReadAddressWheelStatus_ = 140;  // 本来は130
  static const uint8_t kReadAddressWheelData_ = 150;    // 本来は137

  static const uint8_t kReadAddressSpeed_ = 0x15;
  static const uint8_t kReadAddressLimitSpeed1_ = 0x33;
  static const uint8_t kReadAddressLimitSpeed2_ = 0x34;

  // TLM
  int32_t ConvertMag2Tlm(double mag);
  int GenerateTelemetry();
};
#endif  // S2E_AOBC_COMPONENT_AOCS_CUBEWHEEL_HPP_

