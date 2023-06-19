#pragma once
#include <components/base/i2c_target_communication_with_obc.hpp>
#include <components/real/aocs/magnetometer.hpp>

/* References
Manual: https://strawberry-linux.com/pub/AK8963.pdf
HowToUse: 無し
Note: Functions not used in the project are not implemented
*/

// Magnetometer
class MPU9250_MAG : public Magnetometer, public I2cTargetCommunicationWithObc {
 public:
  MPU9250_MAG(Magnetometer mag_sensor, const int sils_port_id, const unsigned int hils_port_id, const unsigned char i2c_addr, OnBoardComputer *obc,
              HilsPortManager *hils_port_manager, const bool *is_mag_on);

  // Override: MagSensor functions
  void MainRoutine(int count) override;

 private:
  // Status
  const bool *is_mag_on_;        // mag_onはMPU9250_GYROが管理している
  unsigned char config_ = 0x00;  // config
  unsigned char status_ = 0;     // TODO: ちゃんと実装する？今の所この情報は使っていないみたい

  // TlmCmd parameters
  const unsigned char kTlmSize_ = 2;
  const double raw_max_ = pow(2, 8 * kTlmSize_ - 1);
  // Max
  // TODO:本当はSensorBaseの最大値と関連付けたいが、今はPrivate変数になっているので、ひとまずこれで
  const double mag_max_uT_ = 4800.0;
  // Conversion coefficients
  const double mag_convert_uT_to_raw_ = raw_max_ / mag_max_uT_;

  // Registers
  const unsigned char kRegObsMag_ = 0x02;

  // Comands
  const unsigned char kCmdMagConfig_ = 0x0a;

  // HILS
  const unsigned int kStoredFrameSize = 3;

  // Cmd
  void ReadCmdConfig();

  // TLM
  void WriteMagTlm();

  void SetConvertCoefficients();
  void Convert2Tlm(unsigned char tlm[2], const double value);
};
