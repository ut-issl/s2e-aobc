#pragma once
#include <components/real/aocs/gyro_sensor.hpp>
#include <components/base/i2c_target_communication_with_obc.hpp>

/* References
Manual: https://media.digikey.com/pdf/Data%20Sheets/TDK%20PDFs/MPU-9250_Rev_1.1.pdf
HowToUse: 無し
Note: Functions not used in the project are not implemented
*/

const unsigned char kMpuTlmSize_ = 2;

class MPU9250_GYRO: public GyroSensor, public I2cTargetCommunicationWithObc
{
public:
  MPU9250_GYRO(
    GyroSensor gyro,
    const int sils_port_id,
    const unsigned int hils_port_id,
    const unsigned char i2c_addr,
    OnBoardComputer* obc,
    HilsPortManager* hils_port_manager
  );

  // Override: MagSensor functions
  void MainRoutine(int count) override;

  inline const bool* GetIsMagOn(void) const {return &is_mag_on_;}

private:
  bool is_gyro_on_ = false;
  bool is_mag_on_ = false;

  // Max TODO:本当はSensorBaseの最大値と関連付けたいが、今はPrivate変数になっているので、ひとまずこれで
  double omega_max_deg_s_ = 250.0;
  double acc_max_G_ = 2.0;
  const double raw_max_ = pow(2, 8 * kMpuTlmSize_ - 1);

  // Conversion coefficients 温度計は固定値
  double omega_convert_deg_s_to_raw_;
  double acc_convert_G_to_raw_;
  const double temp_convert_degC_to_raw_  = 333.87;
  const double temp_offset_degC_          = 21.0;
  
  // Dummy data
  double acc_c_G_[kGyroDimension] = {-0.02, 0.01, -1.0}; // とりあえずテキトウな値を入れる
  double temperature_degC_ = 30.0; // 温度補正に合わせて設定

  // Registers
  const unsigned char kRegObsGyro_     = 0x3b;

  // Commands
  const unsigned char kCmdGyroEnable_  = 0x6b;
  const unsigned char kCmdMagEnable_   = 0x37;
  const unsigned char kCmdGyroLPF_     = 0x1a;
  const unsigned char kCmdGyroRange_   = 0x1b;
  const unsigned char kCmdAccRange_    = 0x1c;
  const unsigned char kCmdAccLPF_      = 0x1d;

  // HILS
  const unsigned int kStoredFrameSize = 3;

  // Cmd
  void ReadCmdGyroEnable();
  void ReadCmdMagEnable();
  void ReadCmdGyroLpf();   // TODO 6Uでの利用では固定値なので、中身の実装は優先度低
  void ReadCmdGyroRange(); // TODO 6Uでの利用では固定値なので、中身の実装は優先度低
  void ReadCmdAccLpf();    // TODO 6Uでの利用では固定値なので、中身の実装は優先度低
  void ReadCmdAccRange();  // TODO 6Uでの利用では固定値なので、中身の実装は優先度低
 
  // TLM
  void WriteGyroTlm();

  void SetConvertCoefficients();
  void Convert2Tlm(unsigned char tlm[kMpuTlmSize_], const double value);
};
