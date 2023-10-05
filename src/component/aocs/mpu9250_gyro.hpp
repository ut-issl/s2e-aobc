/**
 * @file mpu9250_gyro.hpp
 * @brief Class to emulate gyro sensor in MPU9250 9 axis sensor
 * @note Manual: https://media.digikey.com/pdf/Data%20Sheets/TDK%20PDFs/MPU-9250_Rev_1.1.pdf
 *       Functions not used in the project are not implemented
 */

#ifndef S2E_AOBC_COMPONENT_AOCS_MPU9250_GYRO_HPP_
#define S2E_AOBC_COMPONENT_AOCS_MPU9250_GYRO_HPP_

#include <components/base/i2c_target_communication_with_obc.hpp>
#include <components/real/aocs/gyro_sensor.hpp>

const unsigned char kMpuTlmSize_ = 2;  //!< Telemetry size

/**
 * @class MPU9250_GYRO
 * @brief Class to emulate gyro sensor in MPU9250 9 axis sensor
 */
class MPU9250_GYRO : public GyroSensor, public I2cTargetCommunicationWithObc {
 public:
  /**
   * @fn MPU9250_GYRO
   * @brief Constructor
   * @param [in] gyro: Gyro sensor settings
   * @param [in] sils_port_id: Port ID for SILS
   * @param [in] hils_port_id: Port ID for HILS
   * @param [in] i2c_addr: I2C address
   * @param [in] obc: Connected OBC
   * @param [in] hils_port_manager: HILS port manager
   */
  MPU9250_GYRO(GyroSensor gyro, const int sils_port_id, const unsigned int hils_port_id, const unsigned char i2c_addr, OnBoardComputer *obc,
               HilsPortManager *hils_port_manager);

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine for sensor observation
   */
  void MainRoutine(int count) override;

  /**
   * @fn GetIsMagOn
   * @brief Return the internal magnetometer is ON or OFF
   */
  inline const bool *GetIsMagOn(void) const { return &is_mag_on_; }

 private:
  bool is_gyro_on_ = false;  //!< Gyro sensor enable status
  bool is_mag_on_ = false;   //!< Magnetometer enable status

  // TODO:本当はSensorBaseの最大値と関連付けたいが、今はPrivate変数になっているので、ひとまずこれで
  double omega_max_deg_s_ = 250.0;                       //!< Maximum angular rate measurement limit
  double acc_max_G_ = 2.0;                               //!< Maximum acceleration measurement limit
  const double raw_max_ = pow(2, 8 * kMpuTlmSize_ - 1);  //!< Maximum value of raw value

  // Conversion coefficients
  double omega_convert_deg_s_to_raw_;               //!< Conversion coefficients for angular rate [deg/s -> raw]
  double acc_convert_G_to_raw_;                     //!< Conversion coefficients for acceleration [G -> raw]
  const double temp_convert_degC_to_raw_ = 333.87;  //!< Conversion coefficients for temperature [degC -> raw]
  const double temp_offset_degC_ = 21.0;            //!< Temperature offset [degC]

  // Dummy data
  double acc_c_G_[kGyroDimension] = {-0.02, 0.01, -1.0};  //!< Observed acceleration [G]
  double temperature_degC_ = 30.0;                        //!< Observed temperature [degC]

  // Registers
  const unsigned char kRegObsGyro_ = 0x3b;  //!< Register address for gyro observation

  // Commands
  const unsigned char kCmdGyroEnable_ = 0x6b;  //!< Command to enable gyro sensor
  const unsigned char kCmdMagEnable_ = 0x37;   //!< Command to enable magnetometer
  const unsigned char kCmdGyroLPF_ = 0x1a;     //!< Command to set gyro sensor Low Pass Filter
  const unsigned char kCmdGyroRange_ = 0x1b;   //!< Command to set gyro sensor measurement range
  const unsigned char kCmdAccRange_ = 0x1c;    //!< Command to set accelerator measurement range
  const unsigned char kCmdAccLPF_ = 0x1d;      //!< Command to set accelerator Low Pass Filter

  // HILS
  const unsigned int kStoredFrameSize = 3;  //!< Stored frame size

  // Command
  /**
   * @fn ReadCmdGyroEnable
   * @brief Read and execute gyro enable command
   */
  void ReadCmdGyroEnable();
  /**
   * @fn ReadCmdMagEnable
   * @brief Read and execute magnetometer enable command
   */
  void ReadCmdMagEnable();
  /**
   * @fn ReadCmdGyroLpf
   * @brief Read and execute gyro low pass filter command
   * @note TODO: Not implemented yet
   */
  void ReadCmdGyroLpf();
  /**
   * @fn ReadCmdGyroRange
   * @brief Read and execute gyro measurement range command
   * @note TODO: Not implemented yet
   */
  void ReadCmdGyroRange();
  /**
   * @fn ReadCmdAccLpf
   * @brief Read and execute accelerator low pass filter command
   * @note TODO: Not implemented yet
   */
  void ReadCmdAccLpf();
  /**
   * @fn ReadCmdAccRange
   * @brief Read and execute accelerator range command
   * @note TODO: Not implemented yet
   */
  void ReadCmdAccRange();

  // Telemetry
  /**
   * @fn WriteGyroTlm
   * @brief Measure and write gyro sensor telemetry
   */
  void WriteGyroTlm();

  /**
   * @fn SetConvertCoefficients
   * @brief Set convert coefficients
   */
  void SetConvertCoefficients();
  /**
   * @fn Convert2Tlm
   * @brief Convert measured value to telemetry output
   * @param [out] tlm: Output telemetry
   * @param [in] value: Input value
   */
  void Convert2Tlm(unsigned char tlm[kMpuTlmSize_], const double value);
};

#endif  // S2E_AOBC_COMPONENT_AOCS_MPU9250_GYRO_HPP_
