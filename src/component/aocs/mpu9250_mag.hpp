/**
 * @file mpu9250_mag.hpp
 * @brief Class to emulate magnetometer in MPU9250 9 axis sensor
 * @note Manual: https://media.digikey.com/pdf/Data%20Sheets/TDK%20PDFs/MPU-9250_Rev_1.1.pdf
 *       Functions not used in the project are not implemented
 */

#ifndef S2E_AOBC_COMPONENT_AOCS_MPU9250_MAG_HPP_
#define S2E_AOBC_COMPONENT_AOCS_MPU9250_MAG_HPP_

#include <components/base/i2c_target_communication_with_obc.hpp>
#include <components/real/aocs/magnetometer.hpp>

/**
 * @class MPU9250_MAG
 * @brief Class to emulate magnetometer in MPU9250 9 axis sensor
 */
class MPU9250_MAG : public Magnetometer, public I2cTargetCommunicationWithObc {
 public:
  /**
   * @fn MPU9250_MAG
   * @brief Constructor
   * @param [in] magnetometer: Magnetometer settings
   * @param [in] sils_port_id: Port ID for SILS
   * @param [in] hils_port_id: Port ID for HILS
   * @param [in] i2c_address: I2C address
   * @param [in] obc: Connected OBC
   * @param [in] hils_port_manager: HILS port manager
   * @param [in] is_mag_on: Magnetometer ON/OFF flag
   */
  MPU9250_MAG(Magnetometer magnetometer, const int sils_port_id, const unsigned int hils_port_id, const unsigned char i2c_address, OnBoardComputer *obc,
              HilsPortManager *hils_port_manager, const bool *is_mag_on);

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine for sensor observation
   */
  void MainRoutine(const int time_count) override;

 private:
  // Status
  const bool *is_mag_on_;        //!< Magnetometer ON/OFF flag which is managed by MPU9250_GYRO
  unsigned char config_ = 0x00;  //!< Configuration infomation
  unsigned char status_ = 0;     //!< Status TODO: Implement. Currently, this is not used.

  // TlmCmd parameters
  const unsigned char kTlmSize_ = 2;                  //!< Telemetry size
  const double raw_max_ = pow(2, 8 * kTlmSize_ - 1);  //!< MAximum value of raw measurement value

  // TODO:本当はSensorBaseの最大値と関連付けたいが、今はPrivate変数になっているので、ひとまずこれで
  const double mag_max_uT_ = 4800.0;                             //!< Maximum observation value [uT]
  const double mag_convert_uT_to_raw_ = raw_max_ / mag_max_uT_;  //!< Conversion coefficients [uT -> raw]

  // Registers
  const unsigned char kRegObsMag_ = 0x02;  //!< Register address of magnetometer observation value

  // Commands
  const unsigned char kCmdMagConfig_ = 0x0a;  //!< Command ID to set magnetometer configuration

  // HILS
  const unsigned int kStoredFrameSize = 3;  //!< Stored frame size

  // Cmd
  /**
   * @fn ReadCmdConfig
   * @brief Read and execute configuration setting command
   */
  void ReadCmdConfig();

  // TLM
  /**
   * @fn WriteMagTlm
   * @brief Measure and write magnetometer telemetry
   */
  void WriteMagTlm();

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
  void Convert2Tlm(unsigned char tlm[2], const double value);
};

#endif  // S2E_AOBC_COMPONENT_AOCS_MPU9250_MAG_HPP_
