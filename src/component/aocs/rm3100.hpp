/**
 * @file rm3100.hpp
 * @brief Class to emulate RM3100 magnetometer
 * @note Manual: https://www.pnicorp.com/wp-content/uploads/RM3100-Testing-Boards-User-Manual-r04-1.pdf
 *       Functions not used in the project are not implemented
 */

#ifndef S2E_AOBC_COMPONENT_AOCS_RM3100_HPP_
#define S2E_AOBC_COMPONENT_AOCS_RM3100_HPP_

#include <components/base/i2c_target_communication_with_obc.hpp>
#include <components/real/aocs/magnetometer.hpp>

/**
 * @class Rm3100
 * @brief Class to emulate RM3100 magnetometer
 */
class Rm3100 : public s2e::components::Magnetometer, public s2e::components::I2cTargetCommunicationWithObc {
 public:
  /**
   * @fn Rm3100
   * @brief Constructor
   * @param [in] magnetometer: Magnetometer settings
   * @param [in] sils_port_id: Port ID for SILS
   * @param [in] hils_port_id: Port ID for HILS
   * @param [in] i2c_address: I2C address
   * @param [in] obc: Connected OBC
   * @param [in] hils_port_manager: HILS port manager
   */
  Rm3100(s2e::components::Magnetometer magnetometer, const int sils_port_id, const unsigned int hils_port_id, const unsigned char i2c_address,
         s2e::components::OnBoardComputer *obc, s2e::simulation::HilsPortManager *hils_port_manager);

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine for sensor observation
   */
  void MainRoutine(const int time_count) override;
  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  std::string GetLogHeader() const override;

 private:
  // Mode setting
  unsigned char mode_ = 1;                      //!< Measurement mode 0: CMM, others: Polling
  const unsigned char kModeSetRegId = 0x01;     //!< Register address of mode setting
  const unsigned char kCmmModeSettings = 0x7d;  //!< CMM mode setting command

  const double kLsb2nT_ = 13.0;             //!< Convert coefficient raw to nT
  const unsigned int kStoredFrameSize = 3;  //!< Stored frame size for HILS

  /**
   * @fn ConvertMag2Tlm
   * @brief Convert measured magnetic field scalar value to telemetry format
   * @param [in] mag: Measured magnetic field value
   * return Converted telemetry data
   */
  int32_t ConvertMag2Tlm(double mag);
  /**
   * @fn GenerateTelemetry
   * @brief Generate telemetry
   * return Telemetry size
   */
  int GenerateTelemetry();
};

#endif  // S2E_AOBC_COMPONENT_AOCS_RM3100_HPP_
