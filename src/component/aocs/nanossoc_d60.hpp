/**
 * @file nanossoc_d60.hpp
 * @brief Class to emulate NanoSSOC D60 sun sensor
 * @note Manual: NA
 */

#ifndef S2E_AOBC_COMPONENT_AOCS_NANOSSOC_D60_HPP_
#define S2E_AOBC_COMPONENT_AOCS_NANOSSOC_D60_HPP_

#include <components/base/i2c_target_communication_with_obc.hpp>
#include <components/real/aocs/sun_sensor.hpp>

/**
 * @class NanoSSOCD60
 * @brief Class to emulate NanoSSOC D60 sun sensor
 */
class NanoSSOCD60 : public SunSensor, public I2cTargetCommunicationWithObc {
 public:
  /**
   * @fn NanoSSOCD60
   * @brief Constructor
   * @param [in] sun_sensor: Sun sensor settings
   * @param [in] sils_port_id: Port ID for SILS
   * @param [in] hils_port_id: Port ID for HILS
   * @param [in] i2c_address: I2C address
   * @param [in] obc: Connected OBC
   * @param [in] hils_port_manager: HILS port manager
   */
  NanoSSOCD60(SunSensor sun_sensor, const int sils_port_id, const unsigned int hils_port_id, const unsigned char i2c_address, OnBoardComputer *obc,
              HilsPortManager *hils_port_manager);
  /**
   * @fn ~NanoSSOCD60
   * @brief Destructor
   */
  ~NanoSSOCD60();

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
  const unsigned char i2c_addr_;            //!< I2C Address
  const unsigned int kStoredFrameSize = 3;  //!< Stored frame size

  double sun_intensity_percent_ = 0.0;  //!< Measured sun intensity [%]

  // General Functions
  /**
   * @fn ConvertFloat2FloatingPoint
   * @brief Function to convert a float value to its internal 32-bit representation
   * @param [in] data: input float value
   */
  int32_t ConvertFloat2FloatingPoint(float data);

  // Telemetry
  /**
   * @fn GenerateTelemetry
   * @brief Generate telemetry
   */
  int GenerateTelemetry();
  /**
   * @fn ConvertAngle2Tlm
   * @brief Convert measured angle to telemetry information
   * @param [in] angle: Measured angle [deg]
   */
  int32_t ConvertAngle2Tlm(double angle);
  /**
   * @fn GenerateErrorCode
   * @brief Generate error code
   */
  unsigned char GenerateErrorCode();
};

#endif  // S2E_AOBC_COMPONENT_AOCS_NANOSSOC_D60_HPP_
