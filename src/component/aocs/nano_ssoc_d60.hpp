/**
 * @file nano_ssoc_d60.hpp
 * @brief Class to emulate NanoSSOC D60 sun sensor
 * @note Manual: NA
 */

#ifndef S2E_AOBC_COMPONENT_AOCS_NANO_SSOC_D60_HPP_
#define S2E_AOBC_COMPONENT_AOCS_NANO_SSOC_D60_HPP_

#include <components/base/i2c_target_communication_with_obc.hpp>
#include <components/real/aocs/sun_sensor.hpp>

/**
 * @class NanoSsocD60
 * @brief Class to emulate NanoSSOC D60 sun sensor
 */
class NanoSsocD60 : public s2e::components::SunSensor, public s2e::components::I2cTargetCommunicationWithObc {
 public:
  /**
   * @fn NanoSsocD60
   * @brief Constructor
   * @param [in] sun_sensor: Sun sensor settings
   * @param [in] sils_port_id: Port ID for SILS
   * @param [in] hils_port_id: Port ID for HILS
   * @param [in] i2c_address: I2C address
   * @param [in] obc: Connected OBC
   * @param [in] hils_port_manager: HILS port manager
   */
  NanoSsocD60(s2e::components::SunSensor sun_sensor, const int sils_port_id, const unsigned int hils_port_id, const unsigned char i2c_address, s2e::components::OnBoardComputer *obc,
              s2e::simulation::HilsPortManager *hils_port_manager);
  /**
   * @fn ~NanoSsocD60
   * @brief Destructor
   */
  ~NanoSsocD60();

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
  const unsigned char i2c_address_;         //!< I2C Address
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
   * @param [in] angle_rad: Measured angle [rad]
   */
  int32_t ConvertAngle2Tlm(double angle_rad);
  /**
   * @fn GenerateErrorCode
   * @brief Generate error code
   */
  unsigned char GenerateErrorCode();
};

#endif  // S2E_AOBC_COMPONENT_AOCS_NANO_SSOC_D60_HPP_
