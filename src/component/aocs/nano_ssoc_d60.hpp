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
class NanoSsocD60 : public SunSensor, public I2cTargetCommunicationWithObc {
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
  NanoSsocD60(SunSensor sun_sensor, const int sils_port_id, const unsigned int hils_port_id, const unsigned char i2c_address, OnBoardComputer *obc,
              HilsPortManager *hils_port_manager);
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

  // Picompo
  uint32_t picompo_handle_ = 0;

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

extern "C" {
uint32_t PICOMPO_open(const char *name);
int32_t PICOMPO_send(uint32_t handle, uint8_t ep, uint8_t *data, uint32_t data_len);
int32_t PICOMPO_recv(uint32_t handle, uint8_t ep, uint8_t *buf, uint32_t buf_len);
}

typedef struct {
  float unfiltered_cell_u_ssa1;
  float unfiltered_cell_u_ssa2;
  float unfiltered_cell_u_ssa3;
  float unfiltered_cell_u_ssa4;
  float filtered_cell_u_ssa1f;
  float filtered_cell_u_ssa2f;
  float filtered_cell_u_ssa3f;
  float filtered_cell_u_ssa4f;
  int32_t angular_position_angle_x;
  int32_t angular_position_angle_y;
  int32_t angular_position_sun_detection;
  uint8_t angular_position_error_code;
} NanoSsocOutputReport;

#endif  // S2E_AOBC_COMPONENT_AOCS_NANO_SSOC_D60_HPP_
