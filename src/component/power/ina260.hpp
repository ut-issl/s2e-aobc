/**
 * @file ina260.hpp
 * @brief Class to emulate INA260 current sensor
 * @note Manual: https://www.tij.co.jp/jp/lit/ds/symlink/ina260.pdf?ts=1624796495336&ref_url=https%253A%252F%252Fwww.google.com%252F
 *       Functions not used in the project are not implemented
 */

#ifndef S2E_AOBC_COMPONENT_POWER_INA260_HPP_
#define S2E_AOBC_COMPONENT_POWER_INA260_HPP_

#include <components/base/component.hpp>
#include <components/base/i2c_target_communication_with_obc.hpp>
#include <components/ports/power_port.hpp>

/**
 * @class INA260
 * @brief Class to emulate INA260 current sensor
 */
class INA260 : public Component, public I2cTargetCommunicationWithObc {
 public:
  // ina_power_port: Power port witch provides electrical power to INA260
  // observation_power_port: Power port witch is observed by INA260

  /**
   * @fn INA260
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_gen: Clock generator
   * @param [in] ina_power_port: Power port witch provides electrical power to INA260
   * @param [in] ina_minimum_voltage: Minimum voltage to drive INA260 [V]
   * @param [in] ina_assumed_power_consumption: Power consumption of INA260 [W]
   * @param [in] observation_power_port: Power port to measure the current, voltage by the INA260
   * @param [in] i2c_port_id: I2C port ID for the INA260
   * @param [in] i2c_addr: I2C address of the INA260
   * @param [in] obc: Connected OBC information
   */
  INA260(int prescaler, ClockGenerator *clock_gen, PowerPort *ina_power_port, const double ina_minimum_voltage,
         const double ina_assumed_power_consumption, PowerPort *observation_power_port, const int i2c_port_id, const unsigned char i2c_addr,
         OnBoardComputer *obc);
  /**
   * @fn INA260
   * @brief Copy constructor
   */
  INA260(INA260 &&obj) noexcept;

  /**
   * @fn ~INA260
   * @brief Destructor
   */
  ~INA260();

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine for sensor observation
   */
  void MainRoutine(const int time_count) override;

 private:
  PowerPort *observation_power_port_;  //!< Power port to measure the current, voltage by the INA260
  unsigned char mode_ = 1;             //!< 0: Continuous mode, others: triggered (Not emulated yet)
  double over_current_threshold_mA;    //!< Over current threshold [mA]

  /**
   * @enum INA260_REGISTER
   * @brief INA260 register address
   */
  enum class INA260_REGISTER : unsigned char {
    CURRENT = 0x01,
    VOLTAGE = 0x02,
  };

  /**
   * @enum INA260_CMD
   * @brief INA260 command ID
   */
  enum class INA260_CMD : unsigned char {
    CONFIG = 0x00,
    LIMIT_MASK = 0x06,
    LIMIT_VALUE = 0x07,
  };

  const double kConvertObservedValue = 1.25;  //!< Conversion value
  const double kConvertLimitValue = 0.8;      //!< Conversion limit value

  /**
   * @fn GenerateTelemetry
   * @brief Telemetry generation
   */
  void GenerateTelemetry();
  /**
   * @fn ReadConfigCmd
   * @brief Read configuration command
   */
  void ReadConfigCmd();
  /**
   * @fn ReadLimitMaskCmd
   * @brief Read limit mask command
   */
  void ReadLimitMaskCmd();
  /**
   * @fn ReadLimitValueCmd
   * @brief Read limit value command
   */
  void ReadLimitValueCmd();
};

#endif  // S2E_AOBC_COMPONENT_POWER_INA260_HPP_
