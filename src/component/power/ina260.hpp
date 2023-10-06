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
 * @class Ina260
 * @brief Class to emulate INA260 current sensor
 */
class Ina260 : public Component, public I2cTargetCommunicationWithObc {
 public:
  // ina_power_port: Power port witch provides electrical power to Ina260
  // observation_power_port: Power port witch is observed by Ina260

  /**
   * @fn Ina260
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] ina_power_port: Power port witch provides electrical power to Ina260
   * @param [in] ina_minimum_voltage: Minimum voltage to drive Ina260 [V]
   * @param [in] ina_assumed_power_consumption: Power consumption of Ina260 [W]
   * @param [in] observation_power_port: Power port to measure the current, voltage by the Ina260
   * @param [in] i2c_port_id: I2C port ID for the Ina260
   * @param [in] i2c_addr: I2C address of the Ina260
   * @param [in] obc: Connected OBC information
   */
  Ina260(int prescaler, ClockGenerator *clock_generator, PowerPort *ina_power_port, const double ina_minimum_voltage,
         const double ina_assumed_power_consumption, PowerPort *observation_power_port, const int i2c_port_id, const unsigned char i2c_addr,
         OnBoardComputer *obc);
  /**
   * @fn Ina260
   * @brief Copy constructor
   */
  Ina260(Ina260 &&obj) noexcept;

  /**
   * @fn ~Ina260
   * @brief Destructor
   */
  ~Ina260();

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine for sensor observation
   */
  void MainRoutine(const int time_count) override;

 private:
  PowerPort *observation_power_port_;  //!< Power port to measure the current, voltage by the Ina260
  unsigned char mode_ = 1;             //!< 0: Continuous mode, others: triggered (Not emulated yet)
  double over_current_threshold_mA;    //!< Over current threshold [mA]

  /**
   * @enum Ina260Register
   * @brief Ina260 register address
   */
  enum class Ina260Register : unsigned char {
    kCurrent = 0x01,
    kVoltage = 0x02,
  };

  /**
   * @enum Ina260Command
   * @brief Ina260 command ID
   */
  enum class Ina260Command : unsigned char {
    kConfig = 0x00,
    kLimitMask = 0x06,
    kLimitValue = 0x07,
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
