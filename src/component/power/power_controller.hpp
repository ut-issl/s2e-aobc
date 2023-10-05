/**
 * @file power_controller.hpp
 * @brief Class to emulate power controller
 * @note In the AOBC board, the power controller is just a GPIO definition in the PIC. There is no real H/W.
 */

#ifndef S2E_AOBC_COMPONENT_POWER_POWER_CONTROLLER_HPP_
#define S2E_AOBC_COMPONENT_POWER_POWER_CONTROLLER_HPP_

#include <components/base/gpio_connection_with_obc.hpp>
#include <components/real/power/power_control_unit.hpp>

/**
 * @class PowerController
 * @brief Class to emulate power controller
 */
class PowerController : public PowerControlUnit, public GpioConnectionWithObc {
 public:
  /**
   * @fn PowerController
   * @brief Constructor
   * @param [in] pcu: Power Control Unit
   * @param [in] gpio_ports: GPIO port number list
   * @param [in] output_voltage_list: Output voltage list
   * @param [in] obc: Connected OBC information
   */
  PowerController(PowerControlUnit pcu, const std::vector<int> gpio_ports, const std::vector<double> output_voltage_list, OnBoardComputer *obc);

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine for sensor observation
   */
  void MainRoutine(int count) override;

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  std::string GetLogHeader() const override;

 private:
  std::vector<double> output_voltage_list_;  //!< Output voltage list
};

#endif  // S2E_AOBC_COMPONENT_POWER_POWER_CONTROLLER_HPP_
