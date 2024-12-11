/**
 * @file hils_if_driver.hpp
 * @brief Interface driver to read GPIO information for HILS test
 */

#ifndef S2E_AOBC_INTERFACE_HILS_HILS_IF_DRIVER_HPP_
#define S2E_AOBC_INTERFACE_HILS_HILS_IF_DRIVER_HPP_

#include <components/base/component.hpp>
#include <components/base/uart_communication_with_obc.hpp>

#include "../../component/aocs/mtq_seiren.hpp"

/**
 * @class HilsIfDriver
 * @brief Interface driver to read GPIO information for HILS test
 * @note HILS configuration
 *       AOBC - MTQ communication port --GPIO-- PIC board --UART-- PC COM port - S2E
 */
class HilsIfDriver : public s2e::components::Component, public s2e::components::UartCommunicationWithObc, public s2e::components::GpioConnectionWithObc {
 public:
  /**
   * @fn HilsIfDriver
   * @brief Constructor
   * @param [in] prescaler: Prescaler
   * @param [in] clock_generator: Clock generator
   * @param [in] hils_port_id: HILS port ID
   * @param [in] baud_rate: HILS communication baud rate
   * @param [in] hils_port_manager: HILS port manager
   * @param [in] gpio_ports: GPIO port information
   * @param [in] obc: On Board Computer
   */
  HilsIfDriver(const int prescaler, s2e::environment::ClockGenerator *clock_generator, const unsigned int hils_port_id, const unsigned int baud_rate,
               s2e::simulation::HilsPortManager *hils_port_manager, std::vector<int> gpio_ports, s2e::components::OnBoardComputer *obc);
  /**
   * @fn ~HilsIfDriver
   * @brief Destructor
   */
  ~HilsIfDriver();

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine for sensor observation
   */
  void MainRoutine(const int time_count) override;

 protected:
  /**
   * @fn ParseCommand
   * @brief Override function for UART communication
   * @param [in] command_size: Command size
   */
  int ParseCommand(const int command_size) override;
  /**
   * @fn GenerateTelemetry
   * @brief Override function for UART communication
   */
  int GenerateTelemetry() override;

  /**
   * @fn ControlGpio
   * @brief GPIO controller
   */
  void ControlGpio();

  static const uint8_t kRxMaxBytes_ = 6;    //!< Receive max data size [byte]
  static const uint8_t kNumOfMtqGpio_ = 6;  //!< Number of GPIO port for MTQ
  bool is_high_mtq_[kNumOfMtqGpio_];        //!< MTQ GPIO states
};

#endif  // S2E_AOBC_INTERFACE_HILS_HILS_IF_DRIVER_HPP_
