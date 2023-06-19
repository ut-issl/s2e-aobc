#ifndef S2E_AOBC_INTERFACE_HILS_HILS_IF_DRIVER_HPP_
#define S2E_AOBC_INTERFACE_HILS_HILS_IF_DRIVER_HPP_

#include <components/base/component.hpp>
#include <components/base/uart_communication_with_obc.hpp>

#include "../../component/aocs/mtq_seiren.hpp"

class HilsIfDriver : public Component, public UartCommunicationWithObc, public GpioConnectionWithObc {
 public:
  HilsIfDriver(const int prescaler, ClockGenerator *clock_gen, const unsigned int hils_port_id, const unsigned int baud_rate,
               HilsPortManager *hils_port_manager, std::vector<int> gpio_ports, OnBoardComputer *obc);
  ~HilsIfDriver();

  void MainRoutine(int count) override;

 protected:
  int ParseCommand(const int cmd_size) override;
  int GenerateTelemetry() override;
  void ControlGpio();

  static const uint8_t kRxMaxBytes_ = 6;
  static const uint8_t kNumOfMtqGpio_ = 6;
  bool is_high_mtq_[kNumOfMtqGpio_];
};
#endif  // S2E_AOBC_INTERFACE_HILS_HILS_IF_DRIVER_HPP_
