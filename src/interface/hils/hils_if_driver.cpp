#include "hils_if_driver.hpp"

HilsIfDriver::HilsIfDriver(const int prescaler, ClockGenerator *clock_gen, const unsigned int hils_port_id, const unsigned int baud_rate,
                           HilsPortManager *hils_port_manager, std::vector<int> gpio_ports, OnBoardComputer *obc)
    : Component(prescaler, clock_gen), UartCommunicationWithObc(hils_port_id, baud_rate, hils_port_manager), GpioConnectionWithObc(gpio_ports, obc) {}

HilsIfDriver::~HilsIfDriver() {}

void HilsIfDriver::MainRoutine(int count) {
  // IFボードからデータ取得
  int ret = ReceiveCommand(0, kRxMaxBytes_);
  if (ret != 0) {
    return;
  }

  // GPIO操作
  ControlGpio();

  // IFボードにGPIO状態送信
  GenerateTelemetry();

  return;
}

int HilsIfDriver::ParseCommand(const int cmd_size) {
  std::vector<unsigned char> cmd = rx_buffer_;
  int idx = 0;
  for (int i = 0; i < cmd_size; i++) {
    cmd[idx] = rx_buffer_[i];
    idx++;
  }

  if (cmd[0] != 0x49 || cmd[1] != 0x46) return -1;

  for (int i = 0; i < kNumOfMtqGpio_; i++) {
    is_high_mtq_[i] = (cmd[2] >> i) & 1;
  }
  return 0;
}

int HilsIfDriver::GenerateTelemetry() { return 0; }

void HilsIfDriver::ControlGpio() {
  for (int i = 0; i < kNumOfMtqGpio_; i++) {
    Write(i, is_high_mtq_[i]);
  }
}
