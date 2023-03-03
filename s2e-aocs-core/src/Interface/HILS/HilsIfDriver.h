#pragma once
#include "ComponentBase.h"
#include "ObcCommunicationBase.h"
#include "../../Component/AOCS/MTQseiren.h"

class HilsIfDriver: public ComponentBase, public ObcCommunicationBase, public ObcGpioBase
{
public:
  HilsIfDriver(
    const int prescaler,
    ClockGenerator *clock_gen,
    const unsigned int hils_port_id,
    const unsigned int baud_rate,
    HilsPortManager* hils_port_manager,
    vector<int> gpio_ports,
    OBC* obc
  );
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