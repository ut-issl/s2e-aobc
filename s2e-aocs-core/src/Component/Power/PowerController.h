#pragma once
#include "PCU.h"
#include "ObcGpioBase.h"

class PowerController: public PCU, public ObcGpioBase
{
public:
  PowerController(
    PCU pcu,
    const std::vector<int> gpio_ports,  // GPIOのポート番号リスト
    const std::vector<double> output_voltage_list, // 出力電圧リスト
    OBC* obc
  );
  // Override: PCU functions
  void MainRoutine(int count) override;
  std::string GetLogHeader() const override;

private:
  std::vector<double> output_voltage_list_;
};
