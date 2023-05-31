#pragma once
#include <components/base/component.hpp>
#include <components/base/i2c_target_communication_with_obc.hpp>
#include <components/ports/power_port.hpp>

/* References
Manual:
https://www.tij.co.jp/jp/lit/ds/symlink/ina260.pdf?ts=1624796495336&ref_url=https%253A%252F%252Fwww.google.com%252F
HowToUse: NA
Note: Functions not used in the project are not implemented
*/

class INA260 : public Component, public I2cTargetCommunicationWithObc {
 public:
  INA260(int prescaler, ClockGenerator *clock_gen,
         PowerPort *ina_power_port,          // Power port witch provides electrical
                                             // power to INA260
         const double ina_minimum_voltage, const double ina_assumed_power_consumption,
         PowerPort *observation_power_port,  // Power port witch is observed by INA260
         const int i2c_port_id, const unsigned char i2c_addr, OnBoardComputer *obc);

  INA260(INA260 &&obj) noexcept;

  ~INA260();
  // ComponentBase
  void MainRoutine(int count) override;

 private:
  PowerPort *observation_power_port_;
  unsigned char mode_ = 1;  // 0: Continuous mode, others: triggered (Not emulated yet)
  double over_current_threshold_mA;

  enum class INA260_REGISTER : unsigned char {
    CURRENT = 0x01,
    VOLTAGE = 0x02,
  };

  enum class INA260_CMD : unsigned char {
    CONFIG = 0x00,
    LIMIT_MASK = 0x06,
    LIMIT_VALUE = 0x07,
  };

  const double kConvertObservedValue = 1.25;
  const double kConvertLimitValue = 0.8;

  void GenerateTelemetry();
  void ReadConfigCmd();
  void ReadLimitMaskCmd();
  void ReadLimitValueCmd();
};
