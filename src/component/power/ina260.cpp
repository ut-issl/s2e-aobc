/**
 * @file ina260.cpp
 * @brief Class to emulate INA260 current sensor
 */

#include "ina260.hpp"

INA260::INA260(int prescaler, ClockGenerator *clock_gen, PowerPort *ina_power_port, const double ina_minimum_voltage,
               const double ina_assumed_power_consumption, PowerPort *observation_power_port, const int i2c_port_id, const unsigned char i2c_addr,
               OnBoardComputer *obc)
    : Component(prescaler, clock_gen, ina_power_port),
      I2cTargetCommunicationWithObc(i2c_port_id, i2c_addr, obc),
      observation_power_port_(observation_power_port) {
  power_port_->SetMinimumVoltage_V(ina_minimum_voltage);
  power_port_->SetAssumedPowerConsumption_W(ina_assumed_power_consumption);
}

INA260::INA260(INA260 &&obj) noexcept
    : Component(obj), I2cTargetCommunicationWithObc(std::move(obj)), observation_power_port_(obj.observation_power_port_) {
  obj.observation_power_port_ = nullptr;
}

INA260::~INA260() {}

void INA260::MainRoutine(const int time_count) {
  UNUSED(time_count);
  // Read Registers
  ReadConfigCmd();
  ReadLimitValueCmd();
  // ReadLimitMaskCmd(); // TODO 今はエラーに成るのでコメントアウト

  // Update Register
  if (mode_ == 0)  // Continuous mode
  {
    GenerateTelemetry();
  }
  return;
}

void INA260::GenerateTelemetry() {
  // Read PowerPort
  double current_mA = observation_power_port_->GetCurrentConsumption_A() * 1000.0;
  double voltage_mV = observation_power_port_->GetVoltage_V() * 1000.0;

  // Convert data
  signed short current_s16_mA = (signed short)(current_mA / kConvertObservedValue);
  unsigned short voltage_u16_mV = 0x0000;
  if (voltage_mV >= 0.0) {
    voltage_u16_mV = (unsigned short)(voltage_mV / kConvertObservedValue);
  }

  // Write Register
  const int kTlmSize = 2;
  unsigned char current_tlm_tmp_mA[kTlmSize] = {0x00, 0x00};
  unsigned char voltage_tlm_tmp_mV[kTlmSize] = {0x00, 0x00};
  memcpy(current_tlm_tmp_mA, &current_s16_mA, kTlmSize);
  memcpy(voltage_tlm_tmp_mV, &voltage_u16_mV, kTlmSize);
  unsigned char current_tlm_mA[kTlmSize] = {0x00, 0x00};
  unsigned char voltage_tlm_mV[kTlmSize] = {0x00, 0x00};

  for (int i = 0; i < kTlmSize; i++) {
    current_tlm_mA[i] = (unsigned char)(current_s16_mA >> 8 * (1 - i));  // current_tlm_tmp_mA[kTlmSize - i - 1];
    voltage_tlm_mV[i] = (unsigned char)(voltage_u16_mV >> 8 * (1 - i));  // voltage_tlm_tmp_mV[kTlmSize - i - 1];
  }

  WriteRegister((unsigned char)INA260_REGISTER::CURRENT, current_tlm_mA, kTlmSize);
  // WriteRegister((unsigned char)INA260_REGISTER::VOLTAGE, voltage_tlm_mV,
  // kTlmSize); //TODO　現在のI2C
  // Portの仕様では、1レジスタが8byte以上のパターンに対応できない
  UNUSED(voltage_tlm_mV);  // TODO 上をコメントアウトしたら削除する

  return;
}

void INA260::ReadConfigCmd() {
  unsigned char rx_data[3] = {0, 0, 0};
  ReadCommand(rx_data, 3);
  if (rx_data[0] != (unsigned char)INA260_CMD::CONFIG) return;

  if (rx_data[1] == 0x62 && rx_data[2] == 0x07) mode_ = 0;
  return;
}

void INA260::ReadLimitMaskCmd() {
  unsigned char rx_data[3] = {0, 0, 0};
  ReadCommand(rx_data, 3);
  if (rx_data[0] != (unsigned char)INA260_CMD::LIMIT_MASK) return;

  if (rx_data[1] == 0x80 && rx_data[2] == 0x01) {
    observation_power_port_->SetCurrentLimit_A(over_current_threshold_mA);
  } else {
    observation_power_port_->SetCurrentLimit_A(10.0);  // 絶対に引っかからない大きな値として10Aを入れる
  }
  return;
}

void INA260::ReadLimitValueCmd() {
  unsigned char rx_data[3] = {0, 0, 0};
  ReadCommand(rx_data, 3);
  if (rx_data[0] != (unsigned char)INA260_CMD::LIMIT_VALUE) return;

  unsigned short over_current_threshold_raw = 0x0000;
  for (int i = 0; i < 2; i++) {
    over_current_threshold_raw |= (unsigned char)(rx_data[i + 1] << 8 * i);
  }
  over_current_threshold_mA = (double)(over_current_threshold_raw / kConvertLimitValue);

  return;
}
