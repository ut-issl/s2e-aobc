/**
 * @file stim210.cpp
 * @brief Class to emulate STIM210 gyro sensor
 */

#include "stim210.hpp"

#include <library/utilities/macros.hpp>

Stim210::Stim210(GyroSensor gyro, double compo_step_sec, const int sils_port_id, OnBoardComputer *obc)
    : GyroSensor(gyro), UartCommunicationWithObc(sils_port_id, obc), counter_(0), compo_step_sec_(compo_step_sec) {}

Stim210::Stim210(GyroSensor gyro, double compo_step_sec, const int sils_port_id, OnBoardComputer *obc, const unsigned int hils_port_id,
                 const unsigned int baud_rate, HilsPortManager *hils_port_manager)
    : GyroSensor(gyro),
      UartCommunicationWithObc(sils_port_id, obc, hils_port_id, baud_rate, hils_port_manager),
      counter_(0),
      compo_step_sec_(compo_step_sec) {}

void Stim210::MainRoutine(const int time_count) {
  UNUSED(time_count);
  ReceiveCommand(0, kMaxRxSize);
  angular_velocity_c_rad_s_ = quaternion_b2c_.FrameConversion(dynamics_->GetAttitude().GetAngularVelocity_b_rad_s());  // Convert frame
  angular_velocity_c_rad_s_ = Measure(angular_velocity_c_rad_s_);                                                      // Add noises

  counter_ += (unsigned char)(prescaler_ * compo_step_sec_ * sample_rate_hz_[SAMPLE_RATE_2000HZ]);  // 2000Hzでインクリメントされる

  for (size_t i = 0; i < kGyroDimension; i++) {
    temperature_c_degC_[i] = 30.0 + ((double)i) * 0.1;  // TODO: 温度の反映
  }
  // Send Telemetry
  SendTelemetry(0);

  return;
}

std::string Stim210::GetLogHeader() const {
  std::string str_tmp = "";
  std::string section = "Stim210";
  str_tmp += WriteVector(section, "c", "deg/s", 3);

  return str_tmp;
}

int Stim210::ParseCommand(const int command_size) {
  std::vector<unsigned char> cmd = rx_buffer_;
  int idx = 0;
  int ret = -1;
  for (int i = 0; i < command_size; i++) {
    cmd[idx] = rx_buffer_[i];
    idx++;
    if (rx_buffer_[i] == termination_cr)  // CRでコマンドを区切る
    {
      idx = 0;
      switch (cmd[0]) {
        case 'S':
          if (operation_mode_ != OPERATION_SERVICE_MODE) ret = AnalyzeCmdServiceMode(cmd);
          break;
        case 'x':
          if (operation_mode_ == OPERATION_SERVICE_MODE) ret = AnalyzeCmdNormalMode(cmd);
          break;
        case 'r':
          if (operation_mode_ == OPERATION_SERVICE_MODE) ret = AnalyzeCmdTermination(cmd);
          break;
        case 'd':
          if (operation_mode_ == OPERATION_SERVICE_MODE) ret = AnalyzeCmdSetNormalModeFormat(cmd);
          break;
        case 'm':
          if (operation_mode_ == OPERATION_SERVICE_MODE) ret = AnalyzeCmdSetSampleRate(cmd);
          break;
        case 'u':
          if (operation_mode_ == OPERATION_SERVICE_MODE) ret = AnalyzeCmdSetOmegaMode(cmd);
          break;
        case 'f':
          if (operation_mode_ == OPERATION_SERVICE_MODE) ret = AnalyzeCmdSetLPFFrequency(cmd);
          break;
        default:
          return -1;
      }
    }
  }

  return ret;
}

int Stim210::GenerateTelemetry() {
  tx_buffer_.assign(kMaxTxSize, 0);
  switch (operation_mode_) {
    case OPERATION_INIT_MODE:
      // このテレメは利用しないのでテレメを返さない
      return 0;
    case OPERATION_NORMAL_MODE:
      return GenerateNormalModeTlm();
    case OPERATION_SERVICE_MODE:
      // このテレメは利用しないのでテレメを返さない
      return 0;
    default:
      return -1;
  }
}

int Stim210::GenerateNormalModeTlm() {
  int tlm_size = 0;
  GenerateFormatTlm(tlm_size);
  GenerateOmegaTlm(tlm_size);
  GenerateStatusTlm(tlm_size);

  switch (normal_mode_format_) {
    case NORMAL_MODE_STANDARD:
      break;
    case NORMAL_MODE_EXTENDED:
      GenerateBufferTlm(tlm_size);
      break;
    case NORMAL_MODE_RATE_TEMPERATURE:
      GenerateTemperatureTlm(tlm_size);
      break;
    case NORMAL_MODE_RATE_COUNT:
      GenerateCountTlm(tlm_size);
      break;
    case NORMAL_MODE_RATE_LATENCY:
      GenerateLatencyTlm(tlm_size);
      break;
    case NORMAL_MODE_RATE_COUNT_LATENCY:
      GenerateCountTlm(tlm_size);
      GenerateLatencyTlm(tlm_size);
      break;
    case NORMAL_MODE_RATE_TEMPERATURE_COUNT:
      GenerateTemperatureTlm(tlm_size);
      GenerateCountTlm(tlm_size);
      break;
    case NORMAL_MODE_RATE_TEMPERATURE_LATENCY:
      GenerateTemperatureTlm(tlm_size);
      GenerateLatencyTlm(tlm_size);
      break;
    case NORMAL_MODE_RATE_TEMPERATURE_COUNT_LATENCY:
      GenerateTemperatureTlm(tlm_size);
      GenerateCountTlm(tlm_size);
      GenerateLatencyTlm(tlm_size);
      break;
    default:
      return -1;
  }

  GenerateCRCTlm(tlm_size);
  GenerateTerminationTlm(tlm_size);
  return tlm_size;
}

void Stim210::GenerateFormatTlm(int &offset) {
  const int kTlmSize = 1;
  std::vector<unsigned char> tlm = {normal_mode_format_idx_[normal_mode_format_]};
  SetTlm(tlm, offset, kTlmSize);
  return;
}

void Stim210::GenerateOmegaTlm(int &offset) {
  const int kByte2Bit = 8;
  const int kOmegaByte = 3;  // 1軸当たりのバイト数
  const int kTlmSize = kOmegaByte * kGyroDimension;
  std::vector<unsigned char> tlm(kTlmSize, 0);
  int angular_velocity_c_tlm[kGyroDimension] = {0, 0, 0};

  // TODO: LPFの実装

  for (size_t i = 0; i < kGyroDimension; i++) {
    angular_velocity_c_tlm[i] = ConvertOmega2Tlm(angular_velocity_c_rad_s_[i]);
    for (size_t j = 0; j < kOmegaByte; j++) {
      tlm[i * kOmegaByte + j] = (unsigned char)(angular_velocity_c_tlm[i] >> kByte2Bit * (kOmegaByte - j - 1)) & 0xff;
    }
  }
  SetTlm(tlm, offset, kTlmSize);
  return;
}

void Stim210::GenerateStatusTlm(int &offset) {
  const int kTlmSize = 1;
  std::vector<unsigned char> tlm = {status_};
  SetTlm(tlm, offset, kTlmSize);
  return;
}

void Stim210::GenerateBufferTlm(int &offset) {
  const int kTlmSize = 3;
  std::vector<unsigned char> tlm(kTlmSize, 0);
  SetTlm(tlm, offset, kTlmSize);
  return;
}

void Stim210::GenerateTemperatureTlm(int &offset) {
  const int kByte2Bit = 8;
  const int kTempByte = 2;  // 1軸当たりのバイト数
  const int kTlmSize = kTempByte * kGyroDimension;
  std::vector<unsigned char> tlm(kTlmSize, 0);
  int temperature_c_tlm[kGyroDimension] = {0, 0, 0};
  for (size_t i = 0; i < kGyroDimension; i++) {
    temperature_c_tlm[i] = ConvertTemp2Tlm(temperature_c_degC_[i]);
    for (int j = 0; j < kTempByte; j++) {
      tlm[i * kTempByte + j] = (unsigned char)(temperature_c_tlm[i] >> kByte2Bit * (kTempByte - j - 1)) & 0xff;
    }
  }
  SetTlm(tlm, offset, kTlmSize);
  return;
}

void Stim210::GenerateCountTlm(int &offset) {
  const int kTlmSize = 1;
  std::vector<unsigned char> tlm = {counter_};
  SetTlm(tlm, offset, kTlmSize);
  return;
}

void Stim210::GenerateLatencyTlm(int &offset) {
  const int kTlmSize = 2;
  std::vector<unsigned char> tlm = {2, 6};
  SetTlm(tlm, offset, kTlmSize);
  return;
}

void Stim210::GenerateCRCTlm(int &offset) {
  const int kTlmSize = 1;
  std::vector<unsigned char> tlm = {Crc8AtmLeft(kCrcInitial, &tx_buffer_[0], offset, kRevFlag)};
  SetTlm(tlm, offset, kTlmSize);
  return;
}

void Stim210::GenerateTerminationTlm(int &offset) {
  const int kTlmSize = 2;
  std::vector<unsigned char> tlm = {0x0d, termination_cr};

  switch (termination_mode_) {
    case TERMINATION_OFF:
      break;
    case TERMINATION_CRLF:
      SetTlm(tlm, offset, kTlmSize);
      break;
    default:
      break;
  }

  return;
}

void Stim210::SetTlm(std::vector<unsigned char> tlm, int &offset, size_t tlm_size) {
  for (size_t i = 0; i < tlm_size; i++) {
    tx_buffer_[i + offset] = tlm[i];
  }
  offset += tlm_size;
  return;
}

int32_t Stim210::ConvertOmega2Tlm(double angular_velocity_c_rad_s) {
  double angular_velocity_c_dps = angular_velocity_c_rad_s * 180.0 / libra::pi;
  int32_t angular_velocity_c_bit = int32_t(angular_velocity_c_dps * pow(2, 14));

  // Limits
  int32_t upper_limit = 0x007FFFFF;        // Signed 24bit max value
  int32_t lower_limit = -upper_limit - 1;  // Signed 24bit min value
  angular_velocity_c_bit = (std::min)(upper_limit, angular_velocity_c_bit);
  angular_velocity_c_bit = (std::max)(lower_limit, angular_velocity_c_bit);
  // 24 bit
  if (angular_velocity_c_bit >= 0) {
    angular_velocity_c_bit = angular_velocity_c_bit & upper_limit;
  } else {
    angular_velocity_c_bit = (angular_velocity_c_bit & upper_limit) | (upper_limit + 1);
  }
  return angular_velocity_c_bit;
}

int16_t Stim210::ConvertTemp2Tlm(double temp) {
  int16_t temp_c_bit = int16_t(temp * pow(2, 8));

  // Limits
  int16_t upper_limit = 0x007FFF;          // Signed 16bit max value
  int16_t lower_limit = -upper_limit - 1;  // Signed 16bit min value
  temp_c_bit = (std::min)(upper_limit, temp_c_bit);
  temp_c_bit = (std::max)(lower_limit, temp_c_bit);
  // 16 bit
  if (temp_c_bit >= 0) {
    temp_c_bit = temp_c_bit & upper_limit;
  } else {
    temp_c_bit = (temp_c_bit & upper_limit) | (upper_limit + 1);
  }
  return temp_c_bit;
}

int Stim210::AnalyzeCmdServiceMode(std::vector<unsigned char> cmd) {
  const int kTlmSize = 12;
  unsigned char command_servicemode[kTlmSize] = {'S', 'E', 'R', 'V', 'I', 'C', 'E', 'M', 'O', 'D', 'E', termination_cr};
  for (int i = 0; i < kTlmSize; i++) {
    if (command_servicemode[i] != cmd[i]) return -1;
  }

  operation_mode_ = OPERATION_SERVICE_MODE;

  return 0;
}

int Stim210::AnalyzeCmdNormalMode(std::vector<unsigned char> cmd) {
  const int kTlmSize = 4;
  unsigned char command_servicemode[kTlmSize] = {'x', ' ', 'N', termination_cr};
  for (int i = 0; i < kTlmSize; i++) {
    if (command_servicemode[i] != cmd[i]) return -1;
  }

  operation_mode_ = OPERATION_NORMAL_MODE;

  return 0;
}

int Stim210::AnalyzeCmdTermination(std::vector<unsigned char> cmd) {
  if (cmd[1] != ' ') return -1;
  switch (cmd[2]) {
    case '2':
      termination_mode_ = TERMINATION_OFF;
      break;
    case '3':
      termination_mode_ = TERMINATION_CRLF;
      break;
    default:
      return -1;
  }
  return 0;
}

int Stim210::AnalyzeCmdSetNormalModeFormat(std::vector<unsigned char> cmd) {
  unsigned char param = cmd[2];
  if (cmd[1] != ' ') return -1;
  if (cmd[3] != termination_cr) return -1;
  switch (param) {
    case 's':
      normal_mode_format_ = NORMAL_MODE_STANDARD;
      break;
    case 'e':
      normal_mode_format_ = NORMAL_MODE_EXTENDED;
      break;
    case 'j':
      normal_mode_format_ = NORMAL_MODE_RATE_TEMPERATURE;
      break;
    case 'k':
      normal_mode_format_ = NORMAL_MODE_RATE_COUNT;
      break;
    case 'l':
      normal_mode_format_ = NORMAL_MODE_RATE_LATENCY;
      break;
    case 'm':
      normal_mode_format_ = NORMAL_MODE_RATE_COUNT_LATENCY;
      break;
    case 'n':
      normal_mode_format_ = NORMAL_MODE_RATE_TEMPERATURE_COUNT;
      break;
    case 'o':
      normal_mode_format_ = NORMAL_MODE_RATE_TEMPERATURE_LATENCY;
      break;
    case 'p':
      normal_mode_format_ = NORMAL_MODE_RATE_TEMPERATURE_COUNT_LATENCY;
      break;
    default:
      return -1;
  }
  return 0;
}

int Stim210::AnalyzeCmdSetSampleRate(std::vector<unsigned char> cmd) {
  if (cmd[1] != ' ') return -1;
  int tmp = 1;
  // 文字数によって処理を変える
  if (cmd[5] == termination_cr) {
    tmp = int(cmd[2]) * 100 + int(cmd[3]) * 10 + int(cmd[4]);
  } else if (cmd[6] == termination_cr) {
    tmp = int(cmd[2]) * 1000 + int(cmd[3]) * 100 + int(cmd[4]) * 10 + int(cmd[5]);
  } else {
    return -1;
  }

  switch (tmp) {
    case 125:
      sample_rate_mode_ = SAMPLE_RATE_125HZ;
      break;
    case 250:
      sample_rate_mode_ = SAMPLE_RATE_250HZ;
      break;
    case 500:
      sample_rate_mode_ = SAMPLE_RATE_500HZ;
      break;
    case 1000:
      sample_rate_mode_ = SAMPLE_RATE_1000HZ;
      break;
    case 2000:
      sample_rate_mode_ = SAMPLE_RATE_2000HZ;
      break;
    case 0:
      sample_rate_mode_ = SAMPLE_RATE_2000HZ;
      // TODO: GPIO処理が実装されたらこちらで処理。SAMPLE_RATE_EXTERNAL_TRIGGER;
      break;
    default:
      return -1;
  }
  return 0;
}

int Stim210::AnalyzeCmdSetOmegaMode(std::vector<unsigned char> cmd) {
  if (cmd[1] != ' ') return -1;
  switch (cmd[2]) {
    case 'a':
      angular_velocity_mode_ = GYRO_OUTPUT_ANGULAR_RATE;
      break;
    case 'i':
      angular_velocity_mode_ = GYRO_OUTPUT_INCREMENTAL_ANGLE;
      break;
    case 'm':
      angular_velocity_mode_ = GYRO_OUTPUT_AVERAGE_ANGULAR_RATE;
      break;
    case 's':
      angular_velocity_mode_ = GYRO_OUTPUT_INTEGRATED_ANGLE;
      break;
    default:
      return -1;
  }
  return 0;
}

int Stim210::AnalyzeCmdSetLPFFrequency(std::vector<unsigned char> cmd) {
  if (cmd[1] != ' ') return -1;
  int tmp = 1;
  // 終端子の文字数によって処理を変える
  if (cmd[4] == termination_cr) {
    tmp = int(cmd[2]) * 10 + int(cmd[3]);
  } else if (cmd[5] == termination_cr) {
    tmp = int(cmd[2]) * 100 + int(cmd[3]) * 10 + int(cmd[4]);
  } else {
    return -1;
  }

  switch (tmp) {
    case 16:
      lpf_freq_ = LPF_16HZ;
      break;
    case 33:
      lpf_freq_ = LPF_33HZ;
      break;
    case 66:
      lpf_freq_ = LPF_66HZ;
      break;
    case 131:
      lpf_freq_ = LPF_131HZ;
      break;
    case 262:
      lpf_freq_ = LPF_262HZ;
      break;
    default:
      return -1;
  }
  return 0;
}
