/**
 * @file sagitta.cpp
 * @brief Class to emulate Sagitta star sensor
 * @note Manual: NA
 */

#include "sagitta.hpp"

#include <utilities/slip.hpp>

Sagitta::Sagitta(s2e::components::StarSensor stt, const int sils_port_id, s2e::components::OnBoardComputer *obc)
    : s2e::components::StarSensor(stt), s2e::components::UartCommunicationWithObc(sils_port_id, obc) {}

Sagitta::Sagitta(s2e::components::StarSensor stt, const int sils_port_id, s2e::components::OnBoardComputer *obc, const unsigned int hils_port_id,
                 const unsigned int baud_rate, s2e::simulation::HilsPortManager *hils_port_manager)
    : s2e::components::StarSensor(stt), s2e::components::UartCommunicationWithObc(sils_port_id, obc, hils_port_id, baud_rate, hils_port_manager) {}

void Sagitta::MainRoutine(const int time_count) {
  int receive_data_size = ReceiveCommand(0, kMaxCmdSize_);

  if (tlm_reply_mode_ == TLM_REPLY_MODE_SYNCHRONOUS && receive_data_size < 1) return;  // テレメ送信コマンドが送られていない場合
  if (!is_initialized_) {
    runtime_ms_ = 0;
    unix_time_us_ = 0;
    return;
  }
  status_ = 0;
  counter_ += time_count;  // TODO: 内部カウンタに合わせて修正

  runtime_ms_ += uint32_t(step_time_s_ * 1000);
  unix_time_us_ += uint64_t(step_time_s_ * 1000000);  // TODO: unix時間にする。simtimeを内部でもつ？
  // Quaternionの更新
  Measure(&(local_environment_->GetCelestialInformation()), &(dynamics_->GetAttitude()));

  q_tlm_i2c_[0] = static_cast<float>(measured_quaternion_i2c_[3]);  // Sagittaでは1要素目，S2Eでは4要素目がスカラー．
  for (uint8_t i = 0; i < 3; i++) {
    q_tlm_i2c_[i + 1] = static_cast<float>(measured_quaternion_i2c_[i]);
  }
  mcu_temperature_degC_ = 26.0f;
  cmos_temperature_degC_ = 24.0f;
  fpga_temperature_degC_ = 25.0f;

  // Send Telemetry
  SendTelemetry(0);

  return;
}

int Sagitta::ParseCommand(const int command_size) {
  if (command_size < 1) return -1;
  std::vector<uint8_t> decoded_rx = s2e::utilities::decode_slip_with_header(rx_buffer_);
  if (decoded_rx.size() < sizeof(kAddress_) + kXxhashSize_) return -1;

  if (!is_initialized_) {
    return AnalyzeCmdBoot(decoded_rx);
  } else {
    return AnalyzeCmd(decoded_rx);
  }
}

int Sagitta::GenerateTelemetry() {
  tx_buffer_.assign(kMaxTlmSize_, 0);

  uint8_t command_reply_tlm;
  switch (tlm_reply_mode_) {
    case TLM_REPLY_MODE_SYNCHRONOUS:
      command_reply_tlm = kCmdReplyTelemSync_;
      break;
    case TLM_REPLY_MODE_ASYNCHRONOUS:
      if (tlm_counter_ % 10 == 1 && is_subscribed_temperature_ == true)  // 1[Hz]
      {
        rec_mode_ = REC_MODE_TEMPERATURE;
      } else if (tlm_counter_ % 2 == 0 && is_subscribed_quaternion_ == true)  // 5[Hz]
      {
        rec_mode_ = REC_MODE_QUATERNION;
      } else {
        rec_mode_ = REC_MODE_OTHERS;
      }
      tlm_counter_++;
      command_reply_tlm = kCmdReplyTelemAsync_;
      break;
    default:
      return -1;
  }

  switch (rec_mode_) {
    case REC_MODE_RUN_TIME:
      return GenerateTelemetryRunTime(command_reply_tlm);
    case REC_MODE_QUATERNION:
      return GenerateTelemetryQuaternion(command_reply_tlm);
    case REC_MODE_TEMPERATURE:
      return GenerateTelemetryTemperature(command_reply_tlm);
    case REC_MODE_OTHERS:
      return 0;
    default:
      return -1;
  }
}

int Sagitta::AnalyzeCmdBoot(std::vector<uint8_t> decoded_rx) {
  const int kCmdSize = 4;
  // 最初のkCmdSizeのみ見て判断
  unsigned char kCmdBoot[kCmdSize] = {kAddress_, kCmdAction_, kActionIdBoot_, kParamRegion_};

  for (int i = 0; i < kCmdSize; i++) {
    if (decoded_rx[i] != kCmdBoot[i]) return -1;
  }

  is_initialized_ = 1;

  return 1;
}

int Sagitta::AnalyzeCmd(std::vector<uint8_t> decoded_rx) {
  if (decoded_rx[0] != kAddress_) return -1;
  const int cmd_id = decoded_rx[1];

  switch (cmd_id) {
    case kCmdSetParam_:
      AnalyzeCmdRequestTlm(decoded_rx);
      break;
    case kCmdAction_:
      AnalyzeCmdSetTime(decoded_rx);
      break;
    default:
      return -1;
  }

  return 1;
}

int Sagitta::AnalyzeCmdSetTime(std::vector<uint8_t> decoded_rx) {
  if (decoded_rx[2] != kActionIdSetTime_) return -1;

  memcpy(&unix_time_us_, &decoded_rx[3], sizeof(unix_time_us_));

  return 1;
}

int Sagitta::AnalyzeCmdRequestTlm(std::vector<uint8_t> decoded_rx) {
  if (decoded_rx[2] == kParamSubscription) {
    tlm_reply_mode_ = TLM_REPLY_MODE_ASYNCHRONOUS;
    // TODO 3種類以上の非同期テレメに対応
    if (decoded_rx[3] == kTelemIdTemperature_ || decoded_rx[4] == kTelemIdTemperature_) {
      is_subscribed_temperature_ = true;
    } else {
      is_subscribed_temperature_ = false;
    }
    if (decoded_rx[3] == kTelemIdSolution_ || decoded_rx[4] == kTelemIdSolution_) {
      is_subscribed_quaternion_ = true;
    } else {
      is_subscribed_quaternion_ = false;
    }
  } else if (decoded_rx[1] == kCmdRequestTelem_) {
    tlm_reply_mode_ = TLM_REPLY_MODE_SYNCHRONOUS;
    return AnalyzeTlmId(decoded_rx[2]);
  } else {
    return -1;
  }

  return 1;
}

int Sagitta::AnalyzeTlmId(const uint8_t tlm_id) {
  switch (tlm_id) {
    case kTelemIdTime_:
      rec_mode_ = REC_MODE_RUN_TIME;
      break;
    case kTelemIdSolution_:
      rec_mode_ = REC_MODE_QUATERNION;
      break;
    case kTelemIdTemperature_:
      rec_mode_ = REC_MODE_TEMPERATURE;
      break;
    default:
      rec_mode_ = REC_MODE_OTHERS;
      return -1;
  }
  return 1;
}

int Sagitta::GenerateTelemetryRunTime(const uint8_t command_reply_tlm) {
  const int kTlmSize = 32;
  std::vector<uint8_t> tlm(kTlmSize, 0);
  memset(&tlm[0], 0x00, kTlmSize);

  memcpy(&tlm[0], &kAddress_, 1);
  memcpy(&tlm[1], &command_reply_tlm, 1);
  memcpy(&tlm[2], &kTelemIdTime_, 1);
  memcpy(&tlm[3], &status_, 1);
  memcpy(&tlm[4], &counter_, 4);
  memcpy(&tlm[8], &unix_time_us_, 8);
  memcpy(&tlm[16], &runtime_ms_, 4);
  memcpy(&tlm[20], &unix_time_us_, 8);
  uint32_t xxhash = XXHash32::hash(&tlm[0], kTlmSize - kXxhashSize_, kXxhashSeed_);
  memcpy(&tlm[kTlmSize - kXxhashSize_], &xxhash, sizeof(xxhash));

  std::vector<uint8_t> encoded_tx = s2e::utilities::encode_slip_with_header(tlm);
  tx_buffer_.assign(encoded_tx.begin(), encoded_tx.end());
  return (int)encoded_tx.size();
}

int Sagitta::GenerateTelemetryQuaternion(const uint8_t command_reply_tlm) {
  const int kTlmSize = 87;
  std::vector<uint8_t> tlm(kTlmSize, 0);
  memset(&tlm[0], 0x00, kTlmSize);

  memcpy(&tlm[0], &kAddress_, 1);
  memcpy(&tlm[1], &command_reply_tlm, 1);
  memcpy(&tlm[2], &kTelemIdSolution_, 1);
  memcpy(&tlm[3], &status_, 1);
  memcpy(&tlm[4], &counter_, 4);
  memcpy(&tlm[8], &unix_time_us_, 8);
  for (uint8_t i = 0; i < 4; i++) {
    memcpy(&tlm[16 + 4 * i], &q_tlm_i2c_[i], 4);
  }
  bool is_valid = !error_flag_;
  memcpy(&tlm[77], &is_valid, 1);
  uint32_t xxhash = XXHash32::hash(&tlm[0], kTlmSize - kXxhashSize_, kXxhashSeed_);
  memcpy(&tlm[kTlmSize - kXxhashSize_], &xxhash, sizeof(xxhash));

  std::vector<uint8_t> encoded_tx = s2e::utilities::encode_slip_with_header(tlm);
  tx_buffer_.assign(encoded_tx.begin(), encoded_tx.end());
  return (int)encoded_tx.size();
}

int Sagitta::GenerateTelemetryTemperature(const uint8_t command_reply_tlm) {
  const int kTlmSize = 32;
  std::vector<uint8_t> tlm(kTlmSize, 0);
  memset(&tlm[0], 0x00, kTlmSize);

  memcpy(&tlm[0], &kAddress_, 1);
  memcpy(&tlm[1], &command_reply_tlm, 1);
  memcpy(&tlm[2], &kTelemIdTemperature_, 1);
  memcpy(&tlm[3], &status_, 1);
  memcpy(&tlm[4], &counter_, 4);
  memcpy(&tlm[8], &unix_time_us_, 8);
  memcpy(&tlm[16], &mcu_temperature_degC_, sizeof(mcu_temperature_degC_));
  memcpy(&tlm[20], &cmos_temperature_degC_, sizeof(cmos_temperature_degC_));
  memcpy(&tlm[24], &fpga_temperature_degC_, sizeof(fpga_temperature_degC_));
  uint32_t xxhash = XXHash32::hash(&tlm[0], kTlmSize - kXxhashSize_, kXxhashSeed_);
  memcpy(&tlm[kTlmSize - kXxhashSize_], &xxhash, sizeof(xxhash));

  std::vector<uint8_t> encoded_tx = s2e::utilities::encode_slip_with_header(tlm);
  tx_buffer_.assign(encoded_tx.begin(), encoded_tx.end());
  return (int)encoded_tx.size();
}
