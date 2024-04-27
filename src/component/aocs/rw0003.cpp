/**
 * @file rw0003.cpp
 * @brief Class to emulate RW0.003 reaction wheel
 */

#include "rw0003.hpp"

#include <math.h>

#include <utilities/macros.hpp>
#include <utilities/slip.hpp>

Rw0003::Rw0003(ReactionWheel rw, const int sils_port_id, const unsigned int hils_port_id, const unsigned char i2c_address, OnBoardComputer *obc,
               HilsPortManager *hils_port_manager)
    : ReactionWheel(rw), I2cTargetCommunicationWithObc(sils_port_id, hils_port_id, i2c_address, obc, hils_port_manager) {
  Initialize();
}

void Rw0003::MainRoutine(const int time_count) {
  UNUSED(time_count);
  // TODO: HILSで回転数テレメ要求と温度テレメ要求を別々に受け取れているか確認
  // 必要に応じてHILS時のCompoUpdateIntervalSecを小さくする
  if (ReceiveCommand() > 0) {
    is_command_written_ = true;
  }

  // Read Command
  ReadCmd();

  // Generate TLM
  if (is_rw_initialized_ == true) {
    CalcTorque();
    WriteFloatTlm(kReadAddressTemperature_, (float)temperature_degC_);
    WriteFloatTlm(kReadAddressSpeed_, (float)angular_velocity_rad_s_);
  }

  is_command_written_ = false;
  return;
}

std::string Rw0003::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar("RW0003_angular_velocity", "rad/s");
  str_tmp += WriteScalar("RW0003_angular_velocity", "rpm");
  str_tmp += WriteScalar("RW0003_angular_velocity_upper_limit", "rpm");
  str_tmp += WriteScalar("RW0003_target_angular_acceleration", "rad/s2");
  str_tmp += WriteScalar("RW0003_angular_acceleration", "rad/s^2");

  if (is_logged_jitter_) {
    str_tmp += WriteVector("RW0003_jitter_force", "c", "N", 3);
    str_tmp += WriteVector("RW0003_jitter_torque", "c", "Nm", 3);
  }

  return str_tmp;
}

void Rw0003::ReadCmd() {
  uint8_t rx_data[kMaxCmdLength_] = {0};
  ReadCommand(rx_data, kMaxCmdLength_);

  // Decode SLIP
  std::vector<uint8_t> rx_data_v(rx_data, rx_data + kMaxCmdLength_);
  std::vector<uint8_t> decoded_rx = decode_slip(rx_data_v);
  if (decoded_rx.size() <= kHeaderSize_ + kCrcSize_) return;  // no payload

  // Source Address
  uint8_t src_addr = decoded_rx[0];
  if (src_addr != kSourceAddress_) return;  // command error

  // CRC
  const unsigned char i2c_address = GetI2cAddress();
  uint16_t calculated_crc = kCrcInitial_;
  calculated_crc = Crc16CcittRight(calculated_crc, &i2c_address, sizeof(i2c_address), kCrcReverseFlag_);
  calculated_crc = Crc16CcittRight(calculated_crc, &decoded_rx[0], decoded_rx.size(), kCrcReverseFlag_);
  if (calculated_crc != 0x0000) return;  // CRC error

  // MCF
  uint8_t mcf = decoded_rx[1];
  uint8_t command_id, reply_flag;
  reply_flag = decode_mcf(&command_id, mcf);
  UNUSED(reply_flag);

  // Payload
  std::vector<uint8_t> payload{decoded_rx.begin() + kHeaderSize_, decoded_rx.end() - kCrcSize_};

  // Command handling
  switch (command_id) {
    case kCmdIdInit_:
      ReadCmdInit(payload);
      break;
    case kCmdIdWriteFile_:
      ReadCmdWriteFile(payload);
      break;
    case kCmdIdReadFile_:
      ReadCmdReadFile(payload);
      break;
    default:
      break;
  }

  return;
}

uint8_t Rw0003::decode_mcf(uint8_t *command_id, const uint8_t mcf) {
  uint8_t mask = 0x7f;
  *command_id = mcf & mask;
  if (mcf & 0x80)
    return 1;  // with reply
  else
    return 0;  // without reply
}

void Rw0003::ReadCmdInit(const std::vector<uint8_t> payload) {
  if (payload.size() != 4) return;
  std::vector<uint8_t> initial_address{0x00, 0x10, 0x00, 0x00};
  bool is_equal = std::equal(payload.cbegin(), payload.cend(), initial_address.cbegin());

  if (is_equal == true) {
    is_rw_initialized_ = true;
    SetDriveFlag(true);
  }

  return;
}

void Rw0003::ReadCmdWriteFile(const std::vector<uint8_t> payload) {
  if (payload.size() != 6) return;
  if (payload[0] != 0x00) return;

  uint8_t command_id = payload[1];

  switch (command_id) {
    case kWriteCmdIdle_:
      SetDriveFlag(false);
      SetTargetTorque_rw_Nm(0.0);
      break;
    case kWriteCmdSpeed_:
      // TODO: 回転数司令コマンドを実装する
      break;
    case kWriteCmdTorque_:
      float torque_Nm;
      memcpy(&torque_Nm, &payload[2], 4);
      SetTargetTorque_rw_Nm((double)torque_Nm);
      break;
    default:
      break;
  }

  return;
}

void Rw0003::ReadCmdReadFile(const std::vector<uint8_t> payload) {
  if (payload.size() != 1) return;
  if (is_rw_initialized_ != true) return;
  if (is_command_written_ != true) return;
  // これ以降はHILS用に事前にテレメトリを溜めておく
  unsigned char rx[1];
  const int kTlmSize = 15;

  if (payload[0] == kReadAddressTemperature_ || payload[0] == kReadAddressSpeed_) {
    // Set register address to kReadAddressSpeed_
    ReadRegister(kReadAddressSpeed_, rx, 1);
    SendTelemetry(kTlmSize);  // Send only speed telemetry. Temperature Tlm will be abnormal value.
  }
}

void Rw0003::WriteFloatTlm(uint8_t address, float value) {
  std::vector<uint8_t> tlm{kMcfReadEdac_};
  tlm.push_back(address);

  uint8_t value_u8[sizeof(float)];
  memcpy(value_u8, &value, sizeof(float));
  for (unsigned int i = 0; i < sizeof(float); i++) {
    tlm.push_back(value_u8[i]);
  }

  // CRC
  const unsigned char i2c_address = GetI2cAddress();
  uint16_t crc = kCrcInitial_;
  crc = Crc16CcittRight(crc, &kSourceAddress_, sizeof(kSourceAddress_), kCrcReverseFlag_);
  crc = Crc16CcittRight(crc, &i2c_address, sizeof(i2c_address), kCrcReverseFlag_);
  crc = Crc16CcittRight(crc, &tlm[0], tlm.size(), kCrcReverseFlag_);
  uint8_t crc_u8[kCrcSize_];
  memcpy(crc_u8, &crc, kCrcSize_);
  for (unsigned int i = 0; i < kCrcSize_; i++) {
    tlm.push_back(crc_u8[i]);
  }

  // SLIP
  std::vector<uint8_t> tlm_slip;
  tlm_slip = encode_slip(tlm);

  // Write Tlm
  WriteRegister(address, &tlm_slip[0], (uint8_t)tlm_slip.size());
}

void Rw0003::Initialize() {
  // HILS用に事前にテレメトリを溜めておく
  unsigned char rx[1];
  const int kTlmSize = 15;

  // Set register address to kReadAddressSpeed_
  ReadRegister(kReadAddressSpeed_, rx, 1);
  // SLIPを用いたRWでは、ObcI2cTargetCommunicationBaseクラスのstored_frame_counterが機能しない
  // 最初にkStoredFrameSize分のフレームをUSB-I2C
  // Target変換器にためておき、ためたフレーム数を維持する方針
  for (size_t i = 0; i < kStoredFrameSize; i++) {
    SendTelemetry(kTlmSize);
  }
}
