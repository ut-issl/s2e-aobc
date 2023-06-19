#pragma once
#include <components/base/i2c_target_communication_with_obc.hpp>
#include <components/real/aocs/reaction_wheel.hpp>

#include "../../library/crc.hpp"

/* References
Manual: NA
Note: Functions not used in the project are not implemented
*/

class RW0003 : public ReactionWheel, public I2cTargetCommunicationWithObc {
 public:
  RW0003(ReactionWheel rw, const int sils_port_id, const unsigned int hils_port_id, const unsigned char i2c_addr, OnBoardComputer *obc,
         HilsPortManager *hils_port_manager);

  // Override: RWModel functions
  void MainRoutine(int count) override;
  std::string GetLogHeader() const override;

 private:
  bool is_rw_initialized_ = false;
  // Dummy data
  double temperture_degC_ = 30.0;

  // Communication
  uint16_t crc_;
  const uint8_t kSrcAddr_ = 0x11;
  static const uint8_t kHeaderSize_ = 2;
  static const uint8_t kFooterSize_ = 1;
  static const uint8_t kCrcSize_ = 2;
  static const uint8_t kMaxCmdLength_ = 15;
  static const uint8_t kMaxTlmLength_ = 15;

  // Command
  static const uint8_t kCmdIdInit_ = 0x01;
  static const uint8_t kCmdIdReadFile_ = 0x07;
  static const uint8_t kCmdIdWriteFile_ = 0x08;

  // Write Command
  static const uint8_t kWriteCmdIdle_ = 0x00;
  static const uint8_t kWriteCmdSpeed_ = 0x03;
  static const uint8_t kWriteCmdTorque_ = 0x12;

  // Register address
  static const uint8_t kReadAddressTemperature_ = 0x03;
  static const uint8_t kReadAddressSpeed_ = 0x15;
  static const uint8_t kReadAddressLimitSpeed1_ = 0x33;
  static const uint8_t kReadAddressLimitSpeed2_ = 0x34;

  static const uint8_t kMcfReadEdac_ = 0xa7;
  static const uint16_t kCrcInitial_ = 0xffff;
  static const bool kCrcRevFlag_ = false;

  // HILS
  bool is_cmd_written_ = false;
  const unsigned int kStoredFrameSize = 3;
  void Initialize();

  // Cmd
  void ReadCmd();
  void ReadCmdInit(std::vector<uint8_t> payload);
  void ReadCmdWriteFile(const std::vector<uint8_t> payload);
  void ReadCmdReadFile(const std::vector<uint8_t> payload);
  uint8_t decode_mcf(uint8_t *cmd_id, const uint8_t mcf);

  // Tlm
  void WriteFloatTlm(uint8_t address, float value);
};
