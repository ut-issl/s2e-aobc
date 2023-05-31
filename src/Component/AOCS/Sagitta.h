#pragma once
#include <components/base/uart_communication_with_obc.hpp>
#include <components/real/aocs/star_sensor.hpp>

#include "../../Library/xxhash32.h"

/* References
ICD: NA
*/

class Sagitta : public StarSensor, public UartCommunicationWithObc {
public:
  Sagitta(StarSensor stt, const int sils_port_id, OnBoardComputer *obc);
  Sagitta(StarSensor stt, const int sils_port_id, OnBoardComputer *obc,
          const unsigned int hils_port_id, const unsigned int baud_rate,
          HilsPortManager *hils_port_manager);

  // Override: STT functions
  void MainRoutine(int count) override;

private:
  typedef enum {
    REC_MODE_RUN_TIME = 0,
    REC_MODE_TEMPERATURE,
    REC_MODE_QUATERNION,
    REC_MODE_OTHERS,
    REC_MODE_MAX,
  } REC_MODE;

  typedef enum {
    TLM_REPLY_MODE_SYNCHRONOUS = 0,
    TLM_REPLY_MODE_ASYNCHRONOUS,
    TLM_REPLY_MODE_MAX,
  } TLM_REPLY_MODE;

  bool is_initialized_ = 0;
  uint32_t runtime_ms_ = 0;
  double compo_step_sec_ = 0.0;
  REC_MODE rec_mode_ = REC_MODE_OTHERS;
  TLM_REPLY_MODE tlm_reply_mode_ = TLM_REPLY_MODE_SYNCHRONOUS;
  float q_tlm_i2c_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float mcu_temperature_degC_ = 25.0f;  // temperature of the microcontroller
  float cmos_temperature_degC_ = 23.0f; // temperature of the image sensor
  float fpga_temperature_degC_ = 24.0f; // temperature of the FPGA

  uint8_t status_ =
      1; // status of telemetry (0:OK, 1:not implemented, 2-255:tlm error)
  uint32_t counter_ = 0;      // internal counter
  uint64_t unix_time_us_ = 0; // telemetry dummy data [microseconds] Ref. 2.4.6
                              // Telemetry reply/Asynchronous Telemetry Reply

  static const unsigned int kMaxCmdSize_ = 80;
  static const unsigned int kMaxTlmSize_ = 128;
  static const unsigned int kHeaderSize_ = 1;
  static const unsigned int kFooterSize_ = 1;
  static const unsigned int kMaxSubscRegSize_ = 2;
  static const unsigned int kXxhashSize_ = 4;
  static const unsigned int kXxhashSeed_ = 1425;
  static const uint8_t kHeader_ = 0xc0; // ヘッダー
  static const uint8_t kFooter_ = 0xc0; // フッター
  bool is_subscribed_temperature_ = 0; // 非同期で温度テレメを送るフラグ
  bool is_subscribed_quaternion_ = 0; // 非同期でquaternionテレメを送るフラグ
  uint8_t tlm_counter_ = 0; // 非同期でテレメを送る頻度を決めるカウンタ

  static const uint8_t kAddress_ =
      0x21; // テレコマに含まれるSTTのアドレス。複数のSTTを用いる場合はそれぞれ異なるアドレスが付与される。
  static const uint8_t kCmdSetParam_ = 0x00;
  static const uint8_t kCmdRequestParam_ = 0x01;
  static const uint8_t kCmdRequestTelem_ = 0x02;
  static const uint8_t kCmdReplyTelemSync_ = 0x82;
  static const uint8_t kCmdReplyTelemAsync_ = 0x84;
  static const uint8_t kCmdAction_ = 0x03;

  static const uint8_t kTelemIdTime_ = 0x01;
  static const uint8_t kTelemIdSolution_ = 0x18;
  static const uint8_t kTelemIdTemperature_ = 0x1B;
  static const uint8_t kActionIdBoot_ = 0x01;
  static const uint8_t kParamIdMountingQuaternion_ = 0x06;
  static const uint8_t kParamRegion_ = 0x01;
  static const uint8_t kParamSubscription = 0x12;

  // Override functions
  int ParseCommand(const int cmd_size) override;
  int GenerateTelemetry() override;

  int AnalyzeCmdBoot(std::vector<uint8_t> decoded_rx);
  int AnalyzeCmdRequestTlm(std::vector<uint8_t> decoded_rx);
  int AnalyzeTlmId(const uint8_t tlm_id);

  int GenerateTelemetryRunTime(const uint8_t cmd_reply_tlm);
  int GenerateTelemetryQuaternion(const uint8_t cmd_reply_tlm);
  int GenerateTelemetryTemperature(const uint8_t cmd_reply_tlm);

  int EncodeSLIP(unsigned char *tlm, const int cmd_data_len);
};