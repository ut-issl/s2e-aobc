/**
 * @file sagitta.hpp
 * @brief Class to emulate Sagitta star sensor
 * @note Manual: NA
 */

#ifndef S2E_AOBC_COMPONENT_AOCS_SAGITTA_HPP_
#define S2E_AOBC_COMPONENT_AOCS_SAGITTA_HPP_

#include <components/base/uart_communication_with_obc.hpp>
#include <components/real/aocs/star_sensor.hpp>

#include "../../library/xxhash32.hpp"

/**
 * @class Sagitta
 * @brief Class to emulate Sagitta star sensor
 */
class Sagitta : public StarSensor, public UartCommunicationWithObc {
 public:
  /**
   * @fn Sagitta
   * @brief Constructor with HILS
   * @param [in] stt: Star sensor setting
   * @param [in] sils_port_id: Port ID for SILS
   * @param [in] obc: Connected OBC
   */
  Sagitta(StarSensor stt, const int sils_port_id, OnBoardComputer *obc);
  /**
   * @fn Sagitta
   * @brief Constructor with HILS
   * @param [in] stt: Star sensor setting
   * @param [in] sils_port_id: Port ID for SILS
   * @param [in] obc: Connected OBC
   * @param [in] hils_port_id: Port ID for HILS
   * @param [in] baud_rate: UART baud rate
   * @param [in] hils_port_manager: HILS port manager
   */
  Sagitta(StarSensor stt, const int sils_port_id, OnBoardComputer *obc, const unsigned int hils_port_id, const unsigned int baud_rate,
          HilsPortManager *hils_port_manager);

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine for sensor observation
   */
  void MainRoutine(int count) override;

 private:
  /**
   * @enum REC_MODE
   * @brief Recode mode
   */
  typedef enum {
    REC_MODE_RUN_TIME = 0,
    REC_MODE_TEMPERATURE,
    REC_MODE_QUATERNION,
    REC_MODE_OTHERS,
    REC_MODE_MAX,
  } REC_MODE;

  /**
   * @enum TLM_REPLY_MODE
   * @brief Telemetry reply mode
   */
  typedef enum {
    TLM_REPLY_MODE_SYNCHRONOUS = 0,
    TLM_REPLY_MODE_ASYNCHRONOUS,
    TLM_REPLY_MODE_MAX,
  } TLM_REPLY_MODE;

  bool is_initialized_ = 0;                                     //!< Initialized flag
  uint32_t runtime_ms_ = 0;                                     //!< Total running time [ms]
  double compo_step_sec_ = 0.0;                                 //!< Component execution step [s]
  REC_MODE rec_mode_ = REC_MODE_OTHERS;                         //!< Recode mode
  TLM_REPLY_MODE tlm_reply_mode_ = TLM_REPLY_MODE_SYNCHRONOUS;  //!< Telemetry reply mode
  float q_tlm_i2c_[4] = {1.0f, 0.0f, 0.0f, 0.0f};               //!< Measured quaternion
  float mcu_temperature_degC_ = 25.0f;                          //!< Temperature of the micro controller
  float cmos_temperature_degC_ = 23.0f;                         //!< Temperature of the image sensor
  float fpga_temperature_degC_ = 24.0f;                         //!< Temperature of the FPGA
  uint8_t status_ = 1;                                          //!< Status of telemetry (0:OK, 1:not implemented, 2-255:tlm error)
  uint32_t counter_ = 0;                                        //!< Internal counter
  uint64_t unix_time_us_ = 0;                                   //!< Telemetry dummy data [microseconds] Ref. 2.4.6

  //!< Constants
  static const unsigned int kMaxCmdSize_ = 80;      //!< Max command size
  static const unsigned int kMaxTlmSize_ = 128;     //!< Max telemetry size
  static const unsigned int kHeaderSize_ = 1;       //!< Header size
  static const unsigned int kFooterSize_ = 1;       //!< Footer size
  static const unsigned int kMaxSubscRegSize_ = 2;  //!< Subscription registry size
  static const unsigned int kXxhashSize_ = 4;       //!< xxhash size
  static const unsigned int kXxhashSeed_ = 1425;    //!< xxhash seed
  static const uint8_t kHeader_ = 0xc0;             //!< Header
  static const uint8_t kFooter_ = 0xc0;             //!< Footer
  bool is_subscribed_temperature_ = 0;              //!< Flag to send temperature telemetry in asynchronous mode
  bool is_subscribed_quaternion_ = 0;               //!< Flag to send quaternion telemetry in asynchronous mode
  uint8_t tlm_counter_ = 0;                         //!< Counter to send asynchronous telemetry
  static const uint8_t kAddress_ = 0x21;            //!< RS-485 Address of STT

  // Command
  static const uint8_t kCmdSetParam_ = 0x00;         //!< Set parameter command ID
  static const uint8_t kCmdRequestParam_ = 0x01;     //!< Request parameter command ID
  static const uint8_t kCmdRequestTelem_ = 0x02;     //!< Request telemetry command ID
  static const uint8_t kCmdReplyTelemSync_ = 0x82;   //!< Reply synchronous telemetry command ID
  static const uint8_t kCmdReplyTelemAsync_ = 0x84;  //!< Reply asynchronous telemetry command ID
  static const uint8_t kCmdAction_ = 0x03;           //!< Action command ID

  // Telemetry
  static const uint8_t kTelemIdTime_ = 0x01;                //!< Time telemetry ID
  static const uint8_t kTelemIdSolution_ = 0x18;            //!< Solution telemetry ID
  static const uint8_t kTelemIdTemperature_ = 0x1B;         //!< Temperature telemetry ID
  static const uint8_t kActionIdBoot_ = 0x01;               //!< Boot action ID
  static const uint8_t kParamIdMountingQuaternion_ = 0x06;  //!< Mounting quaternion parameter ID
  static const uint8_t kParamRegion_ = 0x01;                //!< Region parameter ID
  static const uint8_t kParamSubscription = 0x12;           //!< Subscription parameter ID

  // Override functions
  /**
   * @fn ParseCommand
   * @brief Override function of UartCommunicationWithObc
   */
  int ParseCommand(const int command_size) override;
  /**
   * @fn GenerateTelemetry
   * @brief Override function of UartCommunicationWithObc
   */
  int GenerateTelemetry() override;

  /**
   * @fn AnalyzeCmdBoot
   * @brief Analyze Boot command
   * @param [in] decoded_rx: Decoded command data
   * @return 1: Success, -1: Error
   */
  int AnalyzeCmdBoot(std::vector<uint8_t> decoded_rx);
  /**
   * @fn AnalyzeCmdRequestTlm
   * @brief Analyze request telemetry command
   * @param [in] decoded_rx: Decoded command data
   * @return 1: Success, -1: Error
   */
  int AnalyzeCmdRequestTlm(std::vector<uint8_t> decoded_rx);
  /**
   * @fn AnalyzeTlmId
   * @brief Analyze telemetry ID
   * @param [in] tlm_id: Telemetry ID
   * @return 1: Success, -1: Error
   */
  int AnalyzeTlmId(const uint8_t tlm_id);

  /**
   * @fn GenerateTelemetryRunTime
   * @brief Generate runtime telemetry
   * @param [in] cmd_reply_tlm: Replay telemetry ID
   * @return Encoded telemetry size
   */
  int GenerateTelemetryRunTime(const uint8_t cmd_reply_tlm);
  /**
   * @fn GenerateTelemetryQuaternion
   * @brief Generate quaternion telemetry
   * @param [in] cmd_reply_tlm: Replay telemetry ID
   * @return Encoded telemetry size
   */
  int GenerateTelemetryQuaternion(const uint8_t cmd_reply_tlm);
  /**
   * @fn GenerateTelemetryTemperature
   * @brief Generate temperature telemetry
   * @param [in] cmd_reply_tlm: Replay telemetry ID
   * @return Encoded telemetry size
   */
  int GenerateTelemetryTemperature(const uint8_t cmd_reply_tlm);

  /**
   * @fn EncodeSLIP
   * @brief Not used? FIXME: Delete
   */
  int EncodeSLIP(unsigned char *tlm, const int cmd_data_len);
};

#endif  // S2E_AOBC_COMPONENT_AOCS_SAGITTA_HPP_
