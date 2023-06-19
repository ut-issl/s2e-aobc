#ifndef S2E_AOBC_COMPONENT_AOCS_STIM210_HPP_
#define S2E_AOBC_COMPONENT_AOCS_STIM210_HPP_

#include <components/base/uart_communication_with_obc.hpp>
#include <components/real/aocs/gyro_sensor.hpp>

#include "../../library/crc.hpp"

/* References
Documents: https://www.sensonor.com/products/gyro-modules/stim210/
*/

class STIM210 : public GyroSensor, public UartCommunicationWithObc {
 public:
  STIM210(GyroSensor gyro, double compo_step_sec, const int sils_port_id, OnBoardComputer *obc);
  STIM210(GyroSensor gyro, double compo_step_sec, const int sils_port_id, OnBoardComputer *obc, const unsigned int hils_port_id,
          const unsigned int baud_rate, HilsPortManager *hils_port_manager);

  // Override
  void MainRoutine(int count) override;
  std::string GetLogHeader() const override;

 private:
  libra::Vector<kGyroDimension> temperature_c_degC_{30.0};
  libra::Vector<kGyroDimension> angular_velocity_c_rads_{0.0};
  libra::Vector<kGyroDimension> angle_c_rad_{0.0};
  unsigned char status_ = 0;
  unsigned char counter_ = 0;  // 内部サンプル周波数の2000Hzでインクリメントされる
  unsigned char latency_ = 0;

  double compo_step_sec_ = 5.0e-4;  // 2000Hz

  /**
   * @enum   OPERATION_MODE
   * @brief  STIM210の運用モード
   * @note   uint8_tを想定
   */
  typedef enum { OPERATION_INIT_MODE = 0, OPERATION_NORMAL_MODE, OPERATION_SERVICE_MODE, OPERATION_MODE_MAX } OPERATION_MODE;

  /**
   * @enum   NORMAL_MODE_FORMAT
   * @brief  ノーマルモードの出力設定
   * @note   uint8_tを想定
   */
  typedef enum {
    NORMAL_MODE_STANDARD = 0,
    NORMAL_MODE_EXTENDED,
    NORMAL_MODE_RATE_TEMPERATURE,
    NORMAL_MODE_RATE_COUNT,
    NORMAL_MODE_RATE_LATENCY,
    NORMAL_MODE_RATE_COUNT_LATENCY,
    NORMAL_MODE_RATE_TEMPERATURE_COUNT,
    NORMAL_MODE_RATE_TEMPERATURE_LATENCY,
    NORMAL_MODE_RATE_TEMPERATURE_COUNT_LATENCY,
    NORMAL_MODE_MAX
  } NORMAL_MODE_FORMAT;

  /**
   * @enum   GYRO_OUTPUT_MODE
   * @brief  ジャイロ出力モード
   * @note   uint8_tを想定
   */
  typedef enum {
    GYRO_OUTPUT_ANGULAR_RATE = 0,
    GYRO_OUTPUT_INCREMENTAL_ANGLE,
    GYRO_OUTPUT_AVERAGE_ANGULAR_RATE,
    GYRO_OUTPUT_INTEGRATED_ANGLE,
    GYRO_OUTPUT_MODE_MAX
  } GYRO_OUTPUT_MODE;

  /**
   * @enum   TERMINATION_MODE
   * @brief  終端子の設定（0: OFF, 1: CRLF）
   * @note   uint8_tを想定
   */
  typedef enum { TERMINATION_OFF = 0, TERMINATION_CRLF, TERMINATION_MODE_MAX } TERMINATION_MODE;

  /**
   * @enum   LPF
   * @brief  ジャイロ出力ローパスフィルタの指定可能なカットオフ周波数
   * @note   uint8_tを想定
   */
  typedef enum { LPF_16HZ = 0, LPF_33HZ, LPF_66HZ, LPF_131HZ, LPF_262HZ, LPF_MAX } LPF;

  /**
   * @enum   SAMPLE_RATE
   * @brief  ジャイロ出力のサンプルレート
   * @note   uint8_tを想定
   */
  typedef enum {
    SAMPLE_RATE_EXTERNAL_TRIGGER = 0,
    SAMPLE_RATE_125HZ,
    SAMPLE_RATE_250HZ,
    SAMPLE_RATE_500HZ,
    SAMPLE_RATE_1000HZ,
    SAMPLE_RATE_2000HZ,
    SAMPLE_RATE_MAX
  } SAMPLE_RATE;

  OPERATION_MODE operation_mode_ = OPERATION_INIT_MODE;
  NORMAL_MODE_FORMAT normal_mode_format_ = NORMAL_MODE_STANDARD;
  GYRO_OUTPUT_MODE omega_mode_ = GYRO_OUTPUT_ANGULAR_RATE;
  TERMINATION_MODE termination_mode_ = TERMINATION_OFF;
  LPF lpf_freq_ = LPF_262HZ;
  SAMPLE_RATE sample_rate_mode_ = SAMPLE_RATE_2000HZ;

  const uint8_t normal_mode_format_idx_[STIM210::NORMAL_MODE_MAX] = {0x90, 0x92, 0xa0, 0xa2, 0xa4,
                                                                     0xa5, 0x99, 0xa6, 0xa8};  //!< ノーマルモードでのテレメの1byte目に含まれるidx
  const uint32_t sample_rate_hz_[STIM210::SAMPLE_RATE_MAX] = {1, 125, 250, 500, 1000, 2000};

  const unsigned char termination_cr = 0x0d;
  const int kMaxRxSize = 12;
  const int kMaxTxSize = 30;
  const uint8_t kCrcInitial = 0xff;
  const bool kRevFlag = false;

  // Override functions
  int ParseCommand(const int cmd_size) override;
  int GenerateTelemetry() override;

  // TLM
  int32_t ConvertOmega2Tlm(double omega);
  int16_t ConvertTemp2Tlm(double temp);

  // UART telemetry from STIM210
  int GenerateNormalModeTlm();
  void GenerateFormatTlm(int &offset);
  void GenerateOmegaTlm(int &offset);
  void GenerateStatusTlm(int &offset);
  void GenerateBufferTlm(int &offset);
  void GenerateTemperatureTlm(int &offset);
  void GenerateCountTlm(int &offset);
  void GenerateLatencyTlm(int &offset);
  void GenerateCRCTlm(int &offset);
  void GenerateTerminationTlm(int &offset);
  void SetTlm(std::vector<unsigned char> tlm, int &offset, size_t tlm_size);

  // CMD
  int AnalyzeCmdServiceMode(std::vector<unsigned char> cmd);
  int AnalyzeCmdNormalMode(std::vector<unsigned char> cmd);
  int AnalyzeCmdTermination(std::vector<unsigned char> cmd);
  int AnalyzeCmdSetNormalModeFormat(std::vector<unsigned char> cmd);
  int AnalyzeCmdSetSampleRate(std::vector<unsigned char> cmd);
  int AnalyzeCmdSetOmegaMode(std::vector<unsigned char> cmd);
  int AnalyzeCmdSetLPFFrequency(std::vector<unsigned char> cmd);
};
#endif  // S2E_AOBC_COMPONENT_AOCS_STIM210_HPP_
