/**
 * @file stim210.hpp
 * @brief Class to emulate STIM210 gyro sensor
 * @note Manual: https://www.sensonor.com/products/gyro-modules/stim210/
 */

#ifndef S2E_AOBC_COMPONENT_AOCS_STIM210_HPP_
#define S2E_AOBC_COMPONENT_AOCS_STIM210_HPP_

#include <components/base/uart_communication_with_obc.hpp>
#include <components/real/aocs/gyro_sensor.hpp>

#include "../../library/crc.hpp"

/**
 * @class STIM210
 * @brief Class to emulate STIM210 gyro sensor
 */
class STIM210 : public GyroSensor, public UartCommunicationWithObc {
 public:
  /**
   * @fn STIM210
   * @brief Constructor
   * @param [in] gyro: Gyro sensor setting
   * @param [in] compo_step_sec: Component update step [s]
   * @param [in] sils_port_id: Port ID for SILS
   * @param [in] obc: Connected OBC
   */
  STIM210(GyroSensor gyro, double compo_step_sec, const int sils_port_id, OnBoardComputer *obc);
  /**
   * @fn STIM210
   * @brief Constructor with HILS
   * @param [in] gyro: Gyro sensor setting
   * @param [in] compo_step_sec: Component update step [s]
   * @param [in] sils_port_id: Port ID for SILS
   * @param [in] obc: Connected OBC
   * @param [in] hils_port_id: Port ID for HILS
   * @param [in] baud_rate: UART baud rate
   * @param [in] hils_port_manager: HILS port manager
   */
  STIM210(GyroSensor gyro, double compo_step_sec, const int sils_port_id, OnBoardComputer *obc, const unsigned int hils_port_id,
          const unsigned int baud_rate, HilsPortManager *hils_port_manager);

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine for sensor observation
   */
  void MainRoutine(int count) override;
  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  std::string GetLogHeader() const override;

 private:
  libra::Vector<kGyroDimension> temperature_c_degC_{30.0};      //!< Measured temperature for each component axis [degC]
  libra::Vector<kGyroDimension> angular_velocity_c_rads_{0.0};  //!< Measured angular velocity at component frame [rad/s]
  libra::Vector<kGyroDimension> angle_c_rad_{0.0};              //!< Accumulated angle at component frame [rad]
  unsigned char status_ = 0;                                    //!< Status
  unsigned char counter_ = 0;                                   //!< Internal counter increment by 2000Hz
  unsigned char latency_ = 0;                                   //!< Latency information
  double compo_step_sec_ = 5.0e-4;                              //!< Internal component update frequency 2000Hz

  /**
   * @enum   OPERATION_MODE
   * @brief  STIM210 Operation mode
   */
  typedef enum { OPERATION_INIT_MODE = 0, OPERATION_NORMAL_MODE, OPERATION_SERVICE_MODE, OPERATION_MODE_MAX } OPERATION_MODE;

  /**
   * @enum   NORMAL_MODE_FORMAT
   * @brief  Output setting in normal mode
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
   * @brief  Output mode of gyro measurement result
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
   * @brief  Termination setting（0: OFF, 1: CRLF）
   */
  typedef enum { TERMINATION_OFF = 0, TERMINATION_CRLF, TERMINATION_MODE_MAX } TERMINATION_MODE;

  /**
   * @enum   LPF
   * @brief  Cutoff frequency of internal low pass filter
   */
  typedef enum { LPF_16HZ = 0, LPF_33HZ, LPF_66HZ, LPF_131HZ, LPF_262HZ, LPF_MAX } LPF;

  /**
   * @enum   SAMPLE_RATE
   * @brief  Sample rate of gyro measurement
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

  OPERATION_MODE operation_mode_ = OPERATION_INIT_MODE;           //!< Operation mode
  NORMAL_MODE_FORMAT normal_mode_format_ = NORMAL_MODE_STANDARD;  //!< Normal mode telemetry format
  GYRO_OUTPUT_MODE omega_mode_ = GYRO_OUTPUT_ANGULAR_RATE;        //!< Measurement result output mode
  TERMINATION_MODE termination_mode_ = TERMINATION_OFF;           //!< Termination mode
  LPF lpf_freq_ = LPF_262HZ;                                      //!< Internal low pass filter setting
  SAMPLE_RATE sample_rate_mode_ = SAMPLE_RATE_2000HZ;             //!< Sample rate

  const uint8_t normal_mode_format_idx_[STIM210::NORMAL_MODE_MAX] = {0x90, 0x92, 0xa0, 0xa2, 0xa4,
                                                                     0xa5, 0x99, 0xa6, 0xa8};  //!< Telemetry header in normal mode
  const uint32_t sample_rate_hz_[STIM210::SAMPLE_RATE_MAX] = {1, 125, 250, 500, 1000, 2000};   //!< Sample rate list [Hz]

  const unsigned char termination_cr = 0x0d;  //!< Termination of CR
  const int kMaxRxSize = 12;                  //!< Max RX size (command)
  const int kMaxTxSize = 30;                  //!< Max TX size  (telemetry)
  const uint8_t kCrcInitial = 0xff;           //!< CRC initial value
  const bool kRevFlag = false;                //!< CRC reverse flag

  // Override functions for UartCommunicationWithObc
  /**
   * @fn ParseCommand
   * @brief Parse command
   * @param [in] command_size: Command size
   */
  int ParseCommand(const int command_size) override;
  /**
   * @fn GenerateTelemetry
   * @brief Generate telemetry
   */
  int GenerateTelemetry() override;

  // TLM
  /**
   * @fn ConvertOmega2Tlm
   * @brief Convert measured angular velocity value to telemetry
   * @param [in] omega: Measured angular velocity [rad/s]
   * @return Converted telemetry data
   */
  int32_t ConvertOmega2Tlm(double omega);
  /**
   * @fn ConvertTemp2Tlm
   * @brief Convert measured temperature value to telemetry
   * @param [in] temp: Measured temperature [degC]
   * @return Converted telemetry data
   */
  int16_t ConvertTemp2Tlm(double temp);

  // Telemetry
  /**
   * @fn GenerateNormalModeTlm
   * @brief Generate normal mode telemetry
   * @return Telemetry length
   */
  int GenerateNormalModeTlm();
  /**
   * @fn GenerateFormatTlm
   * @brief Generate format telemetry
   * @param [in/out] offset: Start position of telemetry(input) and end position of telemetry(output)
   */
  void GenerateFormatTlm(int &offset);
  /**
   * @fn GenerateOmegaTlm
   * @brief Generate angular velocity telemetry
   * @param [in/out] offset: Start position of telemetry(input) and end position of telemetry(output)
   */
  void GenerateOmegaTlm(int &offset);
  /**
   * @fn GenerateStatusTlm
   * @brief Generate status telemetry
   * @param [in/out] offset: Start position of telemetry(input) and end position of telemetry(output)
   */
  void GenerateStatusTlm(int &offset);
  /**
   * @fn GenerateBufferTlm
   * @brief Generate buffer telemetry
   * @param [in/out] offset: Start position of telemetry(input) and end position of telemetry(output)
   */
  void GenerateBufferTlm(int &offset);
  /**
   * @fn GenerateTemperatureTlm
   * @brief Generate temperature telemetry
   * @param [in/out] offset: Start position of telemetry(input) and end position of telemetry(output)
   */
  void GenerateTemperatureTlm(int &offset);
  /**
   * @fn GenerateCountTlm
   * @brief Generate count telemetry
   * @param [in/out] offset: Start position of telemetry(input) and end position of telemetry(output)
   */
  void GenerateCountTlm(int &offset);
  /**
   * @fn GenerateLatencyTlm
   * @brief Generate latency telemetry
   * @param [in/out] offset: Start position of telemetry(input) and end position of telemetry(output)
   */
  void GenerateLatencyTlm(int &offset);
  /**
   * @fn GenerateCRCTlm
   * @brief Generate CRC telemetry
   * @param [in/out] offset: Start position of telemetry(input) and end position of telemetry(output)
   */
  void GenerateCRCTlm(int &offset);
  /**
   * @fn GenerateTerminationTlm
   * @brief Generate termination telemetry
   * @param [in/out] offset: Start position of telemetry(input) and end position of telemetry(output)
   */
  void GenerateTerminationTlm(int &offset);

  /**
   * @fn SetTlm
   * @brief Set telemetry data
   * @param [in] tlm: Telemetry data
   * @param [in/out] offset: Start position of telemetry(input) and end position of telemetry(output)
   * @param [in] tlm_size: Telemetry data size
   */
  void SetTlm(std::vector<unsigned char> tlm, int &offset, size_t tlm_size);

  // Command
  /**
   * @fn AnalyzeCmdServiceMode
   * @brief Analyze service mode command
   * @param [in] cmd: Command data
   * @return 0: Success, -1: Error
   */
  int AnalyzeCmdServiceMode(std::vector<unsigned char> cmd);
  /**
   * @fn AnalyzeCmdNormalMode
   * @brief Analyze normal mode command
   * @param [in] cmd: Command data
   * @return 0: Success, -1: Error
   */
  int AnalyzeCmdNormalMode(std::vector<unsigned char> cmd);
  /**
   * @fn AnalyzeCmdTermination
   * @brief Analyze termination setting command
   * @param [in] cmd: Command data
   * @return 0: Success, -1: Error
   */
  int AnalyzeCmdTermination(std::vector<unsigned char> cmd);
  /**
   * @fn AnalyzeCmdSetNormalModeFormat
   * @brief Analyze normal mode format setting command
   * @param [in] cmd: Command data
   * @return 0: Success, -1: Error
   */
  int AnalyzeCmdSetNormalModeFormat(std::vector<unsigned char> cmd);
  /**
   * @fn AnalyzeCmdSetSampleRate
   * @brief Analyze sample rate setting command
   * @param [in] cmd: Command data
   * @return 0: Success, -1: Error
   */
  int AnalyzeCmdSetSampleRate(std::vector<unsigned char> cmd);
  /**
   * @fn AnalyzeCmdSetOmegaMode
   * @brief Analyze angular rate output setting command
   * @param [in] cmd: Command data
   * @return 0: Success, -1: Error
   */
  int AnalyzeCmdSetOmegaMode(std::vector<unsigned char> cmd);
  /**
   * @fn AnalyzeCmdSetLPFFrequency
   * @brief Analyze low pass filter cutoff frequency setting command
   * @param [in] cmd: Command data
   * @return 0: Success, -1: Error
   */
  int AnalyzeCmdSetLPFFrequency(std::vector<unsigned char> cmd);
};

#endif  // S2E_AOBC_COMPONENT_AOCS_STIM210_HPP_
