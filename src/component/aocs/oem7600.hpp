/**
 * @file oem7600.hpp
 * @brief Class to emulate OEM7600 GNSS receiver
 * @note Manual: https://docs.novatel.com/OEM7/Content/Commands/CommandsLogs.htm?tocpath=Commands%20%2526%20Logs%7C_____0
 */

#ifndef S2E_AOBC_COMPONENT_AOCS_OEM7600_HPP_
#define S2E_AOBC_COMPONENT_AOCS_OEM7600_HPP_

#include <components/base/uart_communication_with_obc.hpp>
#include <components/real/aocs/gnss_receiver.hpp>

#define OEM7600_MAX_TLM_LIST 6  //!< Max telemetry list TODO: modified in the future as needed
#define OEM7600_MAX_CMD_ARG 8   //!< Max command argument number TODO: modified in the future as needed

/**
 * @struct Oem7600Command
 * @brief OEM7600 command structure
 */
typedef struct {
  std::string command_name;                       //!< Command name
  std::string command_args[OEM7600_MAX_CMD_ARG];  //!< Command argument list
  bool is_valid;                                  //!< Command validation flag
} Oem7600Command;

/**
 * @enum OEM7600_BINARY_TLM_ID
 * @brief OEM7600 telemetry ID for Binary format
 */
typedef enum { OEM7600_TLM_ID_BEST_XYZ = 0x00F1, OEM7600_TLM_ID_HARDWARE_MONITOR = 0x03C3, OEM7600_TLM_ID_ERROR = 0x0000 } OEM7600_BINARY_TLM_ID;

/**
 * @struct Oem7600Telemetry
 * @brief OEM7600 telemetry structure
 */
struct Oem7600Telemetry {
  std::string telemetry_name = "";    //!< Telemetry name
  std::string out_type = "once";      //!< Output type
  unsigned int output_frequency = 0;  //!< Output frequency
  unsigned int count_to_out = 0;      //!< Count to output

  /**
   * @fn count_up
   * @brief Function to adjust telemetry output timing
   */
  void count_up() {
    count_to_out++;
    if (count_to_out >= output_frequency) {
      count_to_out = 0;
    }
  }
};

/**
 * @class OEM7600
 * @brief Class to emulate EM7600 GNSS receiver
 */
class Oem7600 : public s2e::components::GnssReceiver, public s2e::components::UartCommunicationWithObc {
 public:
  /**
   * @fn Oem7600
   * @brief Constructor
   * @param [in] gnss_receiver: GNSS-R setting
   * @param [in] sils_port_id: Port ID for SILS
   * @param [in] obc: Connected OBC
   * @param [in] telemetry_channel: OEM7600 telemetry channel
   */
  Oem7600(s2e::components::GnssReceiver gnss_receiver, const int sils_port_id, s2e::components::OnBoardComputer *obc, const unsigned char telemetry_channel);
  /**
   * @fn Oem7600
   * @brief Constructor with HILS setting
   * @param [in] gnss_receiver: GNSS-R setting
   * @param [in] sils_port_id: Port ID for SILS
   * @param [in] obc: Connected OBC
   * @param [in] telemetry_channel: OEM7600 telemetry channel
   * @param [in] hils_port_id: Port ID for HILS
   * @param [in] baud_rate: UART baud rate
   * @param [in] hils_port_manager: HILS port manager
   */
  Oem7600(s2e::components::GnssReceiver gnss_receiver, const int sils_port_id, s2e::components::OnBoardComputer *obc, const unsigned char telemetry_channel,
          const unsigned int hils_port_id, const unsigned int baud_rate, s2e::simulation::HilsPortManager *hils_port_manager);

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine for sensor observation
   */
  void MainRoutine(const int time_count) override;
  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  std::string GetLogHeader() const override;

 private:
  const unsigned char telemetry_channel_;         //!< OEM7600 telemetry channel
  std::vector<Oem7600Telemetry> telemetry_list_;  //!< Telemetry list
  double last_position_fix_time_local_ = 0;       //!< Last position fix time TODO: removed after s2e-core modification

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

  /**
   * @fn UpdateLocal
   * @brief Update positioning information (TODO: removed after s2e-core modification)
   */
  void UpdateLocal();

  // Telemetry
  /**
   * @fn GenerateTelemetryPacket
   * @brief Generate telemetry body
   * @param [in] telemetry_name: Telemetry name
   */
  std::string GenerateTelemetryPacket(const std::string telemetry_name);
  /**
   * @fn GenerateTelemetryHeaderAscii
   * @brief Generate telemetry header with ASCII format
   * @param [in] telemetry_name: Telemetry name
   */
  std::string GenerateTelemetryHeaderAscii(const std::string telemetry_name);
  /**
   * @fn GenerateBestxyzTelemetryAscii
   * @brief Generate "bestxyza" tlm with ASCII format
   */
  std::string GenerateBestxyzTelemetryAscii(void);
  /**
   * @fn GenerateBestxyzTelemetryAscii
   * @brief Generate "timea" tlm with ASCII format
   */
  std::string GenerateTimeTelemetryAscii(void);
  /**
   * @fn GenerateHwmonitorTelemetryAscii
   * @brief Generate "hwmonitora" tlm with ASCII format
   */
  std::string GenerateHwmonitorTelemetryAscii(void);
  /**
   * @fn GenerateGpggaTelemetry
   * @brief Generate "gpgga" tlm
   */
  std::string GenerateGpggaTelemetry(void);
  /**
   * @fn GenerateTelemetryHeaderBinary
   * @brief Generate telemetry header with binary format
   * @param [in] telemetry_name: Telemetry name
   */
  std::string GenerateTelemetryHeaderBinary(const std::string telemetry_name);
  /**
   * @fn GenerateBestxyzTelemetryBinary
   * @brief Generate "bestxyza" tlm with binary format
   */
  std::string GenerateBestxyzTelemetryBinary(void);
  /**
   * @fn GenerateHwmonitorTelemetryBinary
   * @brief Generate "hwmonitora" tlm with binary format
   */
  std::string GenerateHwmonitorTelemetryBinary(void);

  // CMD
  /**
   * @fn DecodeCommand
   * @brief Command decoder
   */
  Oem7600Command DecodeCommand();
  /**
   * @fn CommandLog
   * @brief Decoder correspond to "log" command
   */
  int CommandLog(const std::string comport_id, const std::string telemetry_name, const std::string type, const std::string freq);

  /**
   * @fn CalculateCrc32
   * @brief CRC calculation TODO: Move to s2e-core's library
   */
  unsigned int CalculateCrc32(const char *tlm_data_for_crc, const size_t data_length);
  /**
   * @fn CalculateCrc32Subroutine
   * @brief CRC calculation TODO: Move to s2e-core's library
   */
  unsigned int CalculateCrc32Subroutine(const unsigned int initial_value);

  /**
   * @fn TelemetryNameSearch
   * @brief Check wether the designated tlm name in "log" cmd is valid or not
   * @param [in] telemetry_name: Telemetry name
   * @return Valid or not
   */
  bool TelemetryNameSearch(const std::string telemetry_name);
  /**
   * @fn GetTlmIdOfBinaryTlm
   * @brief Get tlm id for binary format tlm packet
   * @param [in] telemetry_name: Telemetry name
   * @return telemetry ID
   */
  OEM7600_BINARY_TLM_ID GetTlmIdOfBinaryTlm(const std::string telemetry_name);
  /**
   * @fn GetTlmLengthOfBinaryTlm
   * @brief Get tlm length of variable part for binary format tlm
   * @param [in] telemetry_name: Telemetry name
   * @return Telemetry length
   */
  unsigned short GetTlmLengthOfBinaryTlm(const std::string telemetry_name);

  /**
   * @fn ConvLatLonToNmea
   * @brief Convert lat/lon in [rad] to NMEA format
   * @param [in] angle_rad: latitude or longitude
   * @param [in] type: lat or lon
   * @return Converted result
   */
  std::string ConvLatLonToNmea(const double angle_rad, const std::string type);

  /**
   * @fn StringZeroPaddingLocal
   * @brief Add zero for padding
   * @param [in] str_org: input string
   * @param [in] len: length of output
   * @return String with zero padding
   */
  std::string StringZeroPaddingLocal(const std::string str_org, const unsigned int len);

  /**
   * @fn ConvertDoubleToByte
   * @brief Convert position,velocity data in double format into byte format
   * @param [in] double_data: Input value
   * @return Converted binary data
   */
  std::vector<unsigned char> ConvertDoubleToByte(const double double_data);
  /**
   * @fn ConvertFloatToByte
   * @brief Convert position,velocity data in float format into byte format
   * @param [in] float_data: Input value
   * @return Converted binary data
   */
  std::vector<unsigned char> ConvertFloatToByte(const float float_data);

  const std::string telemetry_name_dictionary_[OEM7600_MAX_TLM_LIST] = {"bestxyza",   "timea",    "gpgga",
                                                                        "hwmonitora", "bestxyzb", "hwmonitorb"};  //!< Telemetry name list
};

#endif  // S2E_AOBC_COMPONENT_AOCS_OEM7600_HPP_
