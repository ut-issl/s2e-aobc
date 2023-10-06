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
 * @struct OEM7600_CMD
 * @brief OEM7600 command structure
 */
typedef struct {
  std::string cmd_name;                       //!< Command name
  std::string cmd_args[OEM7600_MAX_CMD_ARG];  //!< Command argument list
  bool is_valid;                              //!< Command validation flag
} OEM7600_CMD;

/**
 * @enum OEM7600_BINARY_TLM_ID
 * @brief OEM7600 telemetry ID for Binary format
 */
typedef enum { OEM7600_TLM_ID_BEST_XYZ = 0x00F1, OEM7600_TLM_ID_HARDWARE_MONITOR = 0x03C3, OEM7600_TLM_ID_ERROR = 0x0000 } OEM7600_BINARY_TLM_ID;

/**
 * @struct OEM7600_TLM
 * @brief OEM7600 telemetry structure
 */
typedef struct {
  std::string tlm_name = "";      //!< Telemetry name
  std::string out_type = "once";  //!< Output type
  unsigned int output_freq = 0;   //!< Output frequency
  unsigned int count_to_out = 0;  //!< Count to output

  /**
   * @fn count_up
   * @brief Function to adjust telemetry output timing
   */
  void count_up() {
    count_to_out++;
    if (count_to_out >= output_freq) {
      count_to_out = 0;
    }
  }
} OEM7600_TLM;

/**
 * @class OEM7600
 * @brief Class to emulate EM7600 GNSS receiver
 */
class Oem7600 : public GnssReceiver, public UartCommunicationWithObc {
 public:
  /**
   * @fn Oem7600
   * @brief Constructor
   * @param [in] gnss_receiver: GNSS-R setting
   * @param [in] sils_port_id: Port ID for SILS
   * @param [in] obc: Connected OBC
   * @param [in] telemetry_channel: OEM7600 telemetry channel
   */
  Oem7600(GnssReceiver gnss_receiver, const int sils_port_id, OnBoardComputer *obc, const unsigned char telemetry_channel);
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
  Oem7600(GnssReceiver gnss_receiver, const int sils_port_id, OnBoardComputer *obc, const unsigned char telemetry_channel,
          const unsigned int hils_port_id, const unsigned int baud_rate, HilsPortManager *hils_port_manager);

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
  const unsigned char telemetry_channel_;    //!< OEM7600 telemetry channel
  std::vector<OEM7600_TLM> tlm_list_;        //!< Telemetry list
  double last_position_fix_time_local_ = 0;  //!< Last position fix time TODO: removed after s2e-core modification

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
   * @fn Update_local
   * @brief Update positioning information (TODO: removed after s2e-core modification)
   */
  void Update_local();

  // Telemetry
  /**
   * @fn Gen_TLM_Packet
   * @brief Generate telemetry body
   * @param [in] tlm_name: Telemetry name
   */
  std::string Gen_TLM_Packet(const std::string tlm_name);
  /**
   * @fn Gen_TLM_Header_Ascii
   * @brief Generate telemetry header with ASCII format
   * @param [in] tlm_name: Telemetry name
   */
  std::string Gen_TLM_Header_Ascii(const std::string tlm_name);
  /**
   * @fn Gen_BestXYZTlm_Ascii
   * @brief Generate "bestxyza" tlm with ASCII format
   */
  std::string Gen_BestXYZTlm_Ascii(void);
  /**
   * @fn Gen_BestXYZTlm_Ascii
   * @brief Generate "timea" tlm with ASCII format
   */
  std::string Gen_TimeTlm_Ascii(void);
  /**
   * @fn Gen_HWMonitorTlm_Ascii
   * @brief Generate "hwmonitora" tlm with ASCII format
   */
  std::string Gen_HWMonitorTlm_Ascii(void);
  /**
   * @fn Gen_GPGGATlm
   * @brief Generate "gpgga" tlm
   */
  std::string Gen_GPGGATlm(void);
  /**
   * @fn Gen_TLM_Header_Binary
   * @brief Generate telemetry header with binary format
   * @param [in] tlm_name: Telemetry name
   */
  std::string Gen_TLM_Header_Binary(const std::string tlm_name);
  /**
   * @fn Gen_BestXYZTlm_Binary
   * @brief Generate "bestxyza" tlm with binary format
   */
  std::string Gen_BestXYZTlm_Binary(void);
  /**
   * @fn Gen_HWMonitorTlm_Binary
   * @brief Generate "hwmonitora" tlm with binary format
   */
  std::string Gen_HWMonitorTlm_Binary(void);

  // CMD
  /**
   * @fn DecodeCommand
   * @brief Command decoder
   */
  OEM7600_CMD DecodeCommand();
  /**
   * @fn Cmd_LOG
   * @brief Decoder correspond to "log" command
   */
  int Cmd_LOG(const std::string comport_ID, const std::string tlm_name, const std::string type, const std::string freq);

  /**
   * @fn OEM7600_calculate_crc32
   * @brief CRC calculation TODO: Move to s2e-core's library
   */
  unsigned int OEM7600_calculate_crc32(const char *tlm_data_for_crc, const size_t data_length);
  /**
   * @fn OEM7600_calculate_crc32_subroutine
   * @brief CRC calculation TODO: Move to s2e-core's library
   */
  unsigned int OEM7600_calculate_crc32_subroutine(const unsigned int initial_value);

  /**
   * @fn TLM_NameSearch
   * @brief Check wether the designated tlm name in "log" cmd is valid or not
   * @param [in] tlm_name: Telemetry name
   * @return Valid or not
   */
  bool TLM_NameSearch(const std::string tlm_name);
  /**
   * @fn GetTlmIdOfBinaryTlm
   * @brief Get tlm id for binary format tlm packet
   * @param [in] tlm_name: Telemetry name
   * @return telemetry ID
   */
  OEM7600_BINARY_TLM_ID GetTlmIdOfBinaryTlm(const std::string tlm_name);
  /**
   * @fn GetTlmLengthOfBinaryTlm
   * @brief Get tlm length of variable part for binary format tlm
   * @param [in] tlm_name: Telemetry name
   * @return Telemetry length
   */
  unsigned short GetTlmLengthOfBinaryTlm(const std::string tlm_name);

  /**
   * @fn convLatLontoNMEA
   * @brief Convert lat/lon in [rad] to NMEA format
   * @param [in] rad: latitude or longitude
   * @param [in] type: lat or lon
   * @return Converted result
   */
  std::string convLatLontoNMEA(const double rad, const std::string type);

  /**
   * @fn string_zeropad_local
   * @brief Add zero for padding
   * @param [in] str_org: input string
   * @param [in] len: length of output
   * @return String with zero padding
   */
  std::string string_zeropad_local(const std::string str_org, const unsigned int len);

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

  const std::string tlm_name_dictionary_[OEM7600_MAX_TLM_LIST] = {"bestxyza",   "timea",    "gpgga",
                                                                  "hwmonitora", "bestxyzb", "hwmonitorb"};  //!< Telemetry name list
};

#endif  // S2E_AOBC_COMPONENT_AOCS_OEM7600_HPP_
