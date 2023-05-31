#pragma once
#include <components/base/uart_communication_with_obc.hpp>
#include <components/real/aocs/gnss_receiver.hpp>

/* References
Manual: NA
HowToUse: NA
*/

#define OEM7600_MAX_TLM_LIST 6  // TBD(modified in the future as needed)
#define OEM7600_MAX_CMD_ARG 8   // TBD(modified in the future as needed)

// OEM7600 command structure
typedef struct {
  std::string cmd_name;
  std::string cmd_args[OEM7600_MAX_CMD_ARG];
  bool is_valid;
} OEM7600_CMD;

// OEM7600 telemetry ID for Binary format
typedef enum {
  // binary format系列
  OEM7600_TLM_ID_BEST_XYZ = 0x00F1,
  OEM7600_TLM_ID_HARDWARE_MONITOR = 0x03C3,
  OEM7600_TLM_ID_ERROR = 0x0000
} OEM7600_BINARY_TLM_ID;

// OEM7600 telemetry structure
typedef struct {
  std::string tlm_name = "";
  std::string out_type = "once";
  unsigned int output_freq = 0;
  unsigned int count_to_out = 0;

  // function for adjust tlm output timing
  void count_up() {
    count_to_out++;
    if (count_to_out >= output_freq) {
      count_to_out = 0;
    }
  }
} OEM7600_TLM;

class OEM7600 : public GnssReceiver, public UartCommunicationWithObc {
 public:
  OEM7600(GnssReceiver gnssr, const int sils_port_id, OnBoardComputer *obc, const unsigned char oem_tlm_ch);
  OEM7600(GnssReceiver gnssr, const int sils_port_id, OnBoardComputer *obc, const unsigned char oem_tlm_ch, const unsigned int hils_port_id,
          const unsigned int baud_rate, HilsPortManager *hils_port_manager);

  // Override: GNSSReceiver functions
  void MainRoutine(int count) override;
  std::string GetLogHeader() const override;

 private:
  const unsigned char oem_tlm_ch_;
  std::vector<OEM7600_TLM> tlm_list_;
  double last_position_fix_time_local_ = 0;  // to be removed after core_oss modification

  // Override functions
  int ParseCommand(const int cmd_size) override;
  int GenerateTelemetry() override;

  // General function
  void Update_local();  // update positioning infromation(to be removed after
                        // core_oss modification)

  // TLM Packet Generator
  std::string Gen_TLM_Packet(const std::string tlm_name);  // generate tlm body

  // TLM Packet Generator in Ascii format
  std::string Gen_TLM_Header_Ascii(const std::string tlm_name);  // generate tlm header
  std::string Gen_BestXYZTlm_Ascii(void);                        // function correspond to "bestxyza" tlm
  std::string Gen_TimeTlm_Ascii(void);                           // function correspond to "timea" tlm
  std::string Gen_HWMonitorTlm_Ascii(void);                      // function correspond to "hwmonitora" tlm
  std::string Gen_GPGGATlm(void);                                // function correspond to "gpgga" tlm

  // TLM Packet Generator in Binary format
  std::string Gen_TLM_Header_Binary(const std::string tlm_name);  // generate tlm header
  std::string Gen_BestXYZTlm_Binary(void);                        // function correspond to "bestxyzb" tlm
  std::string Gen_HWMonitorTlm_Binary(void);                      // function correspond to "hwmonitorb" tlm

  // CMD
  OEM7600_CMD DecodeCommand();          // command decoder
  int Cmd_LOG(const std::string comport_ID, const std::string tlm_name, const std::string type,
              const std::string freq);  // decoder correspond to "log" command

  // CRC Calculation
  unsigned int OEM7600_calculate_crc32(const char *tlm_data_for_crc, const size_t data_length);
  unsigned int OEM7600_calculate_crc32_subroutine(const unsigned int initial_value);

  // Other utility functions for tlm/cmd string manipuration
  bool TLM_NameSearch(const std::string tlm_name);                        // check wether the designated tlm name in
                                                                          // "log" cmd is valid or not
  OEM7600_BINARY_TLM_ID GetTlmIdOfBinaryTlm(const std::string tlm_name);  // get tlm id for binary format tlm packet
  unsigned short GetTlmLengthOfBinaryTlm(const std::string tlm_name);     // get tlm length of variable part for binary format tlm

  std::string convLatLontoNMEA(const double rad,
                               const std::string type);                      // convert lat/lon in [rad] to NMEA format
  std::string string_zeropad_local(const std::string str_org,
                                   const unsigned int len);                  // zero padding(to be removed and improved after
                                                                             // core_oss modification)
  std::vector<unsigned char> ConvertDoubleToByte(const double double_data);  // convert position,velocity data in double
                                                                             // format into byte format
  std::vector<unsigned char> ConvertFloatToByte(const float float_data);     // convert position,velocity data in float format
                                                                             // into byte format

  // TLM name list (additinoal tlm names will be added in the future as needed)
  const std::string tlm_name_dictionary_[OEM7600_MAX_TLM_LIST] = {"bestxyza", "timea", "gpgga", "hwmonitora", "bestxyzb", "hwmonitorb"};
};
