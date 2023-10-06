/**
 * @file oem7600.cpp
 * @brief Class to emulate OEM7600 GNSS receiver
 */

#include "oem7600.hpp"

#include <algorithm>  // toupper
#include <library/utilities/macros.hpp>

#define MAX_CMD_LEN 1024                        // TBD
#define MAX_TLM_LEN 1024                        // TBD
#define OEM7600_COMMON_BINARY_HEADER_SIZE (28)  // Binary format Tlmの固定長部分のテレメサイズ
#define OEM7600_BESTXYZ_BINARY_TLM_SIZE (112)   // Binary formatの位置情報テレメサイズ
#define OEM7600_HWMONITOR_BINARY_TLM_SIZE (52)  // Binary formatのhardware monitor tlm テレメサイズ

#define _USE_MATH_DEFINES
// #define TLM_FUNCTION_DEBUG_MODE

Oem7600::Oem7600(GnssReceiver gnss_receiver, const int sils_port_id, OnBoardComputer *obc, const unsigned char telemetry_channel)
    : GnssReceiver(gnss_receiver), UartCommunicationWithObc(sils_port_id, obc), telemetry_channel_(telemetry_channel) {}

Oem7600::Oem7600(GnssReceiver gnss_receiver, const int sils_port_id, OnBoardComputer *obc, const unsigned char telemetry_channel,
                 const unsigned int hils_port_id, const unsigned int baud_rate, HilsPortManager *hils_port_manager)
    : GnssReceiver(gnss_receiver),
      UartCommunicationWithObc(sils_port_id, obc, hils_port_id, baud_rate, hils_port_manager),
      telemetry_channel_(telemetry_channel) {}

void Oem7600::MainRoutine(const int time_count) {
  UNUSED(time_count);
  ReceiveCommand(0, MAX_CMD_LEN);

#ifdef TLM_FUNCTION_DEBUG_MODE
  // dummy commands for debugging tlm functions
  // std::string temp("log com1 bestxyza ontime 1");
  // std::string temp("log com1 timea ontime 1");
  // std::string temp("log com1 gpgga ontime 1");
  // std::string temp("log com1 bestxyzb ontime 1");
  std::string temp("log com1 hwmonitorb ontime 1");
  rx_buffer_.assign(std::begin(temp), end(temp));
  ParseCommand(0);
#endif

  // update positioning infromation(to be removed after core_oss modification)
  Update_local();

  // Send Telemetry just once
  if (!tlm_list_.empty()) {
    SendTelemetry(0);
  }

  return;
}

// update positioning infromation(to be removed after core_oss modification)
void Oem7600::Update_local() {
  // check visibility (transfered method from GNSSReciever Class)
  Vector<3> pos_true_eci_m = dynamics_->GetOrbit().GetPosition_i_m();
  libra::Quaternion q_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();

  CheckAntenna(pos_true_eci_m, q_i2b);

  if (is_gnss_visible_ == 1) {  // Antenna of GNSS-R can detect GNSS signal

    // should be modified to add noise in the future
    Vector<3> pos_true_ecef_m = dynamics_->GetOrbit().GetPosition_ecef_m();
    velocity_ecef_m_s_ = dynamics_->GetOrbit().GetVelocity_ecef_m_s();
    position_llh_ = dynamics_->GetOrbit().GetLatLonAlt();
    AddNoise(pos_true_eci_m, pos_true_ecef_m);

    last_position_fix_time_local_ = simulation_time_->GetElapsedTime_s();  // store position fixed time [sec]
    utc_ = simulation_time_->GetCurrentUtc();
    ConvertJulianDayToGPSTime(simulation_time_->GetCurrentTime_jd());
  } else {
    utc_ = simulation_time_->GetCurrentUtc();
    ConvertJulianDayToGPSTime(simulation_time_->GetCurrentTime_jd());
  }
}

int Oem7600::ParseCommand(const int command_size) {
  UNUSED(command_size);
  // decode recieved cmd
  OEM7600_CMD rcvd_cmd = DecodeCommand();

  // if the decoded cmd is obviously invalid
  if (rcvd_cmd.is_valid == false) {
    return -1;
  }
  // if the decoded cmd seems to be valid, details of the cmd is investigated by comparing its format to the expected one from its name
  else {
    if (rcvd_cmd.cmd_name == "log") {
      return Cmd_LOG(rcvd_cmd.cmd_args[0], rcvd_cmd.cmd_args[1], rcvd_cmd.cmd_args[2], rcvd_cmd.cmd_args[3]);
    }

    // command actions for the other types of command will be implimented in the future...
  }

  return 0;
}

int Oem7600::GenerateTelemetry() {
  // length of tlm cannnot be determined at this moment
  std::string tlm_all = "";

  // find all tlm must be padded from tlm list
  for (size_t i = 0; i < tlm_list_.size(); i++) {
    tlm_list_[i].count_up();  // tlm output timing is adjusted to previously commanded one

    if ((tlm_list_[i].count_to_out == 0) || ((tlm_list_[i].out_type == "once") || (tlm_list_[i].out_type == "onnext"))) {
      tlm_all += Gen_TLM_Packet(tlm_list_[i].tlm_name);

      // delete "once" or "onnext" type tlm from the list so as not to be padded
      // again without activation cmd
      if ((tlm_list_[i].out_type == "once") || (tlm_list_[i].out_type == "onnext")) {
        tlm_list_.erase(tlm_list_.begin() + i);
      }
    }
  }

  // To avoid unhandled error in the parent function, add dummy char if the result of the above process is empty
  if ((int)(tlm_all.length()) == 0) {
    tlm_all = " ";
  }

  // finally tlm data and its length is determined
  tx_buffer_.assign(std::begin(tlm_all), end(tlm_all));
  int TlmSize = (int)(tlm_all.length());

  return TlmSize;
}

// command decoder
OEM7600_CMD Oem7600::DecodeCommand() {
  OEM7600_CMD rcvd_cmd;
  unsigned char space_pos_count = 0;
  unsigned char space_pos[OEM7600_MAX_CMD_ARG];

  // examples of cmd format are...
  // CMD_NAME Arg1
  // CMD_NAME Arg1 Arg2 ...
  // type and length of Args vary according to commands and also according to the context of the each Args

  // find empty space character(separator)
  for (size_t i = 0; i < rx_buffer_.size(); i++) {
    if (rx_buffer_[i] == ' ') {
      space_pos[space_pos_count] = i;
      space_pos_count++;

      if (space_pos_count > OEM7600_MAX_CMD_ARG) {
        rcvd_cmd.is_valid = false;
        return rcvd_cmd;
      }
    }
  }

  // decode recieved command if suffient number of args are found
  if (space_pos_count > 0) {
    // store cmd name
    rcvd_cmd.cmd_name = std::string(rx_buffer_.begin(), rx_buffer_.begin() + space_pos[0]);

    // store cmd args
    if (space_pos_count > 1) {
      for (int i = 1; i < space_pos_count; i++) {
        rcvd_cmd.cmd_args[i - 1] = std::string(rx_buffer_.begin() + space_pos[i - 1] + 1, rx_buffer_.begin() + space_pos[i]);
      }
    }
    // last cmd arg begins from last space pos + 1 and lasts for the end of the cmd
    rcvd_cmd.cmd_args[space_pos_count - 1] = std::string(rx_buffer_.begin() + space_pos[space_pos_count - 1] + 1, rx_buffer_.end());

    rcvd_cmd.is_valid = true;
  }

  // in the case of insufficient number of args recieved
  else {
    rcvd_cmd.is_valid = false;
  }

  return rcvd_cmd;
}

// function correspond to "log" command action
int Oem7600::Cmd_LOG(const std::string comport_ID, const std::string tlm_name, const std::string type, const std::string freq) {
  // convert string "COMX" to int X
  int rcvd_comport = 0;
  char rcvd_comport_name = (char)(comport_ID[comport_ID.length() - 1]);
  if (isdigit(rcvd_comport_name)) {
    rcvd_comport = rcvd_comport_name - '0';
  } else {
    return -1;
  }

  // convert string "freq" to int freq
  int rcvd_out_freq = 0;
  if (type == "ontime") {
    char rcvd_freq = (char)(freq[freq.length() - 1]);
    if (isdigit(rcvd_freq)) {
      rcvd_out_freq = rcvd_freq - '0';
    } else {
      return -1;
    }
  }

  // if recieved comport id is correct
  if (telemetry_channel_ == rcvd_comport) {
    // check wether the designated tlm name is valid or not
    if (TLM_NameSearch(tlm_name)) {
      // update current tlm list
      OEM7600_TLM rcvd_tlm = {tlm_name, type, 0, (unsigned int)rcvd_out_freq};

      if (tlm_list_.empty()) {
        tlm_list_.push_back(rcvd_tlm);
        return 0;
      } else {
        // if the same tlm name already exists in the list
        for (size_t i = 0; i < tlm_list_.size(); i++) {
          if (tlm_list_[i].tlm_name == rcvd_tlm.tlm_name) {
            tlm_list_[i] = rcvd_tlm;
            return 0;
          }
        }
        // otherwise
        tlm_list_.push_back(rcvd_tlm);
        return 0;
      }
    }
  }

  // all of the other cases are irregular
  return -1;
}

// generate tlm body
std::string Oem7600::Gen_TLM_Packet(const std::string tlm_name) {
  std::string str_tmp = "";

  // In Case of NMEA tlm, header and footer are different from both of ASII,Binary formats
  if (tlm_name == "gpgga") {
    str_tmp += Gen_GPGGATlm();
    str_tmp += "\r\n";  // footer for ASCII format;
    return str_tmp;
  }

  // In Case of other ASCII format
  if (tlm_name[tlm_name.length() - 1] == 'a') {
    str_tmp += Gen_TLM_Header_Ascii(tlm_name);

    if (tlm_name == "bestxyza") {
      str_tmp += Gen_BestXYZTlm_Ascii();
    } else if (tlm_name == "timea") {
      str_tmp += Gen_TimeTlm_Ascii();
    } else if (tlm_name == "hwmonitora") {
      str_tmp += Gen_HWMonitorTlm_Ascii();
    } else {
      str_tmp = "";
      return str_tmp;  // returns empty string
    }

    // calculate CRC32 in ASCII format
    size_t tlm_length = str_tmp.length();
    if (tlm_length > 0) {
      const char *tlm_data_char = str_tmp.c_str();
      unsigned int crc32 = OEM7600_calculate_crc32(tlm_data_char, tlm_length);
      str_tmp += "*";                 // separator before CRC for ASCII format;
      str_tmp += WriteScalar(crc32);  // CRC in string format
      str_tmp.pop_back();             // erase unnecessary conma
      str_tmp += "\r\n";              // footer for ASCII format;
    }

    return str_tmp;
  }

  // In Case of Binary format
  if (tlm_name[tlm_name.length() - 1] == 'b') {
    str_tmp += Gen_TLM_Header_Binary(tlm_name);

    if (tlm_name == "bestxyzb") {
      str_tmp += Gen_BestXYZTlm_Binary();
    } else if (tlm_name == "hwmonitorb") {
      str_tmp += Gen_HWMonitorTlm_Binary();
    } else {
      str_tmp = "";
      return str_tmp;  // returns empty string
    }

    // calculate CRC32 in Binary format
    size_t tlm_length = str_tmp.length();
    if (tlm_length > 0) {
      const char *tlm_data_char = str_tmp.c_str();
      unsigned int crc32 = OEM7600_calculate_crc32(tlm_data_char, tlm_length);
      str_tmp.push_back((unsigned char)((crc32 & 0x000000ff)));
      str_tmp.push_back((unsigned char)((crc32 & 0x0000ff00) >> 8));
      str_tmp.push_back((unsigned char)((crc32 & 0x00ff0000) >> 16));
      str_tmp.push_back((unsigned char)((crc32 & 0xff000000) >> 24));
    }

    return str_tmp;
  }

  // In other cases, returns null string
  return str_tmp;
}

// generate tlm header
std::string Oem7600::Gen_TLM_Header_Ascii(const std::string tlm_name) {
  // footer of the header(reserved number and firmware revision)
  const std::string str_reserved = ",2ae1,15823;";

  // Sync + Message
  std::string str_tmp = "#" + tlm_name;
  std::transform(tlm_name.begin(), tlm_name.end(), (str_tmp.begin() + 1), ::toupper);

  std::string port = "COM" + WriteScalar((int)(telemetry_channel_), 1);  // Port
  str_tmp += "," + port;                                                 // after port, "," is already added (WriteScalar did it)
  str_tmp += "0";                                                        // Sequence number
  str_tmp += ",90.5";                                                    // CPU idle time (currently dummy data is padded)

  // Time status
  if (is_gnss_visible_) {
    str_tmp += ",FINESTEERING";
  } else {
    str_tmp += ",UNKNOWN";
  }

  str_tmp += WriteScalar(gps_time_week_);
  str_tmp += WriteScalar(gps_time_s_);
  str_tmp += "02000000";    // Receiver status summary (currently dummy data is padded)
  str_tmp += str_reserved;  // footer

  return str_tmp;
}

// generate tlm header
std::string Oem7600::Gen_TLM_Header_Binary(const std::string tlm_name) {
  const unsigned char kOEM7600RxHeader[] = {0xAA, 0x44, 0x12};  // Binary Tlm Header
  const unsigned char reserved_val_before_farmware_rev[] = {0xDB, 0x52};
  const unsigned char farmware_rev[] = {0xCF, 0x3D};

  std::string str_tmp;
  unsigned char tlm[OEM7600_COMMON_BINARY_HEADER_SIZE] = {};
  unsigned int tlm_parse_pointer = 0;

  // header for binary tlm
  for (int i = 0; i < 3; i++) {
    tlm[i] = kOEM7600RxHeader[i];
  }
  tlm_parse_pointer = 3;

  // header size
  tlm[tlm_parse_pointer++] = (unsigned char)(OEM7600_COMMON_BINARY_HEADER_SIZE);

  // tlm id
  unsigned short tlm_id = (unsigned short)(GetTlmIdOfBinaryTlm(tlm_name));
  if (tlm_id == OEM7600_TLM_ID_ERROR) return str_tmp;  // return null string
  tlm[tlm_parse_pointer++] = (unsigned char)((tlm_id & 0x00ff));
  tlm[tlm_parse_pointer++] = (unsigned char)((tlm_id & 0xff00) >> 8);

  // message type and port (always reserved to 0x00 and 0x20)
  tlm[tlm_parse_pointer++] = 0x00;
  tlm[tlm_parse_pointer++] = 0x20;

  // tlm length of variable part
  unsigned short tlm_length = GetTlmLengthOfBinaryTlm(tlm_name);
  if (tlm_length == 0) return str_tmp;  // return null string
  tlm[tlm_parse_pointer++] = (unsigned char)((tlm_length & 0x00ff));
  tlm[tlm_parse_pointer++] = (unsigned char)((tlm_length & 0xff00) >> 8);

  // message sequence (usually fixed to 0x0000)
  tlm[tlm_parse_pointer++] = 0x00;
  tlm[tlm_parse_pointer++] = 0x00;

  // dummy data of cpu idle time(0~200) and time status
  tlm[tlm_parse_pointer++] = 0xB2;

  if (is_gnss_visible_) {
    tlm[tlm_parse_pointer++] = 0xB4;  // Fine Steering
  } else {
    tlm[tlm_parse_pointer++] = 0x14;  // Unknown
  }

  // week(uint16),msec(uint32)
  tlm[tlm_parse_pointer++] = (unsigned char)(((unsigned short)(gps_time_week_) & 0x00ff));
  tlm[tlm_parse_pointer++] = (unsigned char)(((unsigned short)(gps_time_week_) & 0xff00) >> 8);

  unsigned int gps_time_msec = (unsigned int)(gps_time_s_ * 1000.0);
  tlm[tlm_parse_pointer++] = (unsigned char)((gps_time_msec & 0x000000ff));
  tlm[tlm_parse_pointer++] = (unsigned char)((gps_time_msec & 0x0000ff00) >> 8);
  tlm[tlm_parse_pointer++] = (unsigned char)((gps_time_msec & 0x00ff0000) >> 16);
  tlm[tlm_parse_pointer++] = (unsigned char)((gps_time_msec & 0xff000000) >> 24);

  // dummy data of receiver status(uint32)
  tlm[tlm_parse_pointer++] = 0x00;
  tlm[tlm_parse_pointer++] = 0x00;
  tlm[tlm_parse_pointer++] = 0x00;
  tlm[tlm_parse_pointer++] = 0x02;

  // reserved
  tlm[tlm_parse_pointer++] = reserved_val_before_farmware_rev[0];
  tlm[tlm_parse_pointer++] = reserved_val_before_farmware_rev[1];

  // farmware revision
  tlm[tlm_parse_pointer++] = farmware_rev[0];
  tlm[tlm_parse_pointer++] = farmware_rev[1];

  str_tmp.assign(std::begin(tlm), std::end(tlm));

  return str_tmp;
}

// function correspond to "bestxyza" tlm
std::string Oem7600::Gen_BestXYZTlm_Ascii(void) {
  std::string str_tmp = "";
  std::string sol_staus;

  // solution sutatus
  if (is_gnss_visible_) {
    sol_staus = "SOL_COMPUTED";
  } else {
    sol_staus = "INSUFFICIENT_OBS";
  }

  str_tmp += sol_staus + ",SINGLE,";              // position solution status
  str_tmp += WriteVector(position_ecef_m_, 11);   // postion computed
  str_tmp += "72.5816,61.3924,159.6638,";         // std 1-sigma of computed position(currently dummy data is padded)
  str_tmp += sol_staus + ",DOPPLER_VELOCITY,";    // velocity solution sutatus
  str_tmp += WriteVector(velocity_ecef_m_s_, 8);  // velocity computed
  str_tmp += "0.5747,0.4413,1.4830,";             // std 1-sigma of computed velocity(currently dummy data is padded)
  str_tmp += " ,";                                // base station ID of differenctial positioning (ignored in this model)
  str_tmp += "0.000,";                            // V-latency (velocity measurement time delay, currently dummy data is padded)
  str_tmp += "0.000,";                            // diff_age (differential age in seconds, basically the ouput value is fixed to zero)

  // sol_age (solution age in seconds)
  double sol_age = simulation_time_->GetElapsedTime_s() - last_position_fix_time_local_;
  str_tmp += WriteScalar(sol_age);

  str_tmp += WriteScalar(visible_satellite_number_);  // number of satellites tracked
  str_tmp += WriteScalar(visible_satellite_number_);  // number of satellites used in solution (currently dummy data is padded)
  str_tmp += WriteScalar(visible_satellite_number_);  // number of satellites with L1/E1/B1 signals used in solution (currently dummy data is padded)

  str_tmp += "0,";   // number of satellites with multi-frequency signals used in solution (currently dummy data is padded)
  str_tmp += "0,";   // reserved
  str_tmp += "02,";  // Extended solution status (currently fixed to default value, i.e, pseudorange lono correction is based on Klobuchar model)
  str_tmp += "00,";  // Galileo and BeiDou sig mask (currently fixed to default value, i.e, they are not used in solution)
  str_tmp += "01";   // GPS and GLONASS sig mask (currently fixed to default value, i.e, only GPS is used in solution)

  return str_tmp;
}

// function correspond to "bestxyzb" tlm
std::string Oem7600::Gen_BestXYZTlm_Binary(void) {
  std::string str_tmp;
  unsigned char tlm[OEM7600_BESTXYZ_BINARY_TLM_SIZE] = {};
  unsigned int tlm_parse_pointer = 0;
  const unsigned char kByteSizeLongDouble = 8;
  const unsigned char kByteSizeIntFloat = 4;
  std::vector<unsigned char> byte_buffer_double;
  std::vector<unsigned char> byte_buffer_float;

  // position, velocity fix status
  unsigned int position_solution_status;
  unsigned int position_solution_type;
  unsigned int valocity_solution_status;
  unsigned int velocity_solution_type;

  if (is_gnss_visible_) {
    position_solution_status = 0;  // Solution computed
    position_solution_type = 16;   // Single (Sololy based on GNSSR Signal)
    valocity_solution_status = 0;  // Solution computed
    velocity_solution_type = 8;    // Doppler Velocity
  } else {
    position_solution_status = 1;  // Insufficient observation
    position_solution_type = 0;    // No solution
    valocity_solution_status = 1;  // Insufficient observation
    velocity_solution_type = 0;    // No solution
  }

  // position_solution_status
  tlm[tlm_parse_pointer++] = (unsigned char)((position_solution_status & 0x000000ff));
  tlm[tlm_parse_pointer++] = (unsigned char)((position_solution_status & 0x0000ff00) >> 8);
  tlm[tlm_parse_pointer++] = (unsigned char)((position_solution_status & 0x00ff0000) >> 16);
  tlm[tlm_parse_pointer++] = (unsigned char)((position_solution_status & 0xff000000) >> 24);

  // position_solution_type
  tlm[tlm_parse_pointer++] = (unsigned char)((position_solution_type & 0x000000ff));
  tlm[tlm_parse_pointer++] = (unsigned char)((position_solution_type & 0x0000ff00) >> 8);
  tlm[tlm_parse_pointer++] = (unsigned char)((position_solution_type & 0x00ff0000) >> 16);
  tlm[tlm_parse_pointer++] = (unsigned char)((position_solution_type & 0xff000000) >> 24);

  // position in ECEF[m]
  for (int axis = 0; axis < 3; axis++) {
    byte_buffer_double = ConvertDoubleToByte(position_ecef_m_[axis]);
    for (int i = 0; i < kByteSizeLongDouble; i++) {
      tlm[tlm_parse_pointer++] = byte_buffer_double[i];
    }
  }

  // std 1 sigma of position estimation (currently dummy data is padded) [m]
  float position_error_sigma = 30.0f;
  for (int axis = 0; axis < 3; axis++) {
    byte_buffer_float = ConvertFloatToByte(position_error_sigma);
    for (int i = 0; i < kByteSizeIntFloat; i++) {
      tlm[tlm_parse_pointer++] = byte_buffer_float[i];
    }
  }

  // valocity_solution_status
  tlm[tlm_parse_pointer++] = (unsigned char)((valocity_solution_status & 0x000000ff));
  tlm[tlm_parse_pointer++] = (unsigned char)((valocity_solution_status & 0x0000ff00) >> 8);
  tlm[tlm_parse_pointer++] = (unsigned char)((valocity_solution_status & 0x00ff0000) >> 16);
  tlm[tlm_parse_pointer++] = (unsigned char)((valocity_solution_status & 0xff000000) >> 24);

  // velocity_solution_type
  tlm[tlm_parse_pointer++] = (unsigned char)((velocity_solution_type & 0x000000ff));
  tlm[tlm_parse_pointer++] = (unsigned char)((velocity_solution_type & 0x0000ff00) >> 8);
  tlm[tlm_parse_pointer++] = (unsigned char)((velocity_solution_type & 0x00ff0000) >> 16);
  tlm[tlm_parse_pointer++] = (unsigned char)((velocity_solution_type & 0xff000000) >> 24);

  // velocity in ECEF[m]
  for (int axis = 0; axis < 3; axis++) {
    byte_buffer_double = ConvertDoubleToByte(velocity_ecef_m_s_[axis]);
    for (int i = 0; i < kByteSizeLongDouble; i++) {
      tlm[tlm_parse_pointer++] = byte_buffer_double[i];
    }
  }

  // std 1 sigma of position estimation (currently dummy data is padded) [m]
  float velocity_error_sigma = 0.3f;
  for (int axis = 0; axis < 3; axis++) {
    byte_buffer_float = ConvertFloatToByte(velocity_error_sigma);
    for (int i = 0; i < kByteSizeIntFloat; i++) {
      tlm[tlm_parse_pointer++] = byte_buffer_float[i];
    }
  }

  // Base station ID (usually fixed to zero)
  unsigned int station_id_dummy = 0;
  tlm[tlm_parse_pointer++] = (unsigned char)((station_id_dummy & 0x000000ff));
  tlm[tlm_parse_pointer++] = (unsigned char)((station_id_dummy & 0x0000ff00) >> 8);
  tlm[tlm_parse_pointer++] = (unsigned char)((station_id_dummy & 0x00ff0000) >> 16);
  tlm[tlm_parse_pointer++] = (unsigned char)((station_id_dummy & 0xff000000) >> 24);

  // V-latency (velocity measurement time delay, currently dummy data is padded)
  float vlatency_dummy_sec = 0.0f;
  byte_buffer_float = ConvertFloatToByte(vlatency_dummy_sec);
  for (int i = 0; i < kByteSizeIntFloat; i++) {
    tlm[tlm_parse_pointer++] = byte_buffer_float[i];
  }

  // diff_age (differential age in seconds, basically the ouput value is fixed to zero)
  float diff_age_dummy_sec = 0.0f;
  byte_buffer_float = ConvertFloatToByte(diff_age_dummy_sec);
  for (int i = 0; i < kByteSizeIntFloat; i++) {
    tlm[tlm_parse_pointer++] = byte_buffer_float[i];
  }

  // solution age (elapsed time since last position fixed time [sec])
  float sol_age_sec = (float)(simulation_time_->GetElapsedTime_s() - last_position_fix_time_local_);
  byte_buffer_float = ConvertFloatToByte(sol_age_sec);
  for (int i = 0; i < kByteSizeIntFloat; i++) {
    tlm[tlm_parse_pointer++] = byte_buffer_float[i];
  }

  // number of satellites tracked, number of satellites used in solution, number
  // of satellites with L1/E1/B1 signals used, number of satellites with
  // multi-frequency signals
  tlm[tlm_parse_pointer++] = (unsigned char)visible_satellite_number_;
  tlm[tlm_parse_pointer++] = (unsigned char)visible_satellite_number_;
  tlm[tlm_parse_pointer++] = (unsigned char)visible_satellite_number_;
  tlm[tlm_parse_pointer++] = (unsigned char)visible_satellite_number_;

  // reserved and extended solution status (usually fixed to 0x00, 0x02)
  tlm[tlm_parse_pointer++] = 0x00;
  tlm[tlm_parse_pointer++] = 0x02;

  // GNSS System mask (currently fix to 0x00 and 0x01, i.e. only GPS is observed)
  tlm[tlm_parse_pointer++] = 0x00;
  tlm[tlm_parse_pointer++] = 0x01;

  str_tmp.assign(std::begin(tlm), std::end(tlm));

  return str_tmp;
}

// function correspond to "timea" tlm
std::string Oem7600::Gen_TimeTlm_Ascii(void) {
  std::string str_tmp = "";
  std::string clock_status;

  // clock sutatus
  if (is_gnss_visible_) {
    clock_status += "VALID";
  } else {
    clock_status += "INVALID";
  }

  str_tmp += clock_status + ",";   // clock sutatus
  str_tmp += "-3.004848645e-08,";  // receiver clock offset in seconds from GPST (clock = GPST + offset), currently dummya data is padded
  str_tmp += "2.736064744e-09,";   // std 1-sigma of clock offset (currently dummya data is padded)
  str_tmp += "-17.99999999852,";   // GPST offset in seconds from UTC (UTC = GPST + offset), currently fixed number at A.C.2020 (=-18sec)is padded

  // UTC
  str_tmp += WriteScalar(utc_.year);
  str_tmp += WriteScalar(utc_.month);
  str_tmp += WriteScalar(utc_.day);
  str_tmp += WriteScalar(utc_.hour);
  str_tmp += WriteScalar(utc_.minute);
  str_tmp += WriteScalar((unsigned int)(utc_.second * 1000));  // sec in [msec] format
  str_tmp += clock_status;  // UTC status (in real HW, UTC status is not always equal to clock status, but is deal to be equal in this model )

  return str_tmp;
}

// function correspond to "hwmonitora" tlm
std::string Oem7600::Gen_HWMonitorTlm_Ascii(void) {
  // currently dummy data is padded(voltage, temperature etc.)
  std::string str_tmp =
      "6,22.283273697,100,3.095238209,601,3.110134363,701,1.194749713,800,3."
      "095238209,f01,1.821123362,1100*d59cf319";
  return str_tmp;
}

// function correspond to "hwmonitorb" tlm (currently dummy data is padded = constant voltage, temperature, etc.)
std::string Oem7600::Gen_HWMonitorTlm_Binary(void) {
  std::string str_tmp;
  unsigned char tlm[OEM7600_HWMONITOR_BINARY_TLM_SIZE] = {};
  unsigned int tlm_parse_pointer = 0;
  const unsigned char kByteSizeIntFloat = 4;
  std::vector<unsigned char> byte_buffer_float;

  // number of output
  unsigned int num_of_output_contenst = 6;
  tlm[tlm_parse_pointer++] = (unsigned char)((num_of_output_contenst & 0x000000ff));
  tlm[tlm_parse_pointer++] = (unsigned char)((num_of_output_contenst & 0x0000ff00) >> 8);
  tlm[tlm_parse_pointer++] = (unsigned char)((num_of_output_contenst & 0x00ff0000) >> 16);
  tlm[tlm_parse_pointer++] = (unsigned char)((num_of_output_contenst & 0xff000000) >> 24);

  // temperature [degC] (currently dummy data is padded)
  float temperature_degC = 27.899879f;
  byte_buffer_float = ConvertFloatToByte(temperature_degC);
  for (int i = 0; i < kByteSizeIntFloat; i++) {
    tlm[tlm_parse_pointer++] = byte_buffer_float[i];
  }

  // temperature staus (currently dummy data is padded)
  unsigned int temperature_status = 0x00000100;
  tlm[tlm_parse_pointer++] = (unsigned char)((temperature_status & 0x000000ff));
  tlm[tlm_parse_pointer++] = (unsigned char)((temperature_status & 0x0000ff00) >> 8);
  tlm[tlm_parse_pointer++] = (unsigned char)((temperature_status & 0x00ff0000) >> 16);
  tlm[tlm_parse_pointer++] = (unsigned char)((temperature_status & 0xff000000) >> 24);

  // 3V3 Voltage [V] (currently dummy data is padded)
  float internally_regulated_3V3_voltage_V = 3.101343f;
  byte_buffer_float = ConvertFloatToByte(internally_regulated_3V3_voltage_V);
  for (int i = 0; i < kByteSizeIntFloat; i++) {
    tlm[tlm_parse_pointer++] = byte_buffer_float[i];
  }

  // 3V3 Voltage staus (currently dummy data is padded)
  unsigned int internally_regulated_3V3_voltage_status = 0x00000601;
  tlm[tlm_parse_pointer++] = (unsigned char)((internally_regulated_3V3_voltage_status & 0x000000ff));
  tlm[tlm_parse_pointer++] = (unsigned char)((internally_regulated_3V3_voltage_status & 0x0000ff00) >> 8);
  tlm[tlm_parse_pointer++] = (unsigned char)((internally_regulated_3V3_voltage_status & 0x00ff0000) >> 16);
  tlm[tlm_parse_pointer++] = (unsigned char)((internally_regulated_3V3_voltage_status & 0xff000000) >> 24);

  // Antenna Voltage [V] (currently dummy data is padded)
  float antenna_voltage_V = 3.110134f;
  byte_buffer_float = ConvertFloatToByte(antenna_voltage_V);
  for (int i = 0; i < kByteSizeIntFloat; i++) {
    tlm[tlm_parse_pointer++] = byte_buffer_float[i];
  }

  // Antenna Voltage staus  (currently dummy data is padded)
  unsigned int antenna_voltage_status = 0x00000701;
  tlm[tlm_parse_pointer++] = (unsigned char)((antenna_voltage_status & 0x000000ff));
  tlm[tlm_parse_pointer++] = (unsigned char)((antenna_voltage_status & 0x0000ff00) >> 8);
  tlm[tlm_parse_pointer++] = (unsigned char)((antenna_voltage_status & 0x00ff0000) >> 16);
  tlm[tlm_parse_pointer++] = (unsigned char)((antenna_voltage_status & 0xff000000) >> 24);

  // 1V2 Voltage [V] (currently dummy data is padded)
  float internally_regulated_1V2_voltage_V = 1.190476f;
  byte_buffer_float = ConvertFloatToByte(internally_regulated_1V2_voltage_V);
  for (int i = 0; i < kByteSizeIntFloat; i++) {
    tlm[tlm_parse_pointer++] = byte_buffer_float[i];
  }

  // 1V2 Voltage staus (currently dummy data is padded)
  unsigned int internally_regulated_1V2_voltage_status = 0x00000800;
  tlm[tlm_parse_pointer++] = (unsigned char)((internally_regulated_1V2_voltage_status & 0x000000ff));
  tlm[tlm_parse_pointer++] = (unsigned char)((internally_regulated_1V2_voltage_status & 0x0000ff00) >> 8);
  tlm[tlm_parse_pointer++] = (unsigned char)((internally_regulated_1V2_voltage_status & 0x00ff0000) >> 16);
  tlm[tlm_parse_pointer++] = (unsigned char)((internally_regulated_1V2_voltage_status & 0xff000000) >> 24);

  // supply Voltage [V] (currently dummy data is padded)
  float supply_voltage_V = 3.091575f;
  byte_buffer_float = ConvertFloatToByte(supply_voltage_V);
  for (int i = 0; i < kByteSizeIntFloat; i++) {
    tlm[tlm_parse_pointer++] = byte_buffer_float[i];
  }

  // supply Voltage staus (currently dummy data is padded)
  unsigned int supply_voltage_status = 0x00000f01;
  tlm[tlm_parse_pointer++] = (unsigned char)((supply_voltage_status & 0x000000ff));
  tlm[tlm_parse_pointer++] = (unsigned char)((supply_voltage_status & 0x0000ff00) >> 8);
  tlm[tlm_parse_pointer++] = (unsigned char)((supply_voltage_status & 0x00ff0000) >> 16);
  tlm[tlm_parse_pointer++] = (unsigned char)((supply_voltage_status & 0xff000000) >> 24);

  // 1V8 Voltage [V] (currently dummy data is padded)
  float internally_regulated_1V8_voltage_V = 1.821734f;
  byte_buffer_float = ConvertFloatToByte(internally_regulated_1V8_voltage_V);
  for (int i = 0; i < kByteSizeIntFloat; i++) {
    tlm[tlm_parse_pointer++] = byte_buffer_float[i];
  }

  // 1V8 Voltage staus (currently dummy data is padded)
  unsigned int internally_regulated_1V8_voltage_status = 0x00001100;
  tlm[tlm_parse_pointer++] = (unsigned char)((internally_regulated_1V8_voltage_status & 0x000000ff));
  tlm[tlm_parse_pointer++] = (unsigned char)((internally_regulated_1V8_voltage_status & 0x0000ff00) >> 8);
  tlm[tlm_parse_pointer++] = (unsigned char)((internally_regulated_1V8_voltage_status & 0x00ff0000) >> 16);
  tlm[tlm_parse_pointer++] = (unsigned char)((internally_regulated_1V8_voltage_status & 0xff000000) >> 24);

  str_tmp.assign(std::begin(tlm), std::end(tlm));

  return str_tmp;
}

// function correspond to "gpgga" tlm
std::string Oem7600::Gen_GPGGATlm(void) {
  std::string str_tmp = "$GPGGA,";

  // if 100 sec passed from last position fix timing, all the succeeding outputs reset to blank
  double elapsed_sec = simulation_time_->GetElapsedTime_s() - last_position_fix_time_local_;
  if (elapsed_sec > 100.0) {
    str_tmp += ",,,,,0,,,,,,,,*66";
  }
  // in case of position fixed
  else {
    // UTC
    std::string str_utc_tmp = WriteScalar(utc_.hour, 2);
    str_utc_tmp.erase(str_utc_tmp.end() - 1);  // since "," added in "WriteScalar" is unnecessary in this case, erase it here
    str_tmp += string_zeropad_local(str_utc_tmp, 2);

    str_utc_tmp = WriteScalar(utc_.minute, 2);
    str_utc_tmp.erase(str_utc_tmp.end() - 1);  // since "," added in "WriteScalar" is unnecessary in this case, erase it here
    str_tmp += string_zeropad_local(str_utc_tmp, 2);

    str_utc_tmp = WriteScalar(utc_.second, 4);
    str_tmp += string_zeropad_local(str_utc_tmp, 6);

    // lat in [ddmm.mm] + indicator "N" or "S"
    str_tmp += convLatLontoNMEA(position_llh_[0], "lat");

    // lon in [dddmm.mm] + indicator "E" or "W"
    str_tmp += convLatLontoNMEA(position_llh_[1], "lon");

    // quality
    if (is_gnss_visible_) {
      str_tmp += "1,";
    } else {
      str_tmp += "0,";
    }

    // number of vis sat
    str_tmp += WriteScalar(visible_satellite_number_);

    // hdop (currently dummy data is padded)
    str_tmp += "1.0,";

    // alt in [m]
    str_tmp += WriteScalar(position_llh_[2]);

    // units of alt(fix to M in default)
    str_tmp += "M,";

    // offset b/w geoidal height and WGS84 (currently dummy data is padded)
    str_tmp += "-16.27,";

    // units of offset(fix to M in default)
    str_tmp += "M,";

    // age of correction data in [sec]
    str_tmp += WriteScalar((unsigned int)(elapsed_sec), 2);
    str_tmp.erase(str_tmp.end() - 1);  // since "," added in "WriteScalar" is unnecessary in this case, erase it here

    // check sum (currently dummy data is padded)
    str_tmp += "*60";
  }

  return str_tmp;
}

// check wether the designated tlm name in "log" cmd is valid or not
bool Oem7600::TLM_NameSearch(const std::string tlm_name) {
  for (int i = 0; i < OEM7600_MAX_TLM_LIST; i++) {
    if (tlm_name_dictionary_[i] == tlm_name) {
      return true;
    }
  }

  return false;
}

// get tlm id for binary format tlm packet
OEM7600_BINARY_TLM_ID Oem7600::GetTlmIdOfBinaryTlm(const std::string tlm_name) {
  if (tlm_name == "bestxyzb") {
    return OEM7600_TLM_ID_BEST_XYZ;
  } else if (tlm_name == "hwmonitorb") {
    return OEM7600_TLM_ID_HARDWARE_MONITOR;
  } else {
    return OEM7600_TLM_ID_ERROR;
  }
}

// get tlm length of variable part for binary format tlm
unsigned short Oem7600::GetTlmLengthOfBinaryTlm(const std::string tlm_name) {
  if (tlm_name == "bestxyzb") {
    return OEM7600_BESTXYZ_BINARY_TLM_SIZE;
  } else if (tlm_name == "hwmonitorb") {
    return OEM7600_HWMONITOR_BINARY_TLM_SIZE;
  } else {
    return 0;
  }
}

// convert lat/lon in [rad] to NMEA format
std::string Oem7600::convLatLontoNMEA(const double rad, const std::string type) {
  const double rad2deg = 180.0 / libra::pi;
  const double deg2arcmin = 60.0;

  std::string str_tmp = "";

  double angle_deg = fabs(rad) * rad2deg;
  unsigned int angle_dd = (unsigned int)(angle_deg);
  double angle_rem_amin = (angle_deg - (double)(angle_dd)) * deg2arcmin;

  std::string str_deg = WriteScalar(angle_dd);
  str_deg.erase(str_deg.end() - 1);  // since "," added in "WriteScalar" is unnecessary in this case, erase it here
  if (type == "lat") {
    str_tmp += string_zeropad_local(str_deg, 2);
  } else {
    str_tmp += string_zeropad_local(str_deg, 3);
  }

  str_tmp += WriteScalar(angle_rem_amin, 4);

  if (type == "lat") {
    if (rad < 0) {
      str_tmp += "S,";
    } else {
      str_tmp += "N,";
    }
  } else {
    if (rad < 0) {
      str_tmp += "E,";
    } else {
      str_tmp += "W,";
    }
  }

  return str_tmp;
}

// zero padding(to be removed and improved after core_oss modification)
std::string Oem7600::string_zeropad_local(const std::string str_org, const unsigned int len) {
  std::string str_tmp = str_org;
  if (str_org.length() < len) {
    int pad_n = len - str_org.length();

    for (int i = 0; i < pad_n; i++) {
      str_tmp = "0" + str_tmp;
    }
  }

  // otherwise nothing to do

  return str_tmp;
}

// convert position,velocity data in double format into byte format
std::vector<unsigned char> Oem7600::ConvertDoubleToByte(const double double_data) {
  union {
    double buffer_double_format;
    uint64_t buffer_byte_format;
  } convertbuffer_double_byte;

  const unsigned char kByteSizeDouble = 8;
  std::vector<unsigned char> byte_vector;

  convertbuffer_double_byte.buffer_double_format = double_data;

  uint64_t byte_extruct_mask = 0x00000000000000ff;
  unsigned char bit_shifter_for_byte_extruct = 0;
  for (int i = 0; i < kByteSizeDouble; i++) {
    unsigned char byte_buffer = (unsigned char)((convertbuffer_double_byte.buffer_byte_format & byte_extruct_mask) >> bit_shifter_for_byte_extruct);
    byte_vector.push_back(byte_buffer);
    byte_extruct_mask = byte_extruct_mask << 8;
    bit_shifter_for_byte_extruct += 8;
  }

  return byte_vector;
}

// convert position,velocity data in float format into byte format
std::vector<unsigned char> Oem7600::ConvertFloatToByte(const float float_data) {
  union {
    float buffer_float_format;
    unsigned int buffer_byte_format;
  } convertbuffer_float_byte;

  const unsigned char kByteSizeFloat = 4;
  std::vector<unsigned char> byte_vector;

  convertbuffer_float_byte.buffer_float_format = float_data;

  unsigned int byte_extruct_mask = 0x000000ff;
  unsigned char bit_shifter_for_byte_extruct = 0;
  for (int i = 0; i < kByteSizeFloat; i++) {
    unsigned char byte_buffer = (unsigned char)((convertbuffer_float_byte.buffer_byte_format & byte_extruct_mask) >> bit_shifter_for_byte_extruct);
    byte_vector.push_back(byte_buffer);
    byte_extruct_mask = byte_extruct_mask << 8;
    bit_shifter_for_byte_extruct += 8;
  }

  return byte_vector;
}

// 32bit CRC Calculation
unsigned int Oem7600::OEM7600_calculate_crc32(const char *tlm_data_for_crc, const size_t data_length) {
  unsigned int temporary_value1_for_crc_calcx;
  unsigned int temporary_value2_for_crc_calcx;
  unsigned int calculated_crc32x = 0;
  unsigned int data_length_for_crc_calc = data_length;

  while (data_length_for_crc_calc-- != 0) {
    temporary_value1_for_crc_calcx = (calculated_crc32x >> 8) & 0x00FFFFFF;
    temporary_value2_for_crc_calcx = OEM7600_calculate_crc32_subroutine(((unsigned int)calculated_crc32x ^ *tlm_data_for_crc++) & 0xFF);
    calculated_crc32x = temporary_value1_for_crc_calcx ^ temporary_value2_for_crc_calcx;
  }

  return calculated_crc32x;
}

// 32bit CRC Calculation subroutine
unsigned int Oem7600::OEM7600_calculate_crc32_subroutine(const unsigned int initial_value) {
  const unsigned int kCrc32Polynominal = 0xEDB88320;

  unsigned int uint32_crcx = initial_value;
  for (int j = 8; j > 0; j--) {
    if (uint32_crcx & 1) {
      uint32_crcx = (uint32_crcx >> 1) ^ kCrc32Polynominal;
    } else {
      uint32_crcx >>= 1;
    }
  }

  return uint32_crcx;
}

std::string Oem7600::GetLogHeader() const {
  std::string str_tmp = "";
  std::string MSSection = "OEM7600";
  str_tmp += WriteScalar("OEM7600_year");
  str_tmp += WriteScalar("OEM7600_month");
  str_tmp += WriteScalar("OEM7600_day");
  str_tmp += WriteScalar("OEM7600_hour");
  str_tmp += WriteScalar("OEM7600_min");
  str_tmp += WriteScalar("OEM7600_sec");
  str_tmp += WriteVector(MSSection, "posXYZ_ECEF", "m", 3);
  str_tmp += WriteVector(MSSection, "velXYZ_ECEF", "m/s", 3);
  str_tmp += WriteScalar("OEM7600_lat", "rad");
  str_tmp += WriteScalar("OEM7600_lon", "rad");
  str_tmp += WriteScalar("OEM7600_alt", "m");
  str_tmp += WriteScalar("OEM7600vis_flag");
  str_tmp += WriteScalar("OEM7600vis_num");

  return str_tmp;
}
