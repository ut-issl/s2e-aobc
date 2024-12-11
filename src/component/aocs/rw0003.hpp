/**
 * @file rw0003.hpp
 * @brief Class to emulate RW0.003 reaction wheel
 * @note Manual: NA
 *       Functions not used in the project are not implemented
 */

#ifndef S2E_AOBC_COMPONENT_AOCS_RW0003_HPP_
#define S2E_AOBC_COMPONENT_AOCS_RW0003_HPP_

#include <components/base/i2c_target_communication_with_obc.hpp>
#include <components/real/aocs/reaction_wheel.hpp>

#include "../../library/crc.hpp"

/**
 * @class Rw0003
 * @brief Class to emulate RW0.003 reaction wheel
 */
class Rw0003 : public s2e::components::ReactionWheel, public s2e::components::I2cTargetCommunicationWithObc {
 public:
  /**
   * @fn Rw0003
   * @brief Constructor
   * @param [in] rw: Reaction wheel settings
   * @param [in] sils_port_id: Port ID for SILS
   * @param [in] hils_port_id: Port ID for HILS
   * @param [in] i2c_address: I2C address
   * @param [in] obc: Connected OBC
   * @param [in] hils_port_manager: HILS port manager
   */
  Rw0003(s2e::components::ReactionWheel rw, const int sils_port_id, const unsigned int hils_port_id, const unsigned char i2c_address, s2e::components::OnBoardComputer *obc,
         s2e::simulation::HilsPortManager *hils_port_manager);

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
  bool is_rw_initialized_ = false;  //!< Flag to detect initializing operation
  double temperature_degC_ = 30.0;  //!< RW measured temperature [degC] (dummy data)
  float fault_state_ = 0.0;         //!< 1.0: wheel is in an fault, 0.0: otherwise

  // Communication
  uint16_t crc_;                             //!< Calculated CRC value
  const uint8_t kSourceAddress_ = 0x11;      //!< Source address
  static const uint8_t kHeaderSize_ = 2;     //!< Header size
  static const uint8_t kFooterSize_ = 1;     //!< Footer size
  static const uint8_t kCrcSize_ = 2;        //!< CRC size
  static const uint8_t kMaxCmdLength_ = 15;  //!< Command max length
  static const uint8_t kMaxTlmLength_ = 15;  //!< Telemetry max length

  // Command
  static const uint8_t kCmdIdInit_ = 0x01;       //!< Command ID for initialize
  static const uint8_t kCmdIdReadFile_ = 0x07;   //!< Command ID for read file
  static const uint8_t kCmdIdWriteFile_ = 0x08;  //!< Command ID for write file

  // Write Command
  static const uint8_t kWriteCmdIdle_ = 0x00;    //!< Idle command ID
  static const uint8_t kWriteCmdSpeed_ = 0x03;   //!< Rotation speed setting command ID
  static const uint8_t kWriteCmdTorque_ = 0x12;  //!< Torque setting command ID

  // Register address
  static const uint8_t kReadAddressTemperature_ = 0x03;  //!< Register address of temperature measurement
  static const uint8_t kReadAddressSpeed_ = 0x15;        //!< Register address of rotation speed measurement
  static const uint8_t kReadAddressFaultState_ = 0x19;   //!< Register address of fault state
  static const uint8_t kReadAddressLimitSpeed1_ = 0x33;  //!< Register address of limit speed 1
  static const uint8_t kReadAddressLimitSpeed2_ = 0x34;  //!< Register address of limit speed 2

  static const uint8_t kMcfReadEdac_ = 0xa7;    //!< Read EDAC memory MCF value
  static const uint16_t kCrcInitial_ = 0xffff;  //!< CRC initial value
  static const bool kCrcReverseFlag_ = false;   //!< CRC reverse flag

  // HILS
  bool is_command_written_ = false;         //!< Command written flag
  const unsigned int kStoredFrameSize = 3;  //!< Stored frame size for HILS
  /**
   * @fn Initialize
   * @brief Store data before execution for HILS
   */
  void Initialize();

  // Command
  /**
   * @fn ReadCmd
   * @brief Read and execute command
   */
  void ReadCmd();
  /**
   * @fn ReadCmdInit
   * @brief Read and execute initialize command
   * @param [in] payload: Argument of the command
   */
  void ReadCmdInit(std::vector<uint8_t> payload);
  /**
   * @fn ReadCmdWriteFile
   * @brief Read and execute write file command
   * @param [in] payload: Argument of the command
   */
  void ReadCmdWriteFile(const std::vector<uint8_t> payload);
  /**
   * @fn ReadCmdReadFile
   * @brief Read and execute read file command
   * @param [in] payload: Argument of the command
   */
  void ReadCmdReadFile(const std::vector<uint8_t> payload);
  /**
   * @fn decode_mcf
   * @brief Decode MCF to detect reply requirement
   * @param [in] command_id: Command ID
   * @param [in] mcf: MCF data
   * @return 1: with reply, 0: without reply
   */
  uint8_t decode_mcf(uint8_t *command_id, const uint8_t mcf);

  // Telemetry
  /**
   * @fn WriteFloatTlm
   * @brief Write float data as telemetry format
   * @param [in] address: Telemetry address
   * @param [in] value: Input float value
   */
  void WriteFloatTlm(uint8_t address, float value);
};

#endif  // S2E_AOBC_COMPONENT_AOCS_RW0003_HPP_
