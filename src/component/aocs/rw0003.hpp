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
class Rw0003 : public ReactionWheel, public I2cTargetCommunicationWithObc {
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
  Rw0003(ReactionWheel rw, const int sils_port_id, const unsigned int hils_port_id, const unsigned char i2c_address, OnBoardComputer *obc,
         HilsPortManager *hils_port_manager);

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

  // Picompo
  uint32_t picompo_handle_ = 0;

  /**
   * @fn Initialize
   * @brief Store data before execution for HILS
   */
  void Initialize();
};

extern "C" {
  uint32_t PICOMPO_open(const char *name);
  int32_t PICOMPO_send(uint32_t handle, uint8_t *data, uint32_t data_len);
  int32_t PICOMPO_recv(uint32_t handle, uint8_t *buf, uint32_t buf_len);
}

typedef struct {
  uint8_t is_initialized;
  uint8_t mode;
  uint8_t _padding[2];
  float torque;
} InputReport;

typedef struct {
  float temperature;
  float speed;
} OutputReport;

#endif  // S2E_AOBC_COMPONENT_AOCS_RW0003_HPP_
