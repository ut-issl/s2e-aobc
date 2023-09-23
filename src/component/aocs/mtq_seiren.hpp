/**
 * @file mtq_seiren.hpp
 * @brief Class to emulate MTQ developed by Seiren
 * @note Manual: NA
 */

#ifndef S2E_AOBC_COMPONENT_AOCS_MTQ_SEIREN_HPP_
#define S2E_AOBC_COMPONENT_AOCS_MTQ_SEIREN_HPP_

#include <components/base/gpio_connection_with_obc.hpp>
#include <components/real/aocs/magnetorquer.hpp>

/**
 * @class MTQseiren
 * @brief Class to emulate MTQ developed by Seiren
 */
class MTQseiren : public Magnetorquer, public GpioConnectionWithObc {
 public:
  /**
   * @fn MTQseiren
   * @brief Constructor
   * @param [in] mag_torquer: Magnetorquer setting
   * @param [in] gpio_port: Port ID for GPIO
   * @param [in] obc: Connected OBC
   */
  MTQseiren(Magnetorquer mag_torquer, std::vector<int> gpio_ports, OnBoardComputer *obc);
  /**
   * @fn ~MTQseiren
   * @brief Destructor
   */
  ~MTQseiren();

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
  /**
   * @enum MtqGpioIdx
   * @brief Index of MTQ GPIO
   */
  typedef enum {
    GPIO_X_POSITIVE = 0,
    GPIO_X_NEGATIVE,
    GPIO_Y_POSITIVE,
    GPIO_Y_NEGATIVE,
    GPIO_Z_POSITIVE,
    GPIO_Z_NEGATIVE,
  } MtqGpioIdx;

  libra::Vector<kMtqDimension> polarity_;  //!< Polarity information of MTQ

  /**
   * @fn ConvertGPIOState2Polarity
   * @brief Read GPIO state and determine MTQ polarity
   * @param [in] positive_gpio_pin_idx: Positive GPIO port index
   * @param [in] negative_gpio_pin_idx: Positive GPIO port index
   * @return +1: positive, -1: negative, 0: zero (no output)
   */
  int ConvertGPIOState2Polarity(int positive_gpio_pin_idx, int negative_gpio_pin_idx);

  /**
   * @fn ConvertPolarity2OutputMag
   * @brief Convert polarity to MTQ output
   */
  void ConvertPolarity2OutputMag();
};

#endif  // S2E_AOBC_COMPONENT_AOCS_MTQ_SEIREN_HPP_
