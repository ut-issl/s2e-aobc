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
 * @class MtqSeiren
 * @brief Class to emulate MTQ developed by Seiren
 */
class MtqSeiren : public s2e::components::Magnetorquer, public s2e::components::GpioConnectionWithObc {
 public:
  /**
   * @fn MtqSeiren
   * @brief Constructor
   * @param [in] magnetorquer: Magnetorquer setting
   * @param [in] gpio_port: Port ID for GPIO
   * @param [in] obc: Connected OBC
   */
  MtqSeiren(s2e::components::Magnetorquer magnetorquer, std::vector<int> gpio_ports, s2e::components::OnBoardComputer *obc);
  /**
   * @fn ~MtqSeiren
   * @brief Destructor
   */
  ~MtqSeiren();

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

  s2e::math::Vector<s2e::components::kMtqDimension> polarity_;  //!< Polarity information of MTQ

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
