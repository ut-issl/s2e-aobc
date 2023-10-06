/**
 * @file aobc.hpp
 * @brief Class to emulate AOBC
 */

#ifndef S2E_AOBC_COMPONENT_AOCS_AOBC_HPP_
#define S2E_AOBC_COMPONENT_AOCS_AOBC_HPP_

#include <components/base/component.hpp>

class AocsModuleComponents;

/**
 * @class AOBC
 * @brief Class to emulate AOBC
 */
class AOBC : public Component {
 public:
  /**
   * @fn AOBC
   * @brief Constructor
   * @param [in] clock_generator: Clock generator
   * @param [in] components: Components connected to the AOBC
   */
  AOBC(ClockGenerator *clock_generator, AocsModuleComponents *components);
  /**
   * @fn ~AOBC
   * @brief Destructor
   */
  ~AOBC();

  /**
   * @fn Initialize
   * @brief Initialize functions
   */
  void Initialize();

 protected:
  AocsModuleComponents *components_;  //!< Components connected to the AOBC

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine for sensor observation
   */
  void MainRoutine(const int time_count);
};

#endif  // S2E_AOBC_COMPONENT_AOCS_AOBC_HPP_
