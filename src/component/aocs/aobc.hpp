/**
 * @file aobc.hpp
 * @brief Class to emulate AOBC
 */

#ifndef S2E_AOBC_COMPONENT_AOCS_AOBC_HPP_
#define S2E_AOBC_COMPONENT_AOCS_AOBC_HPP_

#include <components/base/component.hpp>

class AocsModuleComponents;

/**
 * @class Aobc
 * @brief Class to emulate AOBC
 */
class Aobc : public Component {
 public:
  /**
   * @fn Aobc
   * @brief Constructor
   * @param [in] clock_generator: Clock generator
   * @param [in] components: Components connected to the AOBC
   */
  Aobc(ClockGenerator *clock_generator, AocsModuleComponents *components);
  /**
   * @fn ~Aobc
   * @brief Destructor
   */
  ~Aobc();

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
