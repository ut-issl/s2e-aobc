#ifndef S2E_AOBC_COMPONENT_AOCS_AOBC_HPP_
#define S2E_AOBC_COMPONENT_AOCS_AOBC_HPP_

#include <components/base/component.hpp>

class AocsModuleComponents;

class AOBC : public Component {
 public:
  AOBC(ClockGenerator *clock_gen, AocsModuleComponents *components);
  ~AOBC();
  void Initialize();

 protected:
  AocsModuleComponents *components_;
  void MainRoutine(int count);
};

#endif  // S2E_AOBC_COMPONENT_AOCS_AOBC_HPP_
