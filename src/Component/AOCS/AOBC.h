#pragma once
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
