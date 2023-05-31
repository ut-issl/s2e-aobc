#pragma once
#include <components/base/component.hpp>

class ISSL6UComponents;

class AOBC : public Component {
public:
  AOBC(ClockGenerator *clock_gen, ISSL6UComponents *components);
  ~AOBC();
  void Initialize();

protected:
  ISSL6UComponents *components_;
  void MainRoutine(int count);
};
