#pragma once
#include "ComponentBase.h"
#include "Vector.hpp"

class ISSL6UComponents;

class AOBC: public ComponentBase
{
public:
  AOBC(ClockGenerator* clock_gen, ISSL6UComponents* components);
  ~AOBC();
  void Initialize();
protected:
  ISSL6UComponents* components_;
  void MainRoutine(int count);
};