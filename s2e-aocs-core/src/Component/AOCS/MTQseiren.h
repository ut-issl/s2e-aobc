#pragma once
#include "MagTorquer.h"
#include "ObcGpioBase.h"

/* References
Manual: NA
HowToUse: NA
*/

class MTQseiren: public MagTorquer, public ObcGpioBase
{
public:
  MTQseiren(
    MagTorquer mag_torquer,
    vector<int> gpio_ports,
    OBC* obc
  );
  ~MTQseiren();

  // Override: MagTorquer functions
  void MainRoutine(int count) override;
  std::string GetLogHeader() const override;

private:
  typedef enum
  {
    GPIO_X_POSITIVE = 0,
    GPIO_X_NEGATIVE,
    GPIO_Y_POSITIVE,
    GPIO_Y_NEGATIVE,
    GPIO_Z_POSITIVE,
    GPIO_Z_NEGATIVE,
  } MtqGpioIdx;

  libra::Vector<kMtqDim> polarity_;

  // Read GPIO state and determine MTQ polarity
  // return:
  // +1: positive
  // -1: negative
  //  0: zero (no output)
  int ConvertGPIOState2Polarity(int positive_gpio_pin_idx, int negative_gpio_pin_idx);

  void ConvertPolarity2OutputMag();
};
