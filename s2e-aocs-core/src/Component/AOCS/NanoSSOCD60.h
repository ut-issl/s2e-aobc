#pragma once
#include "SunSensor.h"
#include "ObcI2cTargetCommunicationBase.h"

/* References
Manual: NA
HowToUse: NA
*/

class NanoSSOCD60: public SunSensor, public ObcI2cTargetCommunicationBase
{
public:
  NanoSSOCD60(
    SunSensor sun_sensor,
    const int sils_port_id,
    const unsigned int hils_port_id,
    const unsigned char i2c_addr,
    OBC* obc,
    HilsPortManager* hils_port_manager
  );
  ~NanoSSOCD60();

  // Override: SunSensor functions
  void MainRoutine(int count) override;
  std::string GetLogHeader() const override;

private:
  const unsigned char i2c_addr_;
  const unsigned int kStoredFrameSize = 3;
  
  double sun_intensity_percent_ = 0.0;

  //General Functions
  int32_t ConvertFloat2FloatingPoint(float data);

  //TLM
  int GenerateTelemetry();
  int32_t ConvertAngle2Tlm(double angle);
  unsigned char GenerateErrorCode();
};
