#include "MPU9250_MAG.h"
#include <Library/utils/Macros.hpp>

MPU9250_MAG::MPU9250_MAG(
  MagSensor mag_sensor,
  const int sils_port_id,
  const unsigned int hils_port_id,
  const unsigned char i2c_addr,
  OBC* obc,
  HilsPortManager* hils_port_manager,
  const bool* is_mag_on
):MagSensor(mag_sensor), ObcI2cTargetCommunicationBase(sils_port_id, hils_port_id, i2c_addr, obc, hils_port_manager), is_mag_on_(is_mag_on)
{
}

void MPU9250_MAG::MainRoutine(int count)
{
  UNUSED(count);
  // Read Registers
  ReadCmdConfig();

  // Generate TLM
  if (*is_mag_on_ == true && config_ == 0x16) // Power ON and 100Hz Continuous Measurement Mode
  {
    mag_c_ = q_b2c_.frame_conv(magnet_->GetMag_b()); //Convert frame
    mag_c_ = Measure(mag_c_); //Add noises
    WriteMagTlm();
  }

  int cmd_size = ReceiveCommand();
  if (cmd_size != 1) return; // length == 1 means setting of read register address
  // これ以降はHILS用に事前にテレメトリを溜めておく
  const int kTlmSize = 8;
  StoreTelemetry(kStoredFrameSize, kTlmSize);
  return;
}

void MPU9250_MAG::ReadCmdConfig()
{
  unsigned char tmp[2] = {0xff, 0xff};
  ReadCommand(tmp, 2);
  if (tmp[0] != kCmdMagConfig_) return;

  config_ = tmp[1];

  return;
}

void MPU9250_MAG::WriteMagTlm()
{
  unsigned char tlm[] = {0, 0}; 
  unsigned char reg_id = kRegObsMag_;

  // Status
  WriteRegister(reg_id, &status_, 1);
  reg_id++;
  
  // MAG
  for (size_t i=0; i<kMagDim; i++)
  {
    double mag_c_uT = mag_c_[i] / 1000.0;
    Convert2Tlm(tlm, mag_c_uT * mag_convert_uT_to_raw_);
    WriteRegister(reg_id, tlm, kTlmSize_);
    reg_id += kTlmSize_;
  }

  return;
}

void MPU9250_MAG::Convert2Tlm(unsigned char tlm[2], const double value)
{
  signed short tlm_s = (signed short)(value);
  memcpy(tlm, &tlm_s, 2);

  return;
}
