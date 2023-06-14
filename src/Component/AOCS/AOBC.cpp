#include "AOBC.h"

#include <library/utilities/macros.hpp>

#include "../../Simulation/Spacecraft/aocs_module_components.h"

AOBC::AOBC(ClockGenerator *clock_gen, AocsModuleComponents *components) : Component(100, clock_gen), components_(components) { Initialize(); }

AOBC::~AOBC() { delete components_; }

void AOBC::Initialize() {}

void AOBC::MainRoutine(int count) {
  UNUSED(count);
  // Sensor inputs
  // Vector<3> gyro_h_omega_c = components_->GetGyroH().GetOmegaC();
  // double rw_x_rpm = components_->GetRwX().GetVelocityRpm();

  // Debug out
  // cout << "GYRO_H_OMEGA_C = "; print(gyro_h_omega_c,', ',cout);cout << "rad/s
  // \n"; cout << "RW_X_RPM = " << rw_x_rpm << "rpm \n";

  // Actuator outputs
  // components_->GetRwX().SetDriveFlag(true);
  // components_->GetRwX().SetVelocityLimitRpm(3000);
  // components_->GetRwX().SetTargetTorqueRw(0.0001);

  // MTQ verification (note that you mast add some functions to ISSL6UComponents
  // class) Vector<3> mag_earth =
  // components_->GetMagH().measure(components_->GetLocalEnv().GetMag().GetMag_b());
  // Vector<3> mtq_torque; mtq_torque[0] = 0.2; mtq_torque[1] = -0.1;
  // mtq_torque[2] = 0.0; components_->GetMTQ().activate(mtq_torque,mag_earth);
  // Vector<3> mtq_torque_actual = components_->GetMTQ().GetMagTorque_b();
  // cout << "Mag_sensor =" << mag_earth[0] << "," << mag_earth[1] << "," <<
  // mag_earth[2] << "\n"; cout << "MTQ_torque =" << mtq_torque_actual[0] << ","
  // << mtq_torque_actual[1] << "," << mtq_torque_actual[2] << "\n";

  // Thruster verification (note that you mast add some functions to
  // ISSL6UComponents class) components_->GetThruster().set_duty(1.0); Vector<3>
  // center(0.0); Vector<3> vec_thrust =
  // components_->GetThruster().calc_thrust(); Vector<3> vec_torque =
  // components_->GetThruster().calc_torque(center,0); cout << "thruster =" <<
  // vec_thrust[0] << "," << vec_thrust[1] << "," << vec_thrust[2] << "\n";
}
