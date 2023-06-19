#ifndef S2E_AOBC_SIMULATION_SPACECRAFT_AOCS_MODULE_PORT_CONFIG_HPP_
#define S2E_AOBC_SIMULATION_SPACECRAFT_AOCS_MODULE_PORT_CONFIG_HPP_

// コンポとポート番号の対応表

// H/WとしてPowerControllerで定義される
enum class PowerPortIdx {
  INA = 0,
  MPU,
  RM,
  SS,
  MTQ,
  STIM,
  STT,
  OEM,
  RWX,
  RWY,
  RWZ,
  MAX,
  PIC  // INA用にPICを用意する。電源操作されないのでMAXより大きくしておく。
};

// これはOBCソフトウェア側との対応と合わせること！
enum UARTPortConfig { GYRO = 0, UART_COMPONENT_MAX };

#endif  // S2E_AOBC_SIMULATION_SPACECRAFT_AOCS_MODULE_PORT_CONFIG_HPP_
