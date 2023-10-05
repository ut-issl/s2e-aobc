/**
 * @file aocs_module_port_config.hpp
 * @brief Port configuration definition of AOCS module
 */

#ifndef S2E_AOBC_SIMULATION_SPACECRAFT_AOCS_MODULE_PORT_CONFIG_HPP_
#define S2E_AOBC_SIMULATION_SPACECRAFT_AOCS_MODULE_PORT_CONFIG_HPP_

/**
 * @enum PowerPortIdx
 * @brief Index of power port
 * @note It is defined by PowerController as H/W
 */
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

#endif  // S2E_AOBC_SIMULATION_SPACECRAFT_AOCS_MODULE_PORT_CONFIG_HPP_
