/**
 * @file crc.hpp
 * @brief CRC calculation functions
 */

#ifndef S2E_AOBC_LIBRARY_CRC_HPP_
#define S2E_AOBC_LIBRARY_CRC_HPP_

#include <stddef.h>
#include <stdint.h>

/**
 * @brief CRC-8-ATM
 *
 *        生成多項式: x^8 + x^2 + x + 1
 *        ビット送り: 左送り, POLLY: 0x07
 *        読み出し: 1byte(8 bit)
 * @param[in/out] crc: in:CRC初期値, out:CRC計算結果
 * @param[in] data: CRCを計算するbyte列
 * @param[in] size: 列の長さ
 * @param[in] rev_flag: 反転しない:false, 反転する:true
 * @return uint8_t: 計算結果
 */
uint8_t Crc8AtmLeft(uint8_t crc, const unsigned char *data, const size_t size, const bool rev_flag);

/**
 * @brief CRC-16-CCITT
 *
 *        生成多項式: x^16 + x^12 + x^5 + 1
 *        ビット送り: 右送り, POLLY: 0x8408
 *        読み出し:   1byte(8 bit)
 * @param[in/out] crc: in:CRC初期値, out:CRC計算結果
 * @param[in] data: CRCを計算するbyte列
 * @param[in] size: 列の長さ
 * @param[in] rev_flag: 反転しない:false, 反転する:true
 * @return uint16_t: 計算結果
 */
uint16_t Crc16CcittRight(uint16_t crc, const unsigned char *data, const size_t size, const bool rev_flag);

#endif  // S2E_AOBC_LIBRARY_CRC_HPP_
