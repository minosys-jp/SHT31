#ifndef I2C_PORT_H_
#define I2C_PORT_H_

#include <stdlib.h>
#include <string.h>

#include <jendefs.h>
#include <AppHardwareApi.h>

#define I2C_WAIT (5)

/**
 * ポート制御で I2C を実現する
 * 現在のところ、SHT31 専用である
 */

/**
 * I2C バスを初期化する
 */
void vI2C_Init();

/**
 * I2C バスをリセットする
 */
void vI2C_BusReset();

/**
 * SCK ストレッチの検出
 */
bool_t bI2C_SlaveReady();

/**
 * I2C データの送信
 */
bool_t bI2C_WriteData(uint8 addr, uint8 len, uint8 *pbuf);

/**
 * I2C データの受信
 */
bool_t bI2C_ReadData(uint8 addr, uint8 len, uint8 *pbuf);

#endif //I2C_PORT_H_
