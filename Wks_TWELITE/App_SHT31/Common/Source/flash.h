/* Copyright (C) 2017 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-*J,*E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT).                                                   */

/** @file
 *
 * @defgroup FLASH FLASHメモリの読み書き関数群
 * FLASH への読み書き関数
 */

#ifndef FLASH_H_
#define FLASH_H_

#define FLASH_TYPE E_FL_CHIP_INTERNAL
#define FLASH_SECTOR_SIZE (32L* 1024L) // 32KB
#define FLASH_SECTOR_NUMBER 5 // 0..4

/** @ingroup FLASH
 * フラッシュ格納データ構造体
 */
typedef struct _tsFlashApp {
	uint32 u32appkey;		//!<
	uint32 u32ver;			//!<

	uint32 u32appid;		//!< アプリケーションID
	uint32 u32chmask;		//!< 使用チャネルマスク（３つまで）
	uint8 u8id;				//!< 論理ＩＤ (子機 1～100まで指定)
	uint8 u8ch;				//!< チャネル（未使用、チャネルマスクに指定したチャネルから選ばれる）
	uint8 u8pow;			//!< 出力パワー (0-3)
	uint8 u8role;			//!< 未使用(将来のための拡張)
	uint8 u8layer;			//!< 未使用(将来のための拡張)

//	uint16 u16SleepDur_ms;	//!< mode4 スリープ期間[ms]
	uint8 u8room;			//!< 部屋番号
	uint8 u8sensortype;		//!< センサータイプ
	uint16 u16SleepDur_s; 	//!< mode7 スリープ期間[s]
	uint8 u8Fps;			//!< mode3 毎秒送信回数 (4,8,16,32)

	uint32 u32baud_safe;	//!< ボーレート
	uint8 u8parity;         //!< パリティ 0:none, 1:odd, 2:even

	uint32 u32PWM_Hz[4];	//!< PWM変更設定周期

	uint32 u32Opt;			//!< 色々オプション
} tsFlashApp;

/** @ingroup FLASH
 * フラッシュデータ構造体
 * - u32Magic と u8CRC により、書き込みデータが有為かどうか判定する
 * - u8CRC は データ中の CRC8 チェックサム
 */
typedef struct _tsFlash {
	uint32 u32Magic;
	tsFlashApp sData;
	uint8 u8CRC;
} tsFlash;

bool_t bFlash_Read(tsFlash *psFlash, uint8 sector, uint32 offset);
bool_t bFlash_Write(tsFlash *psFlash, uint8 sector, uint32 offset);
bool_t bFlash_Erase(uint8 sector);

#endif /* FLASH_H_ */
