/* Copyright (C) 2017 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-*J,*E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT).                                                   */

/** @file
 *
 * @defgroup SNSDRV センサー入力処理
 * センサー入力時の入力状態処理およびセンサー間の抽象化を行う。
 */

#ifndef SENSOR_DRIVER_H_
#include "jendefs.h"
#include <AppHardwareApi.h>

#include "ToCoNet_event.h"

#define SENSOR_TAG_DATA_NOTYET (-32768) //!< センサー入力がまだ @ingroup SNSDRV
#define SENSOR_TAG_DATA_ERROR  (-32767)  //!< センサー入力がエラー @ingroup SNSDRV
#define IS_SENSOR_TAG_DATA_ERR(c) (c < -32760) //!< その他エラー @ingroup SNSDRV

/** @ingroup SNSDRV
 * センサーの処理状態
 */
typedef enum
{
	E_SNSOBJ_STATE_IDLE = 0x00,    //!< 開始前
	E_SNSOBJ_STATE_CALIBRATE,      //!< 調整中
	E_SNSOBJ_STATE_MEASURING,      //!< 測定中
	E_SNSOBJ_STATE_MEASURE_NEXT,   //!< 次の測定処理中
	E_SNSOBJ_STATE_COMPLETE = 0x80,//!< 測定完了
	E_SNSOBJ_STATE_INACTIVE = 0xff //!< 動作しない状態
} teState_SnsObj;

/**  @ingroup SNSDRV
 * 管理構造体
 */
typedef struct {
	uint8 u8State; //!< 状態

	void *pvData; //!< センサーデータへのポインタ
	bool_t (*pvProcessSnsObj)(void *, teEvent); //!< イベント処理構造体
} tsSnsObj;

/** @ingroup SNSDRV
 * センサー状態マシンを駆動する。
 *
 * @param pObj 管理構造体
 * @param eEv イベント(イベントの種別は状態マシンにより決められるが、汎用的に ToCoNet で定義される E_EVENT_NEW_STATE/E_ORDER_KICK が使用される)
 */
void vSnsObj_Process(tsSnsObj *pObj, teEvent eEv);

#define vSnsObj_NewState(o, s) ((o)->u8State = (s)) //!< 新しい状態へ遷移する @ingroup SNSDRV
#define bSnsObj_isComplete(o) ((o)->u8State == E_SNSOBJ_STATE_COMPLETE) //!< 完了状態か判定する @ingroup SNSDRV

#define SENSOR_DRIVER_H_


#endif /* SENSOR_DRIVER_H_ */
