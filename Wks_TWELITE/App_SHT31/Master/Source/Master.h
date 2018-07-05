/* Copyright (C) 2017 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-*J,*E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT).                                                   */

/** @file
 * アプリケーションのメイン処理
 *
 * @defgroup MASTER アプリケーションのメイン処理
 */

#ifndef  MASTER_H_INCLUDED
#define  MASTER_H_INCLUDED

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <stdlib.h>
#include <string.h>

#include <jendefs.h>
#include <AppHardwareApi.h>

#include "ToCoNet.h"
#include "flash.h"
#include "btnMgr.h"
#include "adc.h"
#include "i2c_port.h"

/** @ingroup MASTER
 * 使用する無線チャネル数の最大値 (複数設定すると Channel Agility を利用する)
 */
#define MAX_CHANNELS 3

/** @ingroup MASTER
 * ネットワークのモード列挙体 (ショートアドレス管理か中継ネットワーク層を使用したものか)
 */
typedef enum {
	E_NWKMODE_MAC_DIRECT,//!< ネットワークは構成せず、ショートアドレスでの通信を行う
	E_NWKMODE_LAYERTREE  //!< 中継ネットワークでの通信を行う(これは使用しない)
} teNwkMode;

/** @ingroup MASTER
 * IO の状態
 */
typedef struct {
	uint32 u32BtmBitmap; //!< (0xFFFFFFFF: 未確定)
	uint32 u32BtmUsed; //!< 利用対象ピンかどうか (0xFFFFFFFF: 未確定)
	uint32 u32BtmChanged; //!< (0xFFFFFFFF: 未確定)

	uint16 au16InputADC[4]; //!< (0xFFFF: 未確定) 入力ポートの ADC 値 [mV]
	uint16 au16OutputDAC[4]; //!< (0xFFFF: 未確定) 送られてきた ADC 値 [mV]
	uint16 u16Volt; //!< 12bits, 0xFFFF: 未確定
	int16 i16Temp; //!< 12bits

	uint8 au8Input[4]; //!< 入力ポート (0: Hi, 1: Lo, 0xFF: 未確定)
	uint8 au8Output[4]; //!< 出力ポート (0: Hi, 1:Lo, 0xFF: 未確定)

	uint16 au16OutputPWMDuty[4]; //!< 無線経由で送られた DUTY 比 (0xFFFF: 未設定、無効)
	uint16 au16InputPWMDuty[4]; //!< 入力された AD に基づく DUTY 比の計算値 (0xFFFF: 未設定、無効)

	uint8 u8Volt; //!< i16Volt から変換

	uint32 u32TxLastTick; //!< 最後に送った時刻
	uint16 au16InputADC_LastTx[4]; //!< 最後に送信したデータ
	uint32 u32RxLastTick; //!< 最後に受信した時刻

	uint16 au16InputADC_History[4][4]; //!< ADCデータ履歴
	uint16 u16Volt_LastTx; //!< 最後に送信した電圧
	uint16 au16Volt_History[32]; //!< ADCデータ電圧履歴
	uint8 u8HistIdx; //!< 履歴情報のインデックス
	int16 i16TxCbId; //!< 送信時のID
} tsIOData;

#define HIST_VOLT_SCALE 5 //!< 電圧履歴数のスケーラ (2^HIST_VOLT_SCALE)  @ingroup MASTER
#define HIST_VOLT_COUNT (1UL << HIST_VOLT_SCALE) //!< 電圧履歴数 @ingroup MASTER


/** @ingroup MASTER
 * IO 設定要求
 */
typedef struct {
	uint8 u8IOports;          //!< 出力IOの状態 (1=Lo, 0=Hi)
	uint8 u8IOports_use_mask; //!< 設定を行うポートなら TRUE
	uint16 au16PWM_Duty[4];      //!< PWM DUTY 比 (0～1024)
	uint8 au16PWM_use_mask[4];   //!< 設定を行うPWMポートなら TRUE
} tsIOSetReq;

/** @ingroup MASTER
 * アプリケーションの情報
 */
typedef struct {
	// ToCoNet
	uint32 u32ToCoNetVersion; //!< ToCoNet のバージョン番号を保持
	uint16 u16ToCoNetTickDelta_ms; //!< ToCoNet の Tick 周期 [ms]
	uint8 u8AppIdentifier; //!< AppID から自動決定

	// メインアプリケーション処理部
	void *prPrsEv; //!< vProcessEvCoreSlpまたはvProcessEvCorePwrなどの処理部へのポインタ
	uint8 u8Hnd_vProcessEvCore; //!< vProcessEvCore のハンドル

	// DEBUG
	uint8 u8DebugLevel; //!< デバッグ出力のレベル

	// Wakeup
	bool_t bWakeupByButton; //!< TRUE なら起床時に DI 割り込みにより起床した
	uint32 u32SleepDur; //!< スリープ間隔 [ms]

	// 再送の標準的な回数
	uint8 u8StandardTxRetry;

	// mode3 fps
	uint8 u8FpsBitMask; //!< mode=3 連続送信時の秒間送信タイミングを判定するためのビットマスク (64fps のカウンタと AND を取って判定)

	// Network mode
	teNwkMode eNwkMode; //!< ネットワークモデル(未使用：将来のための拡張用)
	uint8 u8AppLogicalId; //!< ネットワーク時の抽象アドレス 0:親機 1~:子機, 0xFF:通信しない

	// Network context
	tsToCoNet_Nwk_Context *pContextNwk; //!< ネットワークコンテキスト(未使用)
	tsToCoNet_NwkLyTr_Config sNwkLayerTreeConfig; //!< LayerTree の設定情報(未使用)

	// 中継
	uint8 u8max_hops; //!< 最大中継ホップ数 (1-3)

	// Flash Information
	tsFlash sFlash; //!< フラッシュからの読み込みデータ
	tsFlashApp sConfig_UnSaved; //!< フラッシュへの設定データ (0xFF, 0xFFFFFFFF は未設定)
	int8 bFlashLoaded; //!< フラッシュからの読み込みが正しく行われた場合は TRUE

	uint32 u32DIO_startup; //!< 電源投入時のIO状態

	// config mode
	uint8 u8Mode; //!< 動作モード(IO M1,M2,M3 から設定される)

	// button manager
	tsBTM_Config sBTM_Config; //!< ボタン入力（連照により状態確定する）管理構造体
	PR_BTM_HANDLER pr_BTM_handler; //!< ボタン入力用のイベントハンドラ (TickTimer 起点で呼び出す)
	uint32 u32BTM_Tick_LastChange; //!< ボタン入力で最後に変化が有ったタイムスタンプ (操作の無効期間を作る様な場合に使用する)

	// ADC
	tsObjData_ADC sObjADC; //!< ADC管理構造体（データ部）
	tsSnsObj sADC; //!< ADC管理構造体（制御部）
	bool_t bUpdatedAdc; //!< TRUE:ADCのアップデートが有った。アップデート検出後、FALSE に戻す事。
	uint8 u8AdcState; //!< ADCの状態 (0xFF:初期化前, 0x0:ADC開始要求, 0x1:AD中, 0x2:AD完了)

	// latest state
	tsIOData sIOData_now; //!< 現時点での IO 情報
	tsIOData sIOData_reserve; //!< 保存された状態(0がデフォルトとする)
	uint8 u8IOFixState; //!< IOの読み取り確定ビット
	uint32 u32AdcLastTick; //!< 最後に ADC を開始した時刻

	// Counter
	uint32 u32CtTimer0; //!< 64fps カウンタ。スリープ後も維持
	uint16 u16CtTimer0; //!< 64fps カウンタ。起動時に 0 クリアする
	uint16 u16CtRndCt; //!< 起動時の送信タイミングにランダムのブレを作る

	uint8 u8UartReqNum; //!< UART の要求番号

	uint16 u16TxFrame; //!< 送信フレーム数
	uint8 u8SerMsg_RequestNumber; //!< シリアルメッセージの要求番号

	// I2C
	uint8 u8measure;  //!< 0でなければ残り測定時間
	uint8 u8ms_state;  //!< 測定状態のステートマシン
	uint16 u16temperature;  //!< 温度データ
	uint16 u16humidity;  //!< 湿度データ
	uint16 u16status;  //!< ステータスレジスタの状態
} tsAppData;

/****************************************************************************
 * フラッシュ設定情報
 ***************************************************************************/

#define FL_MASTER_u32(c) sAppData.sFlash.sData.u32##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_UNSAVE_u32(c) sAppData.sConfig_UnSaved.u32##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_IS_MODIFIED_u32(c) (sAppData.sConfig_UnSaved.u32##c != 0xFFFFFFFF)  //!< 構造体要素アクセス用のマクロ @ingroup FLASH

#define FL_MASTER_u16(c) sAppData.sFlash.sData.u16##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_UNSAVE_u16(c) sAppData.sConfig_UnSaved.u16##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_IS_MODIFIED_u16(c) (sAppData.sConfig_UnSaved.u16##c != 0xFFFF)  //!< 構造体要素アクセス用のマクロ @ingroup FLASH

#define FL_MASTER_u8(c) sAppData.sFlash.sData.u8##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_UNSAVE_u8(c) sAppData.sConfig_UnSaved.u8##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_IS_MODIFIED_u8(c) (sAppData.sConfig_UnSaved.u8##c != 0xFF) //!< 構造体要素アクセス用のマクロ @ingroup FLASH

/** @ingroup FLASH
 * フラッシュ設定内容の列挙体
 */
enum {
	E_APPCONF_APPID,     //!< アプリケーションID
	E_APPCONF_CHMASK,    //!< チャネルマスク
	E_APPCONF_TX_POWER,  //!< TX 出力
	E_APPCONF_ID,        //!< 8bitのID(ネットワークアドレス)
	E_APPCONF_ROLE,      //!<
	E_APPCONF_LAYER ,    //!<
//	E_APPCONF_SLEEP4,    //!< mode4 のスリープ期間設定
	E_APPCONF_ROOM,	     //!< 部屋番号
	E_APPCONF_STYPE,     //!< センサー種別
	E_APPCONF_SLEEP7,    //!< mode7 のスリープ期間設定
	E_APPCONF_FPS,       //!< 連続送信モードの秒あたりの送信数
	E_APPCONF_PWM_HZ,    //!< PWM の周波数
	E_APPCONF_SYS_HZ,    //!<
	E_APPCONF_OPT,       //!< DIOの入力方法に関する設定
	E_APPCONF_BAUD_SAFE, //!< BPS ピンをGにしたときのボーレート
	E_APPCONF_BAUD_PARITY, //!< BPS ピンをGにしたときのパリティ設定 (0:None, 1:Odd, 2:Even)
	E_APPCONF_TEST
};

enum {
	E_MS_IDLE = 0,	     //!< 何も実行していない
	E_MS_EXEC,	     //!< 測定中
	E_MS_STATUS,	     //!< ステータス読み出し/変更中
	E_MS_DONE	     //!< 全ての動作が終了した状態
};

/** @ingroup FLASH
 * フラッシュ設定で ROLE に対する要素名の列挙体
 * (未使用、将来のための拡張のための定義)
 */
enum {
	E_APPCONF_ROLE_MAC_NODE = 0,  //!< MAC直接のノード（親子関係は無し）
	E_APPCONF_ROLE_NWK_MASK = 0x10, //!< NETWORKモードマスク
	E_APPCONF_ROLE_PARENT,          //!< NETWORKモードの親
	E_APPCONF_ROLE_ROUTER,        //!< NETWORKモードの子
	E_APPCONF_ROLE_ENDDEVICE,     //!< NETWORKモードの子（未使用、スリープ対応）
	E_APPCONF_ROLE_SILENT = 0x7F, //!< 何もしない（設定のみ)
};

#define E_APPCONF_OPT_LOW_LATENCY_INPUT			 	0x000001UL //!< Hi>Lo を検知後直ぐに送信する。 @ingroup FLASH
#define E_APPCONF_OPT_REGULAR_PACKET_NO_TRANSMIT 	0x000002UL //!< 定期送信を行わない(1秒置き)
#define E_APPCONF_OPT_REGULAR_PACKET_NO_DISP 		0x000004UL //!< 定期送信パケットを表示しない(1秒置きと連射)
#define E_APPCONF_OPT_NO_ADC_BASED_TRANSMIT			0x000010UL //!< ADCの変化に応じた送信を禁止する。 @ingroup FLASH
#define E_APPCONF_OPT_DISABLE_ADC					0x000020UL //!< ADCを計測しない。 @ingroup FLASH
#define E_APPCONF_OPT_ADC_TO_PWM_RAW_OUT			0x000040UL //!< ADC絶対値でPWM出力する @ingroup FLASH
#define E_APPCONF_OPT_ON_PRESS_TRANSMIT				0x000100UL //!< 押し下げ時のみ送信する特殊動作モード。 @ingroup FLASH
#define E_APPCONF_OPT_INQUIRE_ON_WAKE				0x000200UL //!< ウェイクアップ時に親機に対して問い合わせを行う。 @ingroup FLASH
#define E_APPCONF_OPT_NO_PULLUP_FOR_INPUT			0x000800UL //!< DI1-4 入力にプルアップを適用しない。@ingroup FLASH
#define E_APPCONF_OPT_ROUTING_HOP2					0x001000UL //!< 中継段数を２にする。 @ingroup FLASH
#define E_APPCONF_OPT_ROUTING_HOP3					0x002000UL //!< 中継段数な３にする。 @ingroup FLASH
#define E_APPCONF_OPT_ROUTING_CHILD					0x008000UL //!< 中継可能な子機設定。 @ingroup FLASH
#define E_APPCONF_OPT_PWM_INVERT					0x010000UL //!< PWMをDUTYを反転する。 @ingroup FLASH
#define E_APPCONF_OPT_PWM_INIT_LOW					0x020000UL //!< PWMを起床時にLOとする。 @ingroup FLASH
#define E_APPCONF_OPT_PWM_MOVE_PORTS				0x080000UL //!< PWMの利用ポートを入れ替える (ただし I2C, BPS ピンは無効となる) @ingroup FLASH
#define E_APPCONF_OPT_DIO_LOW_ON_BOOT				0x100000UL //!< 起床時にDIOを２秒だけLOにする (LED点灯) @ingroup FLASH
#define E_APPCONF_OPT_PWM_LOW_ON_BOOT				0x200000UL //!< 起床時にPWMを２秒だけ(AI=100%値)にする (LED点灯) @ingroup FLASH
#define E_APPCONF_OPT_DO_INVERT						0x400000UL //!< DIO出力を反転する @ingroup FLASH
#define E_APPCONF_OPT_NO_PULLUP_FOR_OUTPUT			0x800000UL //!< 出力ポート DO1-4, PWM1-4 のプルアップを停止する @ingroup FLASH

#define IS_APPCONF_OPT_LOW_LATENCY_INPUT() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_LOW_LATENCY_INPUT) //!< E_APPCONF_OPT_LOW_LATENCY_INPUT 判定 @ingroup FLASH
#define IS_APPCONF_OPT_REGULAR_PACKET_NO_TRANSMIT() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_REGULAR_PACKET_NO_TRANSMIT) //!< E_APPCONF_OPT_REGULAR_PACKET_NO_TRANSMIT 判定 @ingroup FLASH
#define IS_APPCONF_OPT_REGULAR_PACKET_NO_DISP() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_REGULAR_PACKET_NO_DISP) //!< E_APPCONF_OPT_REGULAR_PACKET_NO_DISP 判定 @ingroup FLASH
#define IS_APPCONF_OPT_NO_ADC_BASED_TRANSMIT() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_NO_ADC_BASED_TRANSMIT) //!< E_APPCONF_OPT_NO_ADC_BASED_TRANSMIT判定 @ingroup FLASH
#define IS_APPCONF_OPT_DISABLE_ADC() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_DISABLE_ADC) //!< E_APPCONF_OPT_DISABLE_ADC判定 @ingroup FLASH
#define IS_APPCONF_OPT_ADC_TO_PWM_RAW_OUT() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_ADC_TO_PWM_RAW_OUT) //!< E_APPCONF_OPT_ADC_TO_PWM_RAW_OUT判定 @ingroup FLASH
#define IS_APPCONF_OPT_ON_PRESS_TRANSMIT() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_ON_PRESS_TRANSMIT) //!< E_APPCONF_OPT_ON_PRESS_TRANSMIT判定 @ingroup FLASH
#define IS_APPCONF_OPT_INQUIRE_ON_WAKE() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_INQUIRE_ON_WAKE) //!< E_APPCONF_OPT_INQUIRE_ON_WAKE判定 @ingroup FLASH
#define IS_APPCONF_OPT_NO_PULLUP_FOR_INPUT() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_NO_PULLUP_FOR_INPUT) //!< E_APPCONF_OPT_NO_PULLUP_FOR_INPUT判定 @ingroup FLASH
#define IS_APPCONF_OPT_ROUTING_CHILD() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_ROUTING_CHILD) //!< E_APPCONF_OPT_ROUTING_END_DEVICE判定 @ingroup FLASH
#define IS_APPCONF_OPT_ROUTING_HOP2() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_ROUTING_HOP2) //!< E_APPCONF_OPT_ROUTING_HOP2判定 @ingroup FLASH
#define IS_APPCONF_OPT_ROUTING_HOP3() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_ROUTING_HOP3) //!< E_APPCONF_OPT_ROUTING_HOP3判定 @ingroup FLASH
#define IS_APPCONF_OPT_PWM_INVERT() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_PWM_INVERT) //!< E_APPCONF_OPT_PWM_INVERT判定 @ingroup FLASH
#define IS_APPCONF_OPT_PWM_INIT_LOW() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_PWM_INIT_LOW) //!< E_APPCONF_OPT_PWM_INIT_LOW判定 @ingroup FLASH
#define IS_APPCONF_OPT_PWM_MOVE_PORTS() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_PWM_MOVE_PORTS) //!< E_APPCONF_OPT_PWM_MOVE_PORTS判定 @ingroup FLASH
#define IS_APPCONF_OPT_DIO_LOW_ON_BOOT() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_DIO_LOW_ON_BOOT) //!< E_APPCONF_OPT_DIO_LOW_ON_BOOT判定 @ingroup FLASH
#define IS_APPCONF_OPT_PWM_LOW_ON_BOOT() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_PWM_LOW_ON_BOOT) //!< E_APPCONF_OPT_PWM_LOW_ON_BOOT判定 @ingroup FLASH
#define IS_APPCONF_OPT_DO_INVERT() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_DO_INVERT) //!< E_APPCONF_OPT_DO_INVERT判定 @ingroup FLASH
#define IS_APPCONF_OPT_NO_PULLUP_FOR_OUTPUT() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_NO_PULLUP_FOR_OUTPUT) //!< E_APPCONF_OPT_NO_PULLUP_FOR_OUTPUT判定 @ingroup FLASH

/** サイレントモードの判定マクロ  @ingroup FLASH */
#define IS_APPCONF_ROLE_SILENT_MODE() (sAppData.sFlash.sData.u8role == E_APPCONF_ROLE_SILENT)

/** PWM値の反転を考慮した値を設定する */
#define _PWM(c) (IS_APPCONF_OPT_PWM_INVERT() ? (1024-c) : c)

/** DO値をセットする */
#define vDoSetLo(c) (IS_APPCONF_OPT_DO_INVERT() ? vPortSetHi(c) : vPortSetLo(c))
#define vDoSetHi(c) (IS_APPCONF_OPT_DO_INVERT() ? vPortSetLo(c) : vPortSetHi(c))
#define vDoSet_TrueAsLo(c,f) vPortSet_TrueAsLo((c), (IS_APPCONF_OPT_DO_INVERT() ? ((f) == FALSE) : (f)))

#endif  /* MASTER_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
