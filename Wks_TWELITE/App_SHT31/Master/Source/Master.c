/* Copyright (C) 2017 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-*J,*E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT).                                                   */

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <string.h>
#include <AppHardwareApi.h>

#include "Master.h"

#include "ccitt8.h"
#include "Interrupt.h"

#include "Version.h"

#include "utils.h"
#include "flash.h"

#include "common.h"
#include "config.h"

// IO Read Options
#include "btnMgr.h"
// ADC
#include "adc.h"
// I2C
#include "SMBus.h"

// 重複チェッカ
#include "duplicate_checker.h"

// Serial options
#include <serial.h>
#include <fprintf.h>
#include <sprintf.h>

#include "modbus_ascii.h"
#include "input_string.h"

/****************************************************************************/
/***        ToCoNet Definitions                                           ***/
/****************************************************************************/
// Select Modules (define befor include "ToCoNet.h")
#define ToCoNet_USE_MOD_TXRXQUEUE_BIG
#define ToCoNet_USE_MOD_CHANNEL_MGR

// includes
#include "ToCoNet.h"
#include "ToCoNet_mod_prototype.h"

#include "app_event.h"

// 実験的な実装
#include "Experimental.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define TX_NODELAY_AND_QUICK_BIT 1
#define TX_NODELAY 2
#define TX_NODELAY_AND_RESP_BIT 3
#define TX_SMALLDELAY 4
#define TX_REPONDING 5
#ifdef USE_MONOSTICK
#define LED_FLASH_MS 500TRUE;
#define DEBUG_WD 0
#endif

#define SHT31_I2C_ADDR (0x45)		//!< 秋月電子 SHT31 キットのアドレス
#define SHT31_I2C_CMD_MS (0x24)		//!< 単発測定コマンド
#define SHT31_I2C_CMD_MS_LO (0x16)	//!< 低精度測定モード
#define SHT31_I2C_CMD_ST (0xf3)		//!< ステータス読み出しコマンド
#define SHT31_I2C_CMD_ST_1 (0x2d)	//!< ステータス読み出しコマンド
#define SHT31_I2C_CMD_STRS (0x30)	//!< ステータス消去コマンド
#define SHT31_I2C_CMD_STRS_1 (0x41)	//!< ステータス消去コマンド
#define SHT31_I2C_CMD_RST (0x30)	//!< リセットコマンド「
#define SHT31_I2C_CMD_RST_1 (0xa2)	//!< リセットコマンド

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);

static void vProcessEvCorePwr(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);
static void vProcessEvCoreSlp(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);
static void vProcessEvCoreSlpRecv(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);

static void vInitHardware(int f_warm_start);

static void vSerialInit(uint32, tsUartOpt *);
static void vProcessSerialCmd(tsModbusCmd *pSer);
static void vProcessInputByte(uint8 u8Byte);
static void vProcessInputString(tsInpStr_Context *pContext);
static void vHandleSerialInput();
static void vSerUpdateScreen();
static void vSerInitMessage();

static void vReceiveSerMsg(tsRxDataApp *pRx);
static void vReceiveIoData(tsRxDataApp *pRx);
static void vReceiveIoSettingRequest(tsRxDataApp *pRx);

static bool_t bCheckDupPacket(tsDupChk_Context *pc, uint32 u32Addr,
		uint16 u16TimeStamp);

static int16 i16TransmitIoData(uint8 u8Quick, bool_t bRegular);
static int16 i16TransmitIoSettingRequest(uint8 u8DstAddr, tsIOSetReq *pReq);
static int16 i16TransmitRepeat(tsRxDataApp *pRx);
static int16 i16TransmitSerMsg(uint8 *p, uint16 u16len, uint32 u32AddrSrc,
		uint8 u8AddrSrc, uint8 u8AddrDst, uint8 u8RelayLv, uint8 u8Req);
static int16 i16TransmitSerData();

#if 0
static void vProcessI2CCommand(uint8 *p, uint16 u16len, uint8 u8AddrSrc);
#ifdef USE_I2C_ACM1620
static bool_t bDraw2LinesLcd_ACM1602(const char *puUpperRow,
		const char *puLowerRow);
#endif
#ifdef USE_I2C_AQM0802A
static bool_t bDraw2LinesLcd_AQM0802A(const char *puUpperRow,
		const char *puLowerRow);
#endif
#endif

static uint16 u16GetAve(uint16 *pu16k, uint8 u8Scale);
static bool_t bUpdateAdcValues();

static void vConfig_SetDefaults(tsFlashApp *p);
static void vConfig_UnSetAll(tsFlashApp *p);
static void vConfig_SaveAndReset();

static void vSleep(uint32 u32SleepDur_ms, bool_t bPeriodic, bool_t bDeep);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
static tsAppData sAppData; //!< アプリケーションデータ  @ingroup MASTER

tsFILE sSerStream; //!< シリアル出力ストリーム  @ingroup MASTER
tsSerialPortSetup sSerPort; //!< シリアルポートの設定  @ingroup MASTER

tsModbusCmd sSerCmd; //!< シリアル入力系列のパーサー (modbus もどき)  @ingroup MASTER
tsSerSeq sSerSeqTx; //!< 分割パケット管理構造体（送信用）  @ingroup MASTER
uint8 au8SerBuffTx[(SERCMD_MAXPAYLOAD + 32) * 2]; //!< sSerSeqTx 用に確保  @ingroup MASTER
tsSerSeq sSerSeqRx; //!< 分割パケット管理構造体（受信用）  @ingroup MASTER
uint8 au8SerBuffRx[SERCMD_MAXPAYLOAD + 32]; //!< sSerSeqRx 用に確保  @ingroup MASTER
tsInpStr_Context sSerInpStr; //!< 文字列入力  @ingroup MASTER
static uint16 u16HoldUpdateScreen = 0; //!< スクリーンアップデートを行う遅延カウンタ  @ingroup MASTER

tsTimerContext sTimerApp; //!< タイマー管理構造体  @ingroup MASTER
tsTimerContext sTimerPWM[4]; //!< タイマー管理構造体  @ingroup MASTER

uint8 au8SerOutBuff[128]; //!< シリアルの出力書式のための暫定バッファ  @ingroup MASTER

tsDupChk_Context sDupChk_IoData; //!< 重複チェック(IO関連のデータ転送)  @ingroup MASTER
tsDupChk_Context sDupChk_SerMsg; //!< 重複チェック(シリアル関連のデータ転送)  @ingroup MASTER

#ifdef USE_MONOSTICK
bool_t bColdStart = TRUE; //!< MoNoStickの時の起動時にLEDを光らせるためのフラグ @ingroup MASTER
bool_t bWDTest = TRUE;
#endif

/****************************************************************************/
/***        FUNCTIONS                                                     ***/
/****************************************************************************/

/** @ingroup MASTER
 * アプリケーションの基本制御状態マシン。
 * - 特別な処理は無い。
 *
 * @param pEv
 * @param eEvent
 * @param u32evarg
 */
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
	switch (pEv->eState) {
	case E_STATE_IDLE:
		if (eEvent == E_EVENT_START_UP) {
#ifdef USE_BROWN_OUT_DETECT
			// BrownOut 検出の有効化
			vAHI_BrownOutConfigure(
					1,// 1:2.0V, 4:2.3V
					TRUE,
					TRUE,
					FALSE,
					FALSE);

			//
			vAHI_WatchdogStop();
			vAHI_WatchdogStart(4);
#endif

			if (IS_APPCONF_ROLE_SILENT_MODE()) {
				vfPrintf(&sSerStream, LB"!Note: launch silent mode."LB);
				ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
			} else {
				// LayerNetwork で無ければ、特別な動作は不要。
				// run as default...

				// 始動メッセージの表示
				if (!(u32evarg & EVARG_START_UP_WAKEUP_MASK)) {
					vSerInitMessage();
				}

				// RUNNING 状態へ遷移
				ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
			}

			break;
		}

		break;

	case E_STATE_RUNNING:
		break;
	default:
		break;
	}
}

/** @ingroup MASTER
 * アプリケーション制御（電源常時 ON モード）
 * - 機能概要
 *   - 起動時にランダムで処理を保留する（同時起動による送信パケットの競合回避のため）
 *   - 初回のDI/AD状態確定まで待つ
 *   - 実行状態では E_EVENT_APP_TICK_A (64fps タイマーイベント) を起点に処理する。
 *     - 32fps のタイミングで送信判定を行う
 *     - 定期パケット送信後は、次回のタイミングを乱数によってブレを作る。
 *
 * - 状態一覧
 *   - E_STATE_IDLE\n
 *     起動直後に呼び出される状態で、同時起動によるパケット衝突を避けるためランダムなウェイトを置き、次の状態に遷移する。
 *   - E_STATE_APP_WAIT_IO_FIRST_CAPTURE\n
 *     初回に DI および AI の入力値が確定するまでの待ちを行い、E_STATE_RUNNING に遷移する。
 *   - E_STATE_RUNNING
 *     秒６４回のタイマー割り込み (E_EVENT_TICK_A) を受けて、入力状態の変化のチェックを行い、無線パケットの送信要求を
 *     発行する。各種判定条件があるので、詳細はコード中のコメント参照。
 *
 * @param pEv
 * @param eEvent
 * @param u32evarg
 */
void vProcessEvCorePwr(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
	switch (pEv->eState) {
	case E_STATE_IDLE:
		if (eEvent == E_EVENT_START_UP) {
			sAppData.u16CtRndCt = 0;
		}

		if (eEvent == E_EVENT_TICK_TIMER) {
			static bool_t bStarted = FALSE;

			if (!sAppData.u16CtRndCt) {
				sAppData.u8AdcState = 0; // ADC の開始
				bStarted = TRUE;
				sAppData.u16CtRndCt = (ToCoNet_u16GetRand() & 0xFF) + 10; // 始動時にランダムで少し待つ（同時電源投入でぶつからないように）
			}
		}

		// 始動時ランダムな待ちを置く
		if (sAppData.u16CtRndCt
				&& PRSEV_u32TickFrNewState(pEv) > sAppData.u16CtRndCt) {
			ToCoNet_Event_SetState(pEv, E_STATE_APP_WAIT_IO_FIRST_CAPTURE);
			sAppData.u16CtRndCt = 32; // この変数は定期送信のタイミング用に再利用する。
		}

		break;

	case E_STATE_APP_WAIT_IO_FIRST_CAPTURE:
		// 起動直後の未確定状態
		if (eEvent == E_EVENT_APP_TICK_A) {
			if (sAppData.u8IOFixState == 0x03) {
				if (IS_APPCONF_OPT_DIO_LOW_ON_BOOT() || IS_APPCONF_OPT_PWM_LOW_ON_BOOT()) {
					ToCoNet_Event_SetState(pEv, E_STATE_APP_SET_INITIAL_ON);
				} else {
					ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
				}
			}

			// 起動後から最初の定期送信までの間 au16InputADC_LastTx[] に値が入らない。
			// ここで強制的に値を入れておく
			int i;
			for (i = 0; i < 4; i++) {
				sAppData.sIOData_now.au16InputADC_LastTx[i] = sAppData.sIOData_now.au16InputADC[i];
			}
		}
		break;

	case E_STATE_APP_SET_INITIAL_ON:
		// 始動時に一定時間全ポートをLoに設定する
		if (eEvent == E_EVENT_NEW_STATE) {
			if (IS_APPCONF_OPT_DIO_LOW_ON_BOOT()) {
				vDoSetLo(PORT_OUT1);
				vDoSetLo(PORT_OUT2);
				vDoSetLo(PORT_OUT3);
				vDoSetLo(PORT_OUT4);
			}

			if (IS_APPCONF_OPT_PWM_LOW_ON_BOOT()) {
				int i;
				for (i = 0; i < 4; i++) {
					sTimerPWM[i].u16duty = _PWM(0);
					vTimerStart(&sTimerPWM[i]); // DUTY比だけ変更する
				}
			}
		}

		if (PRSEV_u32TickFrNewState(pEv) > 2000) {
			if (IS_APPCONF_OPT_DIO_LOW_ON_BOOT()) {
				vDoSetHi(PORT_OUT1);
				vDoSetHi(PORT_OUT2);
				vDoSetHi(PORT_OUT3);
				vDoSetHi(PORT_OUT4);
			}

			if (IS_APPCONF_OPT_PWM_LOW_ON_BOOT()) {
				int i;
				for (i = 0; i < 4; i++) {
					sTimerPWM[i].u16duty = _PWM(1024);
					vTimerStart(&sTimerPWM[i]); // DUTY比だけ変更する
				}
			}

			ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
		}
		break;

	case E_STATE_RUNNING:
		if (sAppData.u8Mode == E_IO_MODE_ROUTER)
			break; // リピーターは何もしない。

#ifdef ON_PRESS_TRANSMIT
		// 一定期間受信が無い場合、IO状態をHIに自動で戻す処理
		static uint32 u32TxLastDiClear = 0x80000000;
		if (IS_APPCONF_OPT_ON_PRESS_TRANSMIT()) {
			if (u32TickCount_ms - sAppData.sIOData_now.u32RxLastTick > ON_PRESS_TRANSMIT_RESET_ms) { // 500ms で自動停止
				int i;

				// ポートの値を設定する（変更フラグのあるものだけ）
				for (i = 0; i < 4; i++) {
					vDoSetHi(au8PortTbl_DOut[i]);
					sAppData.sIOData_now.au8Output[i] = 0;
				}
			}

			// 最後に停止してからの Tick
			if (sAppData.sIOData_now.u32BtmChanged && !sAppData.sIOData_now.u32BtmBitmap && sAppData.sIOData_now.u32BtmUsed) {
				u32TxLastDiClear = u32TickCount_ms;
			}
		}
#endif
		if (eEvent == E_EVENT_APP_TICK_A // 秒64回のタイマー割り込み
		&& (sAppData.u32CtTimer0 & 1) // 秒32回にする
				) {
			// 変更が有った場合は送信する
			int i;
			bool_t bCond = FALSE;
			bool_t bRegular = FALSE;

			if (sAppData.u16CtRndCt)
				sAppData.u16CtRndCt--; // 定期パケット送信までのカウントダウン

			// 初回送信を実施する
			static bool_t bFirstTransmit = FALSE; // 電源投入直後の送信
			if (!bFirstTransmit) {
				if (!IS_APPCONF_OPT_REGULAR_PACKET_NO_TRANSMIT()) {
					bCond = TRUE;
					bRegular = TRUE;
				}
				bFirstTransmit = TRUE;
			}

			// ボタンに変化あり
			if (!bCond && sAppData.sIOData_now.u32BtmChanged) {
				bCond = TRUE;
			}

			// ADC に変化あり
			if (!bCond && !IS_APPCONF_OPT_NO_ADC_BASED_TRANSMIT() && sAppData.bUpdatedAdc) {
				bCond = TRUE;
			}

			// ON PRESS TRANSMIT
			if (!bCond && IS_APPCONF_OPT_ON_PRESS_TRANSMIT()) {
				// どれかボタンが押されているときは送信を続ける
				if (sAppData.sIOData_now.u32BtmBitmap && sAppData.sIOData_now.u32BtmBitmap != 0xFFFFFFFF) {
					bCond = TRUE;
				}

				if ((!sAppData.sIOData_now.u32BtmBitmap) && (u32TickCount_ms - u32TxLastDiClear < ON_PRESS_TRANSMIT_KEEP_TX_ms)) {
					// ボタンが離されてから 1000ms 未満
					bCond = TRUE;
				}
			}

			if (!bCond
					&& sAppData.u8Mode == E_IO_MODE_CHILD_CONT_TX
					&& ((sAppData.u32CtTimer0 & sAppData.u8FpsBitMask) == sAppData.u8FpsBitMask))  {
				// 打って打って打ちまくれ！のモード
				bCond = TRUE;
				bRegular = TRUE;
			}

			// レギュラー送信
			if (!bCond && (sAppData.u16CtRndCt == 0)) {
				if (!IS_APPCONF_OPT_REGULAR_PACKET_NO_TRANSMIT()) {
					bCond = TRUE;
					bRegular = TRUE;
				}
			}

			// 送信
			if (bCond) {
				// デバッグ出力
				DBGOUT(5,
						"A(%02d/%04d)%d%d: v=%04d A1=%04d/%04d A2=%04d/%04d B=%d%d%d%d %08x"LB,
						sAppData.u32CtTimer0, u32TickCount_ms & 8191,
						sAppData.bUpdatedAdc ? 1 : 0,
						sAppData.sIOData_now.u32BtmChanged ? 1 : 0,
						sAppData.sIOData_now.u16Volt,
						sAppData.sIOData_now.au16InputADC[0],
						sAppData.sIOData_now.au16InputPWMDuty[0] >> 2,
						sAppData.sIOData_now.au16InputADC[1],
						sAppData.sIOData_now.au16InputPWMDuty[1] >> 2,
						sAppData.sIOData_now.au8Input[0] & 1,
						sAppData.sIOData_now.au8Input[1] & 1,
						sAppData.sIOData_now.au8Input[2] & 1,
						sAppData.sIOData_now.au8Input[3] & 1,
						sAppData.sIOData_now.u32BtmBitmap);

				// 低遅延送信が必要かどうかの判定
				bool_t bQuick = FALSE;
				if (sAppData.sIOData_now.u32BtmChanged
						&& (sAppData.sFlash.sData.u32Opt
								& E_APPCONF_OPT_LOW_LATENCY_INPUT)) {
					bQuick = TX_NODELAY_AND_QUICK_BIT; // 低レイテンシ専用
				}

				// 送信要求
				sAppData.sIOData_now.i16TxCbId = i16TransmitIoData(bQuick, bRegular);

				// 変更フラグのクリア
				sAppData.bUpdatedAdc = FALSE;
				sAppData.sIOData_now.u32BtmChanged = 0;

				// AD履歴の保存
				for (i = 0; i < 4; i++) {
					sAppData.sIOData_now.au16InputADC_LastTx[i] =
							sAppData.sIOData_now.au16InputADC[i];
				}

				// 次の定期パケットのタイミングを仕込む
				sAppData.u16CtRndCt = (ToCoNet_u16GetRand() & 0xF) + 24;
			}
		}
		break;

	default:
		break;
	}
}

/**  @ingroup MASTER
 * アプリケーション制御（スリープ稼動モード）\n
 * 本状態遷移マシンは、mode=4, mode=7 で起動したときに登録され、測定完了待ち⇒送信⇒
 * 送信完了待ちを実施し、その後、再びスリープを実行する。
 *
 * - 機能概要
 *   - ADやDIの状態が確定するまで待つ。
 *   - 送信する。
 *   - 送信完了を待つ。
 *   - スリープする。
 *
 * - 状態一覧
 *   - E_STATE_IDLE\n
 *     起動直後またはスリープ復帰後の状態。UARTにメッセージを表示し、最初の TickTimer で
 *     E_STATE_RUNNING に遷移する。
 *   - E_STATE_RUNNING\n
 *     IO状態の確定を待って、無線送信要求、E_STATE_WAIT_TX へ遷移。
 *   - E_STATE_WAIT_TX\n
 *     送信完了イベントを待つ。実際は cbToCoNet_TxEvent() よりコールされる。
 *   - E_STATE_FINISHED\n
 *     スリープ条件が成立するまでの待ちを行う。具体的にはボタン駆動した時にチャタリングの
 *     影響が去るまでの時間待ちである。
 *   - E_STATE_APP_SLEEPING\n
 *     スリープ処理を行う。
 *
 * @param pEv
 * @param eEvent
 * @param u32evarg
 */
void vProcessEvCoreSlp(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
	switch (pEv->eState) {
	case E_STATE_IDLE:
		if (eEvent == E_EVENT_START_UP) {

			// vfPrintf(&sSerStream, "START_UP"LB, eEvent);
			if (u32evarg & EVARG_START_UP_WAKEUP_MASK) {
				// スリープからの復帰時の場合
				vfPrintf(&sSerStream, "!INF %s WAKE UP."LB,
						sAppData.bWakeupByButton ? "DI" : "TIMER");
			} else {
#ifdef SET_DO_ON_SLEEP
				// 初回起動時の処理
				if (sAppData.u8Mode == E_IO_MODE_CHILD_SLP_1SEC || sAppData.u8Mode == E_IO_MODE_CHILD_SLP_10SEC) {
					vSleep(100, FALSE, FALSE); // 初回起動時は 100ms 寝る
				}
#endif
			}
		}

		if (eEvent == E_EVENT_TICK_TIMER) { // 何故 TickTiemr を待っていたのか不明だが、このままとする。
			sAppData.u8AdcState = 0; // ADC の開始
			sAppData.u32AdcLastTick = u32TickCount_ms;

			ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
		}
		break;

	case E_STATE_RUNNING:
		// メッセージポンプ
		if (eEvent == E_EVENT_APP_TICK_A // 秒64回のタイマー割り込み
		//&& (sAppData.u32CtTimer0 & 1) // 秒32回にする
				) {
			if (sAppData.u8measure > 0 && --sAppData.u8measure > 0) {
				// タイマーが有効だがまだタイムアウトしていない場合
				break;
			}

			switch (sAppData.u8ms_state) {
				case E_MS_IDLE:
					_C {
#if defined(I2C_SMBUS) || defined(I2C_PORT)
#if defined(I2C_SMBUS)
						unsigned char crem[] = { SHT31_I2C_CMD_MS_LO };
						bool_t bOK = bSMBusWrite(SHT31_I2C_ADDR, SHT31_I2C_CMD_MS, sizeof(crem), crem);
#endif
#if defined(I2C_PORT)
						uint8 crem[] = { SHT31_I2C_CMD_MS, SHT31_I2C_CMD_MS_LO};
						vI2C_BusReset();
						bool_t bOK = bI2C_WriteData(SHT31_I2C_ADDR, sizeof(crem), crem);
#endif
						sAppData.u8measure = 1;	// 6msec 待機する(仕様上は 4ms 以上待てばいいはず)
						if (bOK) {
							sAppData.u8ms_state = E_MS_EXEC;
						} else {
							// 前の値を繰り返し出力する
							sAppData.u8ms_state = E_MS_DONE;
						}
#else
						sAppData.u8ms_state = E_MS_EXEC;
#endif
					}
					break;

				case E_MS_EXEC:
					_C {
#if defined(I2C_SMBUS) || defined(I2C_PORT)
						bool_t bOK;
						unsigned char q[6];
#if defined(I2C_SMBUS)
						unsigned char crem[] = { SHT31_I2C_CMD_ST_1 };
						// 温度・湿度を読みだす
						bOK = bSMBusSequentialRead(SHT31_I2C_ADDR, 6, q);
#endif
#if defined(I2C_PORT)
						unsigned char crem[] = { SHT31_I2C_CMD_ST, SHT31_I2C_CMD_ST_1 };
						// 温度・湿度を読みだす
						bOK = bI2C_ReadData(SHT31_I2C_ADDR, 6, q);
#endif
						if (bOK) {
							sAppData.u16temperature = ((unsigned short)q[0] << 8) | q[1];
							sAppData.u16humidity = ((unsigned short)q[3] << 8) | q[4];
#if defined(I2C_SMBUS)
							bOK = bSMBusWrite(SHT31_I2C_ADDR, SHT31_I2C_CMD_ST, sizeof(crem), crem);
#endif
#if defined(I2C_PORT)
							bOK = bI2C_WriteData(SHT31_I2C_ADDR, sizeof(crem), crem);
#endif
							if (bOK) {
								sAppData.u8ms_state = E_MS_STATUS;
							} else {
								sAppData.u8ms_state = E_MS_DONE;
							}
						} else {
							// 読み出しが長時間不可能だった場合はソフトウェアリセットが必要
#if defined(I2C_SMBUS)
							uint8 rst[] = { SHT31_I2C_CMD_RST_1 };
							bSMBusWrite(SHT31_I2C_ADDR, SHT31_I2C_CMD_RST, sizeof(rst), rst);
#endif
							uint8 rst[] = { SHT31_I2C_CMD_RST, SHT31_I2C_CMD_RST_1 };
							vI2C_BusReset();
							bI2C_WriteData(SHT31_I2C_ADDR, sizeof(rst), rst);
							sAppData.u8ms_state = E_MS_IDLE;
						}
#else
						sAppData.u16temperature = 873;
						sAppData.u16humidity = 0x8000;
						sAppData.u8ms_state = E_MS_STATUS;
#endif
					}
					break;

				case E_MS_STATUS:
					_C {
#if defined(I2C_SMBUS) || defined(I2C_PORT)
						unsigned char q[3];
#if defined(I2C_SMBUS)
						bool_t bOK = bSMBusSequentialRead(SHT31_I2C_ADDR, 3, q);
#endif
#if defined(I2C_PORT)
						bool_t bOK = bI2C_ReadData(SHT31_I2C_ADDR, 3, q);
#endif
						if (bOK) {
							unsigned short u16 = ((unsigned short)q[0] << 8) | q[1];
							if (u16 & 0x8c10) {
								// エラーステータスが上がっていた場合はクリアする
#if defined(I2C_SMBUS)
								unsigned char crem[] = { SHT31_I2C_CMD_STRS_1 };
								bOK = bSMBusWrite(SHT31_I2C_ADDR, SHT31_I2C_CMD_STRS, sizeof(crem), crem);
#endif
#if defined(I2C_PORT)
								unsigned char crem[] = { SHT31_I2C_CMD_STRS, SHT31_I2C_CMD_STRS_1 };
								bOK = bI2C_WriteData(SHT31_I2C_ADDR, sizeof(crem), crem);
#endif
							}
							sAppData.u16status = u16;
						}
						// ステータスが読めた、読めないに拘らず終了ステータスとする
						sAppData.u8ms_state = E_MS_DONE;
#else
						sAppData.u16status = 0;
						sAppData.u8ms_state = E_MS_DONE;
#endif
					}
					break;

				case E_MS_DONE:
					break;
			}
		}
		if (sAppData.u8ms_state == E_MS_DONE) {
			// コマンド送信
			i16TransmitSerData();

			ToCoNet_Event_SetState(pEv, E_STATE_WAIT_TX);
		}
#if 0
	// ボタン状態や ADC は見ない
		DBGOUT(3, "%d", sAppData.u8IOFixState);

		// IO状態が確定すれば送信する。
		if (sAppData.u8IOFixState == 0x3) {
			vfPrintf(&sSerStream,
					"!INF DI1-4:%d%d%d%d A1-4:%04d/%04d/%04d/%04d"LB,
					sAppData.sIOData_now.au8Input[0] & 1,
					sAppData.sIOData_now.au8Input[1] & 1,
					sAppData.sIOData_now.au8Input[2] & 1,
					sAppData.sIOData_now.au8Input[3] & 1,
					sAppData.sIOData_now.au16InputADC[0] == 0xFFFF ?
							9999 : sAppData.sIOData_now.au16InputADC[0],
					sAppData.sIOData_now.au16InputADC[1] == 0xFFFF ?
							9999 : sAppData.sIOData_now.au16InputADC[1],
					sAppData.sIOData_now.au16InputADC[2] == 0xFFFF ?
							9999 : sAppData.sIOData_now.au16InputADC[2],
					sAppData.sIOData_now.au16InputADC[3] == 0xFFFF ?
							9999 : sAppData.sIOData_now.au16InputADC[3]);

			// クイックで送信
			sAppData.sIOData_now.i16TxCbId = i16TransmitIoData(TX_NODELAY, FALSE); // QUICK BIT は設定しない

			// 完了待ちをするため CbId を保存する。
			// TODO: この時点で失敗した場合は、次の状態のタイムアウトで処理されるが非効率である。

			ToCoNet_Event_SetState(pEv, E_STATE_WAIT_TX);
		}
#endif
		break;

	case E_STATE_WAIT_TX:
		if (eEvent == E_EVENT_APP_TX_COMPLETE) {
			ToCoNet_Event_SetState(pEv, E_STATE_FINISHED);
		}
#ifdef USE_SLOW_TX
		if (PRSEV_u32TickFrNewState(pEv) > 200) {
			ToCoNet_Event_SetState(pEv, E_STATE_FINISHED);
		}
#else
		if (PRSEV_u32TickFrNewState(pEv) > 100) {
			ToCoNet_Event_SetState(pEv, E_STATE_FINISHED);
		}
#endif
		break;

	case E_STATE_FINISHED:
		_C {
			static uint8 u8GoSleep = 0;
			if (eEvent == E_EVENT_NEW_STATE) {
				u8GoSleep = sAppData.bWakeupByButton ? 0 : 1;

				vfPrintf(&sSerStream, "!INF SLEEP %dms."LB,
						sAppData.u32SleepDur);
				SERIAL_vFlush(sSerStream.u8Device);
			}

			// ボタンでウェイクアップしたときはチャタリングが落ち着くのを待つのにしばらく停滞する
			if (PRSEV_u32TickFrNewState(pEv) > 20) {
				u8GoSleep = 1;
			}
			if (u8GoSleep == 1) {
				ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEPING);
			}
		}
		break;

	case E_STATE_APP_SLEEPING:
		if (eEvent == E_EVENT_NEW_STATE) {
#ifdef SET_DO_ON_SLEEP
			vPortSetHi(PORT_OUT1);
			vPortSetHi(PORT_OUT2);
#endif
			// Deep Sleep する; タイムアウト後、SW Reset する
			DBGOUT(3, "now sleeping"LB);
			vSleep(sAppData.u32SleepDur, FALSE, TRUE);
			//vSleep(sAppData.u32SleepDur, TRUE, FALSE);
		}

		break;

	default:
		break;
	}
}

/**  @ingroup MASTER
 * アプリケーション制御（スリープ稼動で親機からの受信を得る）\n
 *
 * @param pEv
 * @param eEvent
 * @param u32evarg
 */
void vProcessEvCoreSlpRecv(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
	switch (pEv->eState) {
	case E_STATE_IDLE:
		if (eEvent == E_EVENT_START_UP) {
			// vfPrintf(&sSerStream, "START_UP"LB, eEvent);
			if (u32evarg & EVARG_START_UP_WAKEUP_MASK) {
				// スリープからの復帰時の場合
				vfPrintf(&sSerStream, "!INF %s WAKE UP."LB,
						sAppData.bWakeupByButton ? "DI" : "TIMER");
			} else {
				//ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEPING);
				// break;
			}
		}

		sAppData.u8AdcState = 0; // ADC の開始
		sAppData.u32AdcLastTick = u32TickCount_ms;

		ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
		break;

	case E_STATE_RUNNING:
		DBGOUT(3, "%d", sAppData.u8IOFixState);

		// IO状態が確定すれば送信する。
		if (sAppData.u8IOFixState == 0x3) {
			vfPrintf(&sSerStream,
					"!INF DI1-4:%d%d%d%d A1-4:%04d/%04d/%04d/%04d"LB,
					sAppData.sIOData_now.au8Input[0] & 1,
					sAppData.sIOData_now.au8Input[1] & 1,
					sAppData.sIOData_now.au8Input[2] & 1,
					sAppData.sIOData_now.au8Input[3] & 1,
					sAppData.sIOData_now.au16InputADC[0] == 0xFFFF ?
							9999 : sAppData.sIOData_now.au16InputADC[0],
					sAppData.sIOData_now.au16InputADC[1] == 0xFFFF ?
							9999 : sAppData.sIOData_now.au16InputADC[1],
					sAppData.sIOData_now.au16InputADC[2] == 0xFFFF ?
							9999 : sAppData.sIOData_now.au16InputADC[2],
					sAppData.sIOData_now.au16InputADC[3] == 0xFFFF ?
							9999 : sAppData.sIOData_now.au16InputADC[3]);

			// クイックで送信
			sAppData.sIOData_now.i16TxCbId = i16TransmitIoData(TX_NODELAY_AND_RESP_BIT, FALSE); // QUICK BIT は設定しない

			ToCoNet_Event_SetState(pEv, E_STATE_WAIT_COMMAND);
		}
		break;

	case E_STATE_WAIT_COMMAND:
		if(eEvent == E_EVENT_NEW_STATE) {
			sAppData.sIOData_now.u32RxLastTick = u32TickCount_ms + 1000;
			sToCoNet_AppContext.bRxOnIdle = TRUE;
			ToCoNet_vRfConfig();
		}

		if (u32TickCount_ms - sAppData.sIOData_now.u32RxLastTick < 100) {
			// パケットが受信されたら u32RxLastTick が更新される
			ToCoNet_Event_SetState(pEv, E_STATE_FINISHED);
		} else
		if (PRSEV_u32TickFrNewState(pEv) > 100) {
			ToCoNet_Event_SetState(pEv, E_STATE_FINISHED);
		}
		break;

	case E_STATE_FINISHED:
		if (eEvent == E_EVENT_NEW_STATE) {
			// RF の停止
			sToCoNet_AppContext.bRxOnIdle = FALSE;
			ToCoNet_vRfConfig();

			vfPrintf(&sSerStream, "!INF SLEEP %dms."LB,
					sAppData.u32SleepDur);

			SERIAL_vFlush(sSerStream.u8Device);

			// スリープする
			ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEPING);
		}
		break;

	case E_STATE_APP_SLEEPING:
		if (eEvent == E_EVENT_NEW_STATE) {
			vSleep(sAppData.u32SleepDur, TRUE, FALSE);
		}
		break;

	default:
		break;
	}
}

/** @ingroup MASTER
 * 電源投入時・リセット時に最初に実行される処理。本関数は２回呼び出される。初回は u32AHI_Init()前、
 * ２回目は AHI 初期化後である。
 *
 * - 各種初期化
 * - ToCoNet ネットワーク設定
 * - 設定IO読み取り
 * - 緊急フラッシュ消去処理
 * - 設定値の計算
 * - ハードウェア初期化
 * - イベントマシンの登録
 * - 本関数終了後は登録したイベントマシン、および cbToCoNet_vMain() など各種コールバック関数が
 *   呼び出される。
 *
 * @param bStart TRUE:u32AHI_Init() 前の呼び出し FALSE: 後
 */
void cbAppColdStart(bool_t bStart) {
	if (!bStart) {
		// before AHI initialization (very first of code)

		// Module Registration
		ToCoNet_REG_MOD_ALL();
	} else {
		// メモリのクリア
		memset(&sAppData, 0x00, sizeof(sAppData));
		memset(&sAppData.sIOData_now, 0xFF, sizeof(tsIOData));
		memset(&sAppData.sIOData_reserve, 0xFF, sizeof(tsIOData));
		vConfig_UnSetAll(&sAppData.sConfig_UnSaved);

#ifdef USE_I2C_PORT_AS_PWM_SPECIAL
		memset(&sPwmSpe_Context, 0, sizeof(sPwmSpe_Context));
#endif
		sAppData.u8measure = 5;	// 初回起動で >=125ms 待つ
		//sAppData.u8DebugLevel = 8;	// 最大デバッグ出力
		
		// デフォルトのネットワーク指定値
		sToCoNet_AppContext.u8TxMacRetry = 3; // MAC再送回数
		sToCoNet_AppContext.u32AppId = APP_ID; // アプリケーションID
		sToCoNet_AppContext.u32ChMask = CHMASK; // 利用するチャネル群（最大３つまで）
		sToCoNet_AppContext.u8Channel = CHANNEL; // デフォルトのチャネル

		// フラッシュの読み出し
		sAppData.bFlashLoaded = bFlash_Read(&sAppData.sFlash,
				FLASH_SECTOR_NUMBER - 1, 0);

		// Version String のチェック
		if (sAppData.bFlashLoaded
				&& ((APP_ID != sAppData.sFlash.sData.u32appkey)
						|| (VERSION_U32 != sAppData.sFlash.sData.u32ver))) {
			sAppData.bFlashLoaded = FALSE;
		}

		// フラッシュ設定値の反映
		if (sAppData.bFlashLoaded) {
			sToCoNet_AppContext.u32AppId = sAppData.sFlash.sData.u32appid;
			// sToCoNet_AppContext.u8Channel = sAppData.sFlash.sData.u8ch; // チャネルマネージャで決定するので設定不要
			sToCoNet_AppContext.u32ChMask = sAppData.sFlash.sData.u32chmask;

			if (sAppData.sFlash.sData.u8role == E_APPCONF_ROLE_MAC_NODE) {
				sAppData.eNwkMode = E_NWKMODE_MAC_DIRECT;
			} else if (sAppData.sFlash.sData.u8role == E_APPCONF_ROLE_SILENT) {
				sAppData.eNwkMode = E_NWKMODE_MAC_DIRECT;
			} else {
				sAppData.bFlashLoaded = 0;
			}

			if (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_LOW_LATENCY_INPUT) {
				sToCoNet_AppContext.u16TickHz = 1000; // 低レイテンシモードでは 1KHz 動作
			}
		}

		// フラッシュのロードに失敗したとき
		if (sAppData.bFlashLoaded != TRUE) {
			// 構造体にデフォルト値を格納する
			vConfig_SetDefaults(&(sAppData.sFlash.sData));
		}

		// 出力の設定
		sToCoNet_AppContext.u8TxPower = sAppData.sFlash.sData.u8pow & 0x0F;

		// 標準再送回数の計算
		uint8 u8retry = (sAppData.sFlash.sData.u8pow & 0xF0) >> 4;
		switch (u8retry) {
			case   0: sAppData.u8StandardTxRetry = 0x82; break;
			case 0xF: sAppData.u8StandardTxRetry = 0; break;
			default:  sAppData.u8StandardTxRetry = 0x80 + u8retry; break;
		}

		// ヘッダの１バイト識別子を AppID から計算
		sAppData.u8AppIdentifier = u8CCITT8(
				(uint8*) &sToCoNet_AppContext.u32AppId, 4); // APP ID の CRC8

		// IOより状態を読み取る (ID など)
		sAppData.u32DIO_startup = ~u32PortReadBitmap(); // この時点では全部入力ポート

		// 緊急のフラッシュ消去モード
		if ((0 == (sAppData.u32DIO_startup & (1UL << PORT_CONF1)))
				&& (sAppData.u32DIO_startup & (1UL << PORT_CONF2))
				&& (sAppData.u32DIO_startup & (1UL << PORT_CONF3))
				&& (sAppData.u32DIO_startup & (1UL << 15))
				&& (sAppData.u32DIO_startup & (1UL << PORT_INPUT4))) {
			//中継機設定で、I2C ポートが Lo で起動する
			uint32 u32ct = 0;

			vPortAsOutput(PORT_OUT1);

			for (;;) {
				sAppData.u32DIO_startup = ~u32PortReadBitmap();

				if ((sAppData.u32DIO_startup & (1UL << 15))
						&& (sAppData.u32DIO_startup & (1UL << PORT_INPUT4))) {
					u32ct++;

					vPortSet_TrueAsLo(PORT_OUT1, u32ct & 0x8000);

					if (u32ct > 800000) { // some seconds
						bFlash_Erase(FLASH_SECTOR_NUMBER - 1); // SECTOR ERASE

						vPortSetHi(PORT_OUT1);

						while (1) {
							u32ct++;
							vPortSet_TrueAsLo(PORT_OUT1, u32ct & 0x80000);
						}
					}
				} else {
					// may launch as normal mode
					break;
				}
			}
		}

		// version info
		sAppData.u32ToCoNetVersion = ToCoNet_u32GetVersion();

		// ToCoNet の制御 Tick [ms]
		sAppData.u16ToCoNetTickDelta_ms = 1000 / sToCoNet_AppContext.u16TickHz;

		// その他ハードウェアの初期化
		vInitHardware(FALSE);

#ifdef USE_MONOSTICK
		vPortSetLo(WD_ENABLE);		// 外部のウォッチドッグを有効にする。

		vPortAsOutput(WD_ENABLE);	// パルス出力ピンを出力に設定する

		// ToCoStick の場合はデフォルトで親機に設定する
		bColdStart = TRUE;
		vPortSetLo(PORT_OUT1);
		sTimerPWM[2].u16duty = _PWM(0);
		vTimerStart(&sTimerPWM[2]);

		if (!sAppData.bFlashLoaded) {
			sAppData.u8Mode = E_IO_MODE_PARNET; // 親機のIO設定に強制する
		} else {
			sAppData.u8Mode = 0;
		}
#endif

		// 論理IDの設定チェック、その他設定値のチェック
		//  IO の設定を優先し、フラッシュ設定で矛盾するものについてはデフォルト値を書き直す。
		if (IS_LOGICAL_ID_CHILD(au8IoModeTbl_To_LogicalID[sAppData.u8Mode])) {
			// 子機IDはフラッシュ値が設定されていれば、これを採用
			if (sAppData.bFlashLoaded) {
				sAppData.u8AppLogicalId = sAppData.sFlash.sData.u8id;
			}

			if (!IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId )) {
				sAppData.u8AppLogicalId =
						au8IoModeTbl_To_LogicalID[sAppData.u8Mode];
			}
		}

		// 論理IDを121,122に保存した場合、親機で起動する
		if (sAppData.bFlashLoaded) {
			if (sAppData.sFlash.sData.u8id == 121) {
				sAppData.u8Mode = 1; // 親機のモード番号
				sAppData.u8AppLogicalId =
						au8IoModeTbl_To_LogicalID[sAppData.u8Mode]; // 論理IDを設定
			} else if (sAppData.sFlash.sData.u8id == 122) {
				sAppData.u8Mode = 2; // 親機のモード番号
				sAppData.u8AppLogicalId =
						au8IoModeTbl_To_LogicalID[sAppData.u8Mode]; // 論理IDを設定
			}
		}

		// 各モード依存の初期値の設定など
		switch (sAppData.u8Mode) {
		case E_IO_MODE_PARNET:
			sAppData.u8AppLogicalId = LOGICAL_ID_PARENT;
			break;

		case E_IO_MODE_ROUTER:
			sAppData.u8AppLogicalId = LOGICAL_ID_REPEATER;
			break;

#if 0
		case E_IO_MODE_CHILD_SLP_RECV:
		case E_IO_MODE_CHILD_SLP_1SEC:
			if (!sAppData.u32SleepDur) {
				if (sAppData.bFlashLoaded) {
					sAppData.u32SleepDur = sAppData.sFlash.sData.u16SleepDur_ms;
				} else {
					sAppData.u32SleepDur = MODE4_SLEEP_DUR_ms;
				}
			}
			break;
#endif

		case E_IO_MODE_CHILD_SLP_10SEC:
			if (!sAppData.u32SleepDur) {
				if (sAppData.bFlashLoaded) {
					sAppData.u32SleepDur = sAppData.sFlash.sData.u16SleepDur_s
							* 1000L;
				} else {
					sAppData.u32SleepDur = MODE7_SLEEP_DUR_ms;
				}
			}
			break;

		case E_IO_MODE_CHILD_CONT_TX:
			sAppData.u8FpsBitMask = 1;
			if (sAppData.bFlashLoaded) {
				// 4fps: 1111
				// 8fps:  111 (64/8 -1)
				// 16pfs:  11 (64/16-1)
				// 32fps:   1 (64/32-1)
				sAppData.u8FpsBitMask = 64 / sAppData.sFlash.sData.u8Fps - 1;
				// DBGOUT(0, "fps mask = %x"LB, sAppData.u8FpsBitMask);
			}
			break;

		case E_IO_MODE_CHILD:
			break;

		default: // 未定義機能なので、SILENT モードにする。
			sAppData.u8AppLogicalId = 255;
			sAppData.sFlash.sData.u8role = E_APPCONF_ROLE_SILENT;
			break;
		}

		// その他設定
		if (IS_APPCONF_OPT_ROUTING_HOP2()) {
			sAppData.u8max_hops = 2;
		} else if (IS_APPCONF_OPT_ROUTING_HOP3()) {
			sAppData.u8max_hops = 3;
		} else {
			sAppData.u8max_hops = 1;
		}

		// ショートアドレスの設定(決めうち)
		sToCoNet_AppContext.u16ShortAddress =
				SERCMD_ADDR_CONV_TO_SHORT_ADDR(sAppData.u8AppLogicalId);

		// UART の初期化
		ToCoNet_vDebugInit(&sSerStream);
		ToCoNet_vDebugLevel(0);

		// その他の初期化
		DUPCHK_vInit(&sDupChk_IoData); // 重複チェック用
		DUPCHK_vInit(&sDupChk_SerMsg); // 重複チェック用

		if (!(IS_APPCONF_ROLE_SILENT_MODE())) {

			// 状態遷移マシンの登録
			switch (sAppData.u8Mode) {
			case E_IO_MODE_PARNET:
			case E_IO_MODE_ROUTER:
			case E_IO_MODE_CHILD:
			case E_IO_MODE_CHILD_CONT_TX:
				ToCoNet_Event_Register_State_Machine(vProcessEvCorePwr); // 常時通電用の処理
				sAppData.prPrsEv = (void*) vProcessEvCorePwr;
				sToCoNet_AppContext.bRxOnIdle = TRUE;
				break;
			case E_IO_MODE_CHILD_SLP_1SEC:
			case E_IO_MODE_CHILD_SLP_10SEC:
				ToCoNet_Event_Register_State_Machine(vProcessEvCoreSlp); // スリープ用の処理
				sAppData.prPrsEv = (void*) vProcessEvCoreSlp;
				sToCoNet_AppContext.bRxOnIdle = FALSE;
				break;

			case E_IO_MODE_CHILD_SLP_RECV:
				ToCoNet_Event_Register_State_Machine(vProcessEvCoreSlpRecv); // スリープ兼受信
				sAppData.prPrsEv = (void*) vProcessEvCoreSlpRecv;
				sToCoNet_AppContext.u16TickHz = 1000; // 自動的に低レイテンシモードとする
				sToCoNet_AppContext.bRxOnIdle = FALSE; // 受信処理を行う
				break;
			default: // 未定義機能なので、SILENT モードにする。
				sToCoNet_AppContext.bRxOnIdle = FALSE;
				break;
			}

			// MAC の初期化
			ToCoNet_vMacStart();

			// 主状態遷移マシンの登録
			sAppData.u8Hnd_vProcessEvCore = ToCoNet_Event_Register_State_Machine(vProcessEvCore);
		}
	}
}

/** @ingroup MASTER
 * スリープ復帰後に呼び出される関数。\n
 * 本関数も cbAppColdStart() と同様に２回呼び出され、u32AHI_Init() 前の
 * 初回呼び出しに於いて、スリープ復帰要因を判定している。u32AHI_Init() 関数は
 * これらのレジスタを初期化してしまう。
 *
 * - 変数の初期化（必要なもののみ）
 * - ハードウェアの初期化（スリープ後は基本的に再初期化が必要）
 * - イベントマシンは登録済み。
 *
 * @param bStart TRUE:u32AHI_Init() 前の呼び出し FALSE: 後
 */
void cbAppWarmStart(bool_t bStart) {
	int i;

	if (!bStart) {
		// before AHI init, very first of code.
		//  to check interrupt source, etc.

		sAppData.bWakeupByButton = FALSE;
		if (u8AHI_WakeTimerFiredStatus()) {
			;
		} else if (u32AHI_DioWakeStatus() & u32PortInputMask) {
			// woke up from DIO events
			sAppData.bWakeupByButton = TRUE;
		}

	} else {
#ifdef SET_DO_ON_SLEEP
		if (sAppData.u8Mode == E_IO_MODE_CHILD_SLP_1SEC || sAppData.u8Mode == E_IO_MODE_CHILD_SLP_10SEC) {
			if (u8SleepControl == 0) {
				u8SleepControl = 1;
				vPortAsOutput(PORT_OUT1);
				vPortAsOutput(PORT_OUT2);
				vPortSetLo(PORT_OUT1);
				vPortSetLo(PORT_OUT2);
				ToCoNet_vSleep(E_AHI_WAKE_TIMER_1, PRE_SLEEP_ms, FALSE, FALSE); // PERIODIC RAM OFF SLEEP USING WK0
				while(1);
			} else {
				u8SleepControl = 0;
			}
		}
#endif
		// データ領域の初期化
		memset(&sAppData.sIOData_now, 0xFF, sizeof(tsIOData));

		// いくつかのデータは復元
		sAppData.sIOData_now.u32BtmUsed = sAppData.sIOData_reserve.u32BtmUsed;
		sAppData.sIOData_now.u32BtmChanged = sAppData.sIOData_reserve.u32BtmChanged;
		sAppData.sIOData_now.u32BtmBitmap = sAppData.sIOData_reserve.u32BtmBitmap;
		for (i = 0; i < 4; i++) {
			sAppData.sIOData_now.au8Output[i] = sAppData.sIOData_reserve.au8Output[i];
		}

		memcpy(sAppData.sIOData_now.au16InputADC_LastTx,
				sAppData.sIOData_reserve.au16InputADC_LastTx,
				sizeof(sAppData.sIOData_now.au16InputADC_LastTx));

		// 変数の初期化（必要なものだけ）
		sAppData.u16CtTimer0 = 0; // このカウンタは、起動時からのカウントとする
		sAppData.u8IOFixState = FALSE; // IO読み取り状態の確定待ちフラグ
		sAppData.bUpdatedAdc = 0; // ADCの変化フラグ

		// その他ハードウェアの初期化（基本、起動時に毎回実行する）
		vInitHardware(TRUE);

		// UART の初期化
		ToCoNet_vDebugInit(&sSerStream);
		ToCoNet_vDebugLevel(0);

		// その他の初期化
		DUPCHK_vInit(&sDupChk_IoData);
		DUPCHK_vInit(&sDupChk_SerMsg);

		if (sAppData.u8Mode == E_IO_MODE_CHILD_SLP_RECV) {
			sToCoNet_AppContext.bRxOnIdle = FALSE;
		}

		// MAC の開始
		ToCoNet_vMacStart();
	}
}

/** @ingroup MASTER
 * 本関数は ToCoNet のメインループ内で必ず１回は呼び出される。
 * ToCoNet のメインループでは、CPU DOZE 命令を発行しているため、割り込みなどが発生した時に
 * 呼び出されるが、処理が無い時には呼び出されない。
 * しかし TICK TIMER の割り込みは定期的に発生しているため、定期処理としても使用可能である。
 *
 * - シリアルの入力チェック
 */
void cbToCoNet_vMain(void) {
	vHandleSerialInput(); // シリアルポートの処理
}

/** @ingroup MASTER
 * パケットの受信完了時に呼び出されるコールバック関数。\n
 * パケットの種別によって具体的な処理関数にディスパッチしている。
 * データ種別は psRx->u8Cmd (ToCoNet のパケットヘッダに含まれます) により識別される。
 *
 * - パケット種別
 *   - TOCONET_PACKET_CMD_APP_DATA : シリアル電文パケット
 *   - TOCONET_PACKET_CMD_APP_USER_IO_DATA : IO状態の伝送
 *   - TOCONET_PACKET_CMD_APP_USER_IO_DATA_EXT : シリアル電文による IO 状態の伝送
 *
 * @param psRx 受信パケット
 */
void cbToCoNet_vRxEvent(tsRxDataApp *psRx) {
	//uint8 *p = pRx->auData;

	DBGOUT(3, "Rx packet (cm:%02x, fr:%08x, to:%08x)"LB, psRx->u8Cmd,
			psRx->u32SrcAddr, psRx->u32DstAddr);

	if (IS_APPCONF_ROLE_SILENT_MODE()
			|| sAppData.u8Mode == E_IO_MODE_CHILD_SLP_1SEC
			|| sAppData.u8Mode == E_IO_MODE_CHILD_SLP_10SEC) {
		// SILENT, 1秒スリープ, 10秒スリープでは受信処理はしない。
		return;
	}

	switch (psRx->u8Cmd) {
	case TOCONET_PACKET_CMD_APP_DATA: // シリアルメッセージのパケット
		vReceiveSerMsg(psRx);
		break;
	case TOCONET_PACKET_CMD_APP_USER_IO_DATA: // IO状態の伝送
		if (PRSEV_eGetStateH(sAppData.u8Hnd_vProcessEvCore) == E_STATE_RUNNING) { // 稼動状態でパケット処理をする
#ifdef USE_MONOSTICK
			//bColdStart = FALSE;
#endif
			vReceiveIoData(psRx);
		}
		break;
	case TOCONET_PACKET_CMD_APP_USER_IO_DATA_EXT: // IO状態の伝送(UART経由)
#ifdef USE_MONOSTICK
			//bColdStart = FALSE;
#endif
		if (PRSEV_eGetStateH(sAppData.u8Hnd_vProcessEvCore) == E_STATE_RUNNING) { // 稼動状態でパケット処理をする
			vReceiveIoSettingRequest(psRx);
		}
		break;
	}
}

/** @ingroup MASTER
 * 送信完了時に呼び出されるコールバック関数。
 *
 * - IO 送信完了イベントはイベントマシンにE_EVENT_APP_TX_COMPLETEイベントを伝達する。
 * - シリアルメッセージの一連のパケット群の送信完了も検出しsSerSeqTx.bWaitComplete
 *   フラグをリセットする。
 *
 * @param u8CbId 送信時に設定したコールバックID
 * @param bStatus 送信ステータス
 */
void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus) {
	//uint8 *q = au8SerOutBuff;
	if (IS_APPCONF_ROLE_SILENT_MODE()) {
		return;
	}

	// IO 関連の送信が完了した
	if (sAppData.sIOData_now.i16TxCbId >= 0
			&& u8CbId == sAppData.sIOData_now.i16TxCbId) {
		// スリープを行う場合は、このイベントを持ってスリープ遷移
		ToCoNet_Event_Process(E_EVENT_APP_TX_COMPLETE, u8CbId,
				sAppData.prPrsEv);
	}

	// UART 送信の完了チェック
	if (sSerSeqTx.bWaitComplete) {
		if (u8CbId >= sSerSeqTx.u8Seq
				&& u8CbId < sSerSeqTx.u8Seq + sSerSeqTx.u8PktNum) {
			uint8 idx = u8CbId - sSerSeqTx.u8Seq;
			if (bStatus) {
				sSerSeqTx.bPktStatus[idx] = 1;
			} else {
				if (sSerSeqTx.bPktStatus[idx] == 0) {
					sSerSeqTx.bPktStatus[idx] = -1;
				}
			}
		}

		int i, isum = 0;
		for (i = 0; i < sSerSeqTx.u8PktNum; i++) {
			if (sSerSeqTx.bPktStatus[i] == 0)
				break;
			isum += sSerSeqTx.bPktStatus[i];
		}

		if (i == sSerSeqTx.u8PktNum) {
			/* 送信完了 (MAC レベルで成功した) */
			sSerSeqTx.bWaitComplete = FALSE;

			// VERBOSE MESSAGE
			DBGOUT(3, "* >>> MacAck%s(tick=%d,req=#%d) <<<" LB,
					(isum == sSerSeqTx.u8PktNum) ? "" : "Fail",
					u32TickCount_ms & 65535, sSerSeqTx.u8ReqNum);
		}
	}

	return;
}

/** @ingroup MASTER
 * ネットワーク層などのイベントが通達される。\n
 * 本アプリケーションでは特別な処理は行っていない。
 *
 * @param ev
 * @param u32evarg
 */
void cbToCoNet_vNwkEvent(teEvent ev, uint32 u32evarg) {
	if (IS_APPCONF_ROLE_SILENT_MODE()) {
		return;
	}

	switch (ev) {
	case E_EVENT_TOCONET_NWK_START:
		break;

	case E_EVENT_TOCONET_NWK_DISCONNECT:
		break;

	default:
		break;
	}
}

/** @ingroup MASTER
 * ハードウェア割り込み時に呼び出される。本処理は割り込みハンドラではなく、割り込みハンドラに登録された遅延実行部による処理で、長い処理が記述可能である。
 * 本アプリケーションに於いては、ADC/DIの入力状態のチェック、64fps のタイマーイベントの処理などを行っている。
 *
 * - E_AHI_DEVICE_SYSCTRL
 *   - DI割り込みの処理を行う。これは、低レイテンシモードでの処理である。
 *
 * - E_AHI_DEVICE_TICK_TIMER : このイベントは ToCoNet 組み込みで、ToCoNet の制御周期 (sToCoNet_AppContext.u16TickHz) を
 *   実現するためのタイマーです。ユーザが TickTimer の制御を変更したりすると ToCoNet は動作しなくなります。
 *
 *   - Di入力の変化の確認。変化が有れば、sAppData.sIOData_now 構造体に結果を格納する。
 *     低レイテンシモードの時は、この判定を起点として送信を行う。
 *
 * - E_AHI_DEVICE_TIMER0 : TICK_TIMER から分周して制御周期を作っても良いのですが、TIMER_0 を使用しています。
 *   - カウンタのインクリメント処理
 *   - ADC の完了確認
 *   - パケット重複チェックアルゴリズムのタイムアウト処理
 *   - DIのカウンタ処理 (インタラクティブモードでカウンタ終了時にもパケットを送信する処理を行う）
 *   - イベントマシンに TIMER0 イベントを発行
 *   - インタラクティブモード時の画面再描画
 *
 * @param u32DeviceId
 * @param u32ItemBitmap
 */
void cbToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap) {
	switch (u32DeviceId) {
	case E_AHI_DEVICE_SYSCTRL:
		if (sAppData.u8IOFixState != 0xFF) {
			int i;
			bool_t bTransmit = FALSE;

			/* DIの入力ピンの番号を調べる（低レイテンシモード）
			 *
			 *  ボタンを猿みたいに押してみたが DIO の割り込みは同時に２ビット報告されることは無く、
			 *  順 次処理されるようだ。しかしながら、同時処理されても良いようなコードにしておく。
			 */
			DBGOUT(1, LB"BTN: ");
			for (i = 0; i < 4; i++) {
				/* DI検知したポートについては sAppData.sIOData_now.au8Input[] に値を設定する。
				 *
				 * この値の下位１ビットが、１になると Lo 判定したことを意味する。
				 * この値の上位４ビットは、再度の割り込みを検知しても無視するためのカウンタとして
				 * 用いる。このカウンタは E_AHI_DEVICE_TIMER0 イベントで処理する。
				 */
				DBGOUT(1, "%c",
						u32ItemBitmap & (1UL << au8PortTbl_DIn[i]) ? '1' : '0');

				if (u32ItemBitmap & (1UL << au8PortTbl_DIn[i])) { // 押し下げを検知したポート
					uint8 u8stat = sAppData.sIOData_now.au8Input[i]; // 元の値を取り出す。
					uint8 u8ct = (u8stat & 0xF0) >> 4; // 上位４ビットは、前回の同様の割り込み受信からの 64fps カウンタ

					// カウンタ値が無い場合は、個の割り込みを有効とする。
					if (u8ct == 0) {
						sAppData.sIOData_now.au8Input[i] =
								(LOW_LATENCY_DELAYED_TRANSMIT_COUNTER * 0x10)
										+ 1;
						bTransmit = TRUE;
					} else {
						// カウンタ値が有る場合は、直前に押されたためチャタリングなどが考えられ処理しない
						;
					}
				}
			}

			// いずれかのポートの割り込みが有効であった場合。
			if (bTransmit) {
				/* 速やかに送信する（低レイテンシモード）
				 *   ポートの変更対象フラグを、この割り込みで入力検知したものとする。
				 *   そうしないと、関係ないビットが変更されてしまう
				 */
				uint32 u32used = sAppData.sIOData_now.u32BtmUsed; // 関数呼び出し中だけ値を変更する
				sAppData.sIOData_now.u32BtmUsed = u32ItemBitmap
						& u32PortInputMask; // 割り込みでLoになったDINだけ変更対照として送信する
				sAppData.sIOData_now.i16TxCbId = i16TransmitIoData(TX_NODELAY_AND_QUICK_BIT, FALSE); // 送信処理を行う
				sAppData.sIOData_now.u32BtmUsed = u32used
						| (u32ItemBitmap & u32PortInputMask); //値を復元する
			}
		}
		break;

	case E_AHI_DEVICE_ANALOGUE: //ADC完了時にこのイベントが発生する。
		break;

	case E_AHI_DEVICE_TICK_TIMER: //比較的頻繁な処理
		// ボタンの判定を行う。
		_C {
			uint32 bmPorts, bmChanged, i;
			if (bBTM_GetState(&bmPorts, &bmChanged)) {
#ifdef USE_I2C_PORT_AS_OTHER_FUNCTION
				{
					static uint8 u8LastPort;
					static uint32 u32LastChange;

					uint8 u8PortRead = ((bmPorts & (1UL << 15)) ? 2 : 0)
							+ ((bmPorts & (1UL << 14)) ? 1 : 0);
					// uint8 u8PortChg = ((bmChanged & (1UL << 15)) ? 2 : 0) + ((bmChanged & (1UL << 14)) ? 1 : 0); // 動かん(何故？)

					if (u8PortRead != u8LastPort
							&& !(u32TickCount_ms - u32LastChange < 100)) {
# if defined(USE_I2C_PORT_AS_CHANNEL_SET)
						if (u8PortRead != u8ChPreSet) {
							vChangeChannelPreset(u8PortRead);
							u8ChPreSet = u8PortRead;
						}
# endif
# if defined(USE_I2C_PORT_AS_PWM_SPECIAL)
						u8PwmSpe_TxMode = u8PortRead;
# endif
						bmPorts &= ~((1UL << 15) | (1UL << 14));
						bmChanged &= ~((1UL << 15) | (1UL << 14));

						u32LastChange = u32TickCount_ms;
						u8LastPort = u8PortRead;

						DBGOUT(5, "Port14,15 = %d" LB, u8PortRead);
					}
				}
#endif

#ifdef USE_MONOSTICK
				static bool_t bPulse = FALSE;
				if( bWDTest){
					vPortSet_TrueAsLo(WD_PULSE,  bPulse );
					bPulse = !bPulse;
				}
				if( bColdStart && u32TickCount_ms >= LED_FLASH_MS ){
					bColdStart = FALSE;
					vPortSetHi(PORT_OUT1);
					sTimerPWM[2].u16duty = _PWM(1024);
					vTimerStart(&sTimerPWM[2]);
				}
#endif

				// 読み出し値を格納する
				for (i = 0; i < 4; i++) {
					uint8 u8stat =
							sAppData.sIOData_now.au8Input[i] == 0xFF ?
									0 : sAppData.sIOData_now.au8Input[i];
					// LSBを設定する
					if (bmPorts & (1UL << au8PortTbl_DIn[i])) {
						u8stat |= 0x01;
					} else {
						u8stat &= 0xFE;
					}
					sAppData.sIOData_now.au8Input[i] = u8stat;
				}
				sAppData.sIOData_now.u32BtmBitmap = bmPorts; // au8Input と冗長だが両方管理する。

				if (bmChanged) { // 入力ポートの値が確定したか、変化があった
					// 利用入力ポートの判定
					if (sAppData.sIOData_now.u32BtmUsed == 0xFFFFFFFF) {
						sAppData.sIOData_now.u32BtmUsed = bmPorts; // 一番最初の確定時に Lo のポートは利用ポート
					} else {
						sAppData.sIOData_now.u32BtmUsed |= bmPorts; // 利用ポートのビットマップは一度でもLo確定したポート
					}

					// 変化ポートの判定
					if (sAppData.u8IOFixState & 1) {
						// 初回確定後
						sAppData.sIOData_now.u32BtmChanged |= bmChanged; // 変化フラグはアプリケーションタスクに変更させる
					} else {
						// 初回確定時(スリープ復帰後も含む)
						sAppData.sIOData_now.u32BtmChanged = 0; // 初回は報告しない
					}

					// IO確定ビットを立てる
					sAppData.u8IOFixState |= 0x1;
				}
			}

			// 低レイテンシモードならここで送信を行う。
			if (sAppData.u8IOFixState && sAppData.sIOData_now.u32BtmChanged
					&& (sAppData.sFlash.sData.u32Opt
							& E_APPCONF_OPT_LOW_LATENCY_INPUT)) {
				sAppData.sIOData_now.i16TxCbId = i16TransmitIoData(TX_NODELAY_AND_QUICK_BIT, FALSE);
				sAppData.sIOData_now.u32BtmChanged = 0;
			}
		}
		break;

	case E_AHI_DEVICE_TIMER0:
		// タイマーカウンタをインクリメントする (64fps なので 64カウントごとに１秒)
		sAppData.u32CtTimer0++;
		sAppData.u16CtTimer0++;

		// ADC の完了確認
		if (sAppData.u8AdcState == 0x80) {
			sAppData.u8AdcState = 0; // ADC の開始
		}

		// 重複チェックのタイムアウト処理
		if ((sAppData.u32CtTimer0 & 0xF) == 0) {
			DUPCHK_bFind(&sDupChk_IoData, 0, NULL );
			DUPCHK_bFind(&sDupChk_SerMsg, 0, NULL );
		}

		// 送信処理のタイムアウト処理
		if (sSerSeqTx.bWaitComplete) {
			if (u32TickCount_ms - sSerSeqTx.u32Tick > 1000) {
				// タイムアウトとして、処理を続行
				memset(&sSerSeqTx, 0, sizeof(sSerSeqTx));
			}
		}

		/* インタラクティブモードのカウンタ処理。
		 * カウンタが0になったときに送信フラグを立てる。
		 * 1.3.4 カウンタが0までは押し下げフラグを維持
		 */
		{
			int i;

			// Input ビットのカウンタビットをカウントダウンする。
			bool_t bUpdated = FALSE;
			for (i = 0; i < 4; i++) {
				uint8 u8stat = sAppData.sIOData_now.au8Input[i];
				if (u8stat == 0xFF)
					continue;

				uint8 u8ct = u8stat >> 4;

				if (u8ct) {
					u8ct--;
					if (u8ct == 0) {
						// 送信要求
						bUpdated = TRUE;
					}
				}

				u8stat = (u8ct << 4) + (u8stat & 0x0F);
				sAppData.sIOData_now.au8Input[i] = u8stat;
			}
			if (bUpdated) {
				sAppData.sIOData_now.u32BtmChanged |= 0x80000000;
			}
		}

		// イベント処理部分にイベントを送信
		if (sAppData.prPrsEv && (sAppData.u32CtTimer0 & 1)) {
			ToCoNet_Event_Process(E_EVENT_APP_TICK_A, 0, sAppData.prPrsEv);
		}

		// シリアル画面制御のためのカウンタ
		if (!(--u16HoldUpdateScreen)) {
			if (sSerCmd.bverbose) {
				vSerUpdateScreen();
			}
		}
		break;

	default:
		break;
	}
}

/** @ingroup MASTER
 * 割り込みハンドラ。ここでは長い処理は記述してはいけない。
 *
 * - TICK_TIMER\n
 *   - ADCの実行管理
 *   - ボタン入力判定管理
 */
PUBLIC uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap) {
	uint8 u8handled = FALSE;

	switch (u32DeviceId) {
	case E_AHI_DEVICE_TIMER0:
#ifdef USE_I2C_PORT_AS_PWM_SPECIAL
		switch (u8PwmSpe_RxMode) {
		case 1:
			_C {
				int i;
				for (i = 0; i < 4; i++) {
					if (sAppData.sIOData_now.au16OutputPWMDuty[i] != 0xFFFF) {
						if (sPwmSpe_Context.u16Ct_Total[i] == 0) {
							sTimerPWM[i].u16duty = _PWM(1024); // 消灯
						} else if (sPwmSpe_Context.u16Ct_Now[i]
								> sPwmSpe_Context.u16Ct_Active[i]) {
							sTimerPWM[i].u16duty = _PWM(0);
						} else {
							sTimerPWM[i].u16duty = _PWM(1024);
						}
						vTimerStart(&sTimerPWM[i]); // DUTY比だけ変更する
					}

					sPwmSpe_Context.u16Ct_Now[i]++;
					if (sPwmSpe_Context.u16Ct_Now[i]
							> sPwmSpe_Context.u16Ct_Total[i]) {
						sPwmSpe_Context.u16Ct_Now[i] = 0;
					}
				}
			}
			break;
		case 2:
		case 3:
			_C {
				int i;
				for (i = 0; i < 4; i++) {
					if (sAppData.sIOData_now.au16OutputPWMDuty[i] != 0xFFFF) {
						if (sPwmSpe_Context.u16Ct_Total[i] == 0) {
							sTimerPWM[i].u16duty = _PWM(1024); // 消灯
						} else {
							uint16 u16idx = sPwmSpe_Context.u16Ct_Now[i] * 100
									/ sPwmSpe_Context.u16Ct_Total[i];

							sTimerPWM[i].u16duty = (u8PwmSpe_RxMode == 2) ?
									  u16PwmSpe_SinTbl[u16idx] // サインカーブ
									: u16PwmSpe_fuwa[u16idx];  // フワッカーブ
							sTimerPWM[i].u16duty = _PWM(sTimerPWM[i].u16duty);
						}
						vTimerStart(&sTimerPWM[i]); // DUTY比だけ変更する
					}

					sPwmSpe_Context.u16Ct_Now[i]++;

					if (sPwmSpe_Context.u16Ct_Now[i]
							> sPwmSpe_Context.u16Ct_Total[i]) {
						sPwmSpe_Context.u16Ct_Now[i] = 0;
					}
				}
			}
			break;

		default:
			break;
		}
#endif
		break;

	case E_AHI_DEVICE_ANALOGUE:
		// ADCの計測値を速やかに読みだす
		u16ADC_ReadReg(&sAppData.sObjADC);
		break;

	case E_AHI_DEVICE_TICK_TIMER:
		// ADC は使用しない
#if 0
		if (sAppData.u8AdcState != 0xFF) { // 0xFF は準備中
			switch (sAppData.u8AdcState) {
			case 0:
				vSnsObj_Process(&sAppData.sADC, E_ORDER_KICK);
				sAppData.u8AdcState++;
				break;

			case 1:
				vSnsObj_Process(&sAppData.sADC, E_ORDER_KICK);

				if (bSnsObj_isComplete(&sAppData.sADC)) {
					// ADC値を処理する。
					bool_t bUpdated = bUpdateAdcValues();

					if ((sAppData.u8IOFixState & 2) == 0) {
						sAppData.bUpdatedAdc = 0;

						// 確定情報
						sAppData.u8IOFixState |= 0x2;
					} else {
						if (!IS_APPCONF_OPT_NO_ADC_BASED_TRANSMIT()) {
							sAppData.bUpdatedAdc |= bUpdated;
						}
					}

					sAppData.u8AdcState = 0x80; // 完了ステータス
				}
				break;

			case 2:
				sAppData.u8AdcState++;
				break;

			case 3:
				sAppData.u8AdcState = 0x80; // 完了ステータス
				break;

			case 0x80:
				break;

			default:
				break;
			}
		}
	#endif

		// ボタンハンドラの駆動
		if (sAppData.pr_BTM_handler) {
			// ハンドラを稼働させる
			(*sAppData.pr_BTM_handler)(sAppData.u16ToCoNetTickDelta_ms);
		}
		break;

	default:
		break;
	}

	return u8handled;
}

/** @ingroup MASTER
 * ハードウェアの初期化を行う。スリープ復帰時も原則同じ初期化手続きを行う。
 *
 * - 管理構造体のメモリ領域の初期化
 * - DO出力設定
 * - DI入力設定
 * - DI割り込み設定 (低レイテンシモード)
 * - M1-M3 の読み込み
 * - UARTの初期化
 * - ADC3/4 のプルアップ停止
 * - タイマー用の未使用ポートを汎用IOに解放する宣言
 * - 秒64回のTIMER0の初期化と稼働
 * - ADC/PWM の初期化
 * - I2Cの初期化
 *
 * @param f_warm_start TRUE:スリープ復帰時
 */
static void vInitHardware(int f_warm_start) {
	int i;

	// メモリのクリア
	memset(&sTimerApp, 0, sizeof(tsTimerContext));
	memset(&sTimerPWM[0], 0, sizeof(tsTimerContext));
	memset(&sTimerPWM[1], 0, sizeof(tsTimerContext));
	memset(&sTimerPWM[2], 0, sizeof(tsTimerContext));
	memset(&sTimerPWM[3], 0, sizeof(tsTimerContext));

	// ポートテーブルの設定
	if (!IS_APPCONF_OPT_PWM_MOVE_PORTS()) {
		vSetPortTblMap(0); // 主テーブルを選択
#if 0
		if (IS_APPCONF_OPT_NO_PULLUP_FOR_OUTPUT()) {
			vAHI_DoSetPullup(0x0, 0x3); //DO0,1のプルアップをOFFにする
			vPortDisablePullup(5);
			vPortDisablePullup(8);
		}
#endif
	} else {
		vSetPortTblMap(1); // 副テーブルを選択

		if (IS_APPCONF_OPT_NO_PULLUP_FOR_OUTPUT()) {
			vPortDisablePullup(11);
			vPortDisablePullup(12);
			vPortDisablePullup(13);
			vPortDisablePullup(17);
		}
	}

	// 出力の設定
#ifdef SET_DO_ON_SLEEP
	if (sAppData.u8Mode == E_IO_MODE_CHILD_SLP_1SEC || sAppData.u8Mode == E_IO_MODE_CHILD_SLP_10SEC) {
		for (i = 0; i < 4; i++) {
			vPortAsOutput(au8PortTbl_DOut[i]);
		}
	} else
#endif
	{
		for (i = 0; i < 4; i++) {
			// オプションビットによるプルアップの停止
			if (IS_APPCONF_OPT_NO_PULLUP_FOR_OUTPUT()) {
				vPortDisablePullup(au8PortTbl_DOut[i]);
			}

			// 出力設定
			if (sAppData.sIOData_reserve.au8Output[i] != 0xFF) {
				vDoSet_TrueAsLo(au8PortTbl_DOut[i], sAppData.sIOData_reserve.au8Output[i]);
			} else {
				vDoSetHi(au8PortTbl_DOut[i]);
			}
			vPortAsOutput(au8PortTbl_DOut[i]);
		}
	}

	// 入力の設定
	for (i = 0; i < 4; i++) {
#ifdef USE_MONOSTICK
		// オプションビットによるプルアップの停止
		if( i != 2 ){		// DI3は入力にしない
			if (IS_APPCONF_OPT_NO_PULLUP_FOR_INPUT()) {
				vPortDisablePullup(au8PortTbl_DIn[i]);
			}

			vPortAsInput(au8PortTbl_DIn[i]);
		}
#else
		// オプションビットによるプルアップの停止
		if (IS_APPCONF_OPT_NO_PULLUP_FOR_INPUT()) {
			vPortDisablePullup(au8PortTbl_DIn[i]);
		}

		vPortAsInput(au8PortTbl_DIn[i]);
#endif
	}

	// v1.3 低レイテンシで入力を行う処理
	if (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_LOW_LATENCY_INPUT) {
		// 割り込みを有効にする
		vAHI_DioInterruptEnable(u32PortInputMask, 0); // 割り込みの登録
		vAHI_DioInterruptEdge(0, u32PortInputMask); // 割り込みエッジの登録
	}

	// モード設定
	vPortAsInput(PORT_CONF1);
	vPortAsInput(PORT_CONF2);
	vPortAsInput(PORT_CONF3);
	sAppData.u8Mode = (bPortRead(PORT_CONF1) | (bPortRead(PORT_CONF2) << 1)
					| (bPortRead(PORT_CONF3) << 2));
	//モード7以外の場合は、モード0固定
	// モノライターはモード0で上がる設定になっていたのでこうしてある
	if (sAppData.u8Mode != E_IO_MODE_CHILD)
		sAppData.u8Mode = E_IO_MODE_CHILD_SLP_10SEC;

	// UART 設定
	{
		bool_t bPortBaud = FALSE;
		vPortAsInput(u8PortBaud);
		bPortBaud = bPortRead(u8PortBaud);

		uint32 u32baud = bPortBaud ? UART_BAUD_SAFE : UART_BAUD;
		tsUartOpt sUartOpt;

		memset(&sUartOpt, 0, sizeof(tsUartOpt));

		// BAUD ピンが GND になっている場合、かつフラッシュの設定が有効な場合は、設定値を採用する (v1.0.3)
		if (sAppData.bFlashLoaded && bPortBaud) {
			u32baud = sAppData.sFlash.sData.u32baud_safe;
			sUartOpt.bHwFlowEnabled = FALSE;
			sUartOpt.bParityEnabled = UART_PARITY_ENABLE;
			sUartOpt.u8ParityType = UART_PARITY_TYPE;
			sUartOpt.u8StopBit = UART_STOPBITS;

			// 設定されている場合は、設定値を採用する (v1.0.3)
			switch (sAppData.sFlash.sData.u8parity) {
			case 0:
				sUartOpt.bParityEnabled = FALSE;
				break;
			case 1:
				sUartOpt.bParityEnabled = TRUE;
				sUartOpt.u8ParityType = E_AHI_UART_ODD_PARITY;
				break;
			case 2:
				sUartOpt.bParityEnabled = TRUE;
				sUartOpt.u8ParityType = E_AHI_UART_EVEN_PARITY;
				break;
			}

			vSerialInit(u32baud, &sUartOpt);
		} else {
			vSerialInit(u32baud, NULL );
		}
	}

	// ADC3/4 のピンのプルアップを廃止する
#if 0
	// 入力の pullup は残しておく
	vPortDisablePullup(0);
	vPortDisablePullup(1);
#endif

	// モード設定ピンで Lo になっているポートはプルアップ停止
	// Lo でない場合は、プルアップ停止をするとリーク電流が発生する
	// ※ 暗電流に神経質な mode4, 7 のみ設定する。
	if (sAppData.u8Mode == E_IO_MODE_CHILD_SLP_10SEC) {
		vPortDisablePullup(PORT_CONF1);
		vPortDisablePullup(PORT_CONF2);
		vPortDisablePullup(PORT_CONF3);
	}

//	if (sAppData.u8Mode == 4) {
//		vPortDisablePullup(PORT_CONF3);
//	} else if (sAppData.u8Mode == 5) {
//		vPortDisablePullup(PORT_CONF1);
//		vPortDisablePullup(PORT_CONF3);
//	} else if (sAppData.u8Mode == 7) {
//		vPortDisablePullup(PORT_CONF1);
//		vPortDisablePullup(PORT_CONF2);
//		vPortDisablePullup(PORT_CONF3);
//	}

	// タイマの未使用ポートの解放（汎用ＩＯとして使用するため）
	vAHI_TimerFineGrainDIOControl(0x7); // bit 0,1,2 をセット (TIMER0 の各ピンを解放する, PWM1..4 は使用する)

	// 秒64回のTIMER0の初期化と稼働
	sTimerApp.u8Device = E_AHI_DEVICE_TIMER0;
	sTimerApp.u16Hz = 64;
	sTimerApp.u8PreScale = 4; // 15625ct@2^4
#if 0 //debug setting
			sTimerApp.u16Hz = 1;
			sTimerApp.u8PreScale = 10;// 15625ct@2^4
#endif
	vTimerConfig(&sTimerApp);
	vTimerStart(&sTimerApp);

	// button Manager (for Input)
#ifdef USE_I2C_PORT_AS_OTHER_FUNCTION
	sAppData.sBTM_Config.bmPortMask = u32PortInputMask | (1UL << 14) | (1UL << 15);
#else
	sAppData.sBTM_Config.bmPortMask = u32PortInputMask;
#endif

	if (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_LOW_LATENCY_INPUT) {
		sAppData.sBTM_Config.u16Tick_ms = 1;
		sAppData.sBTM_Config.u8MaxHistory = 5;
	} else {
		sAppData.sBTM_Config.u16Tick_ms = 4;
		sAppData.sBTM_Config.u8MaxHistory = 5;
	}
	sAppData.sBTM_Config.u8DeviceTimer = 0xFF; // TickTimer を流用する。
	sAppData.pr_BTM_handler = prBTM_InitExternal(&sAppData.sBTM_Config);
	vBTM_Enable();

#if 0
	// ADC
	vADC_Init(&sAppData.sObjADC, &sAppData.sADC, TRUE);
	sAppData.u8AdcState = 0xFF; // 初期化中
	sAppData.sObjADC.u8SourceMask = TEH_ADC_SRC_VOLT | TEH_ADC_SRC_TEMP
			| TEH_ADC_SRC_ADC_1 | TEH_ADC_SRC_ADC_2 | TEH_ADC_SRC_ADC_3
			| TEH_ADC_SRC_ADC_4;

	// PWM
	uint16 u16pwm_duty_default = IS_APPCONF_OPT_PWM_INIT_LOW() ? 0 : 1024; // 起動時のデフォルト
	for (i = 0; i < 4; i++) {
		uint16 u16PWM_Hz = sAppData.sFlash.sData.u32PWM_Hz[i]; // PWM周波数
		uint8 u8PWM_prescale = 0; // prescaleの設定
		if (u16PWM_Hz < 10)
			u8PWM_prescale = 9;
		else if (u16PWM_Hz < 100)
			u8PWM_prescale = 6;
		else if (u16PWM_Hz < 1000)
			u8PWM_prescale = 3;
		else
			u8PWM_prescale = 0;

		sTimerPWM[i].u16Hz = u16PWM_Hz;
		sTimerPWM[i].u8PreScale = u8PWM_prescale;
		sTimerPWM[i].u16duty =
				sAppData.sIOData_reserve.au16InputPWMDuty[i] == 0xFFFF ?
						u16pwm_duty_default : _PWM(sAppData.sIOData_reserve.au16InputPWMDuty[i]);
		sTimerPWM[i].bPWMout = TRUE;
		sTimerPWM[i].bDisableInt = TRUE; // 割り込みを禁止する指定
	}

	if (!IS_APPCONF_OPT_PWM_MOVE_PORTS()) {
		vAHI_TimerSetLocation(E_AHI_TIMER_1, TRUE, TRUE); // DIO5, DO1, DO2, DIO8
	} else {
		vAHI_TimerSetLocation(E_AHI_TIMER_1, FALSE, FALSE); // DIO11-13,17, not use DO1,2
	}

	sTimerPWM[0].u8Device = E_AHI_DEVICE_TIMER1;
	sTimerPWM[1].u8Device = E_AHI_DEVICE_TIMER2;
	sTimerPWM[2].u8Device = E_AHI_DEVICE_TIMER3;
	sTimerPWM[3].u8Device = E_AHI_DEVICE_TIMER4;

	for (i = 0; i < 4; i++) {
		vTimerConfig(&sTimerPWM[i]);
		vTimerStart(&sTimerPWM[i]);
	}
#endif

 	// I2C
#ifdef USE_I2C_PORT_AS_OTHER_FUNCTION
	vPortAsInput(14);
	vPortAsInput(15);
#else
	if (!IS_APPCONF_OPT_PWM_MOVE_PORTS()) { // PWM ポート入れ替え時は I2C は無効
		vPortDisablePullup(PORT_I2C_SCK);
		vPortDisablePullup(PORT_I2C_SDA);
#if defined(I2C_SMBUS)
		vSMBusInit();
#endif
#if defined(I2C_PORT)
		vI2C_Init();
#endif
	}
#endif
}

#ifdef USE_I2C_PORT_AS_CHANNEL_SET
/** @ingroup MASTER
 * チャネル設定を行う。
 * - EI1,EI2 の設定に基づきチャネルを変更する
 * @param u8preset
 */
static void vChangeChannelPreset(uint8 u8preset) {
	if (u8preset == 0) {
		// デフォルト値またはフラッシュ格納値を採用する
		sToCoNet_AppContext.u32ChMask = sAppData.sFlash.sData.u32chmask;
	} else {
		sToCoNet_AppContext.u32ChMask = au32ChMask_Preset[u8preset];
	}

	// 設定を反映する
	ToCoNet_vRfConfig();
}
#endif

/** @ingroup MASTER
 * UART を初期化する
 * @param u32Baud ボーレート
 */
static void vSerialInit(uint32 u32Baud, tsUartOpt *pUartOpt) {
	/* Create the debug port transmit and receive queues */
	static uint8 au8SerialTxBuffer[2560];
	static uint8 au8SerialRxBuffer[2560];

	/* Initialise the serial port to be used for debug output */
	sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
	sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
	sSerPort.u32BaudRate = u32Baud;
	sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
	sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
	sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
	sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
	sSerPort.u8SerialPort = UART_PORT_MASTER;
	sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
	SERIAL_vInitEx(&sSerPort, pUartOpt);

	/* prepare stream for vfPrintf */
	sSerStream.bPutChar = SERIAL_bTxChar;
	sSerStream.u8Device = UART_PORT_MASTER;

	/* other initialization */
	INPSTR_vInit(&sSerInpStr, &sSerStream);
	memset(&sSerCmd, 0x00, sizeof(sSerCmd));
	memset(&sSerSeqTx, 0x00, sizeof(sSerSeqTx));
	memset(&sSerSeqRx, 0x00, sizeof(sSerSeqRx));

	sSerCmd.au8data = au8SerBuffTx;
	sSerCmd.u16maxlen = sizeof(au8SerBuffTx);
}

/** @ingroup MASTER
 * 始動時メッセージの表示を行う。
 */
static void vSerInitMessage() {
	vfPrintf(&sSerStream,
			LB"!INF MONO WIRELESS TWELITE APP V%d-%02d-%d, SID=0x%08X, LID=0x%02x"LB,
			VERSION_MAIN, VERSION_SUB, VERSION_VAR, ToCoNet_u32GetSerial(),
			sAppData.u8AppLogicalId);
	if (sAppData.bFlashLoaded == 0) {
		vfPrintf(&sSerStream, "!INF Default config (no save info). .." LB);
	}
	DBGOUT(3, "u8Mode:%d"LB, sAppData.u8Mode);

	vfPrintf(&sSerStream, "!INF DIO --> %020b"LB, sAppData.u32DIO_startup);
	if (IS_APPCONF_ROLE_SILENT_MODE()) {
		vfPrintf(&sSerStream, "!ERR SILENT MODE" LB);
	}
	SERIAL_vFlush(sSerStream.u8Device);
}

/** @ingroup MASTER
 * インタラクティブモードの画面を再描画する。
 * - 本関数は TIMER_0 のイベント処理時に u16HoldUpdateScreen カウンタがデクリメントされ、
 *   これが0になった時に呼び出される。
 *
 * - 設定内容、設定値、未セーブマーク*を出力している。FL_... のマクロ記述を参照。
 *
 */
static void vSerUpdateScreen() {
	V_PRINT("%c[2J%c[H", 27, 27); // CLEAR SCREEN
	V_PRINT(
			"--- CONFIG/MONO WIRELESS TWELITE APP V%d-%02d-%d/SID=0x%08x/LID=0x%02x ---"LB,
			VERSION_MAIN, VERSION_SUB, VERSION_VAR, ToCoNet_u32GetSerial(),
			sAppData.u8AppLogicalId);

	// Application ID
	V_PRINT(" a: set Application ID (0x%08x)%c" LB,
			FL_IS_MODIFIED_u32(appid) ? FL_UNSAVE_u32(appid) : FL_MASTER_u32(appid),
			FL_IS_MODIFIED_u32(appid) ? '*' : ' ');

	// Device ID
	{
		uint8 u8DevID =
				FL_IS_MODIFIED_u8(id) ? FL_UNSAVE_u8(id) : FL_MASTER_u8(id);

		if (u8DevID == 0x00) { // unset
			V_PRINT(" i: set Device ID (--)%c"LB,
					FL_IS_MODIFIED_u8(id) ? '*' : ' ');
		} else {
			V_PRINT(" i: set Device ID (%d=0x%02x)%c"LB, u8DevID, u8DevID,
					FL_IS_MODIFIED_u8(id) ? '*' : ' ');
		}
	}

	V_PRINT(" c: set Channels (");
	{
		// find channels in ch_mask
		uint8 au8ch[MAX_CHANNELS], u8ch_idx = 0;
		int i;
		memset(au8ch, 0, MAX_CHANNELS);
		uint32 u32mask =
				FL_IS_MODIFIED_u32(chmask) ?
						FL_UNSAVE_u32(chmask) : FL_MASTER_u32(chmask);
		for (i = 11; i <= 26; i++) {
			if (u32mask & (1UL << i)) {
				if (u8ch_idx) {
					V_PUTCHAR(',');
				}
				V_PRINT("%d", i);
				au8ch[u8ch_idx++] = i;
			}

			if (u8ch_idx == MAX_CHANNELS) {
				break;
			}
		}
	}
	V_PRINT(")%c" LB, FL_IS_MODIFIED_u32(chmask) ? '*' : ' ');

	V_PRINT(" x: set Tx Power (%02x)%c" LB,
			FL_IS_MODIFIED_u8(pow) ? FL_UNSAVE_u8(pow) : FL_MASTER_u8(pow),
			FL_IS_MODIFIED_u8(pow) ? '*' : ' ');

	V_PRINT(" r: set ROOM number (%d)%c" LB,
			FL_IS_MODIFIED_u8(room) ? FL_UNSAVE_u8(room) : FL_MASTER_u8(room),
			FL_IS_MODIFIED_u8(room) ? '*' : ' ');
	V_PRINT(" t: set sensor type (%d)%c" LB,
			FL_IS_MODIFIED_u8(sensortype) ? FL_UNSAVE_u8(sensortype) : FL_MASTER_u8(sensortype),
			FL_IS_MODIFIED_u8(sensortype));
/*
	V_PRINT(" t: set mode4 sleep dur (%dms)%c" LB,
			FL_IS_MODIFIED_u16(SleepDur_ms) ? FL_UNSAVE_u16(SleepDur_ms) : FL_MASTER_u16(SleepDur_ms),
			FL_IS_MODIFIED_u16(SleepDur_ms) ? '*' : ' ');
*/

	V_PRINT(" y: set mode7 sleep dur (%ds)%c" LB,
			FL_IS_MODIFIED_u16(SleepDur_s) ? FL_UNSAVE_u16(SleepDur_s) : FL_MASTER_u16(SleepDur_s),
			FL_IS_MODIFIED_u16(SleepDur_s) ? '*' : ' ');

	V_PRINT(" f: set mode3 fps (%d)%c" LB,
			FL_IS_MODIFIED_u8(Fps) ? FL_UNSAVE_u8(Fps) : FL_MASTER_u8(Fps),
			FL_IS_MODIFIED_u8(Fps) ? '*' : ' ');

	{
		int i;
		bool_t bMod = FALSE;

		V_PRINT(" z: set PWM HZ (");
		for (i = 0; i < 4; i++) {
			if (i) V_PUTCHAR(',');
			V_PRINT("%d", FL_IS_MODIFIED_u32(PWM_Hz[i]) ? FL_UNSAVE_u32(PWM_Hz[i]) : FL_MASTER_u32(PWM_Hz[i]));

			bMod |= FL_IS_MODIFIED_u32(PWM_Hz[i]) ? TRUE : FALSE;
		}
		V_PRINT(")%c" LB, bMod ? '*' : ' ');
	}

	V_PRINT(" o: set Option Bits (0x%08X)%c" LB,
			FL_IS_MODIFIED_u32(Opt) ? FL_UNSAVE_u32(Opt) : FL_MASTER_u32(Opt),
			FL_IS_MODIFIED_u32(Opt) ? '*' : ' ');

	{
		uint32 u32baud =
				FL_IS_MODIFIED_u32(baud_safe) ?
						FL_UNSAVE_u32(baud_safe) : FL_MASTER_u32(baud_safe);
		if (u32baud & 0x80000000) {
			V_PRINT(" b: set UART baud (%x)%c" LB, u32baud,
					FL_IS_MODIFIED_u32(baud_safe) ? '*' : ' ');
		} else {
			V_PRINT(" b: set UART baud (%d)%c" LB, u32baud,
					FL_IS_MODIFIED_u32(baud_safe) ? '*' : ' ');
		}
	}

	{
		const uint8 au8name[] = { 'N', 'O', 'E' };
		V_PRINT(" p: set UART parity (%c)%c" LB,
				au8name[FL_IS_MODIFIED_u8(parity) ? FL_UNSAVE_u8(parity) : FL_MASTER_u8(parity)],
				FL_IS_MODIFIED_u8(parity) ? '*' : ' ');
	}

	V_PRINT("---"LB);

	V_PRINT(" S: save Configuration" LB " R: reset to Defaults" LB LB);
	//       0123456789+123456789+123456789+1234567894123456789+123456789+123456789+123456789
}

/** @ingroup MASTER
 * シリアルポートからの入力を処理します。
 * - シリアルポートからの入力は uart.c/serial.c により管理される FIFO キューに値が格納されます。
 *   このキューから１バイト値を得るのが SERIAL_i16RxChar() です。
 * - 本関数では、入力したバイトに対し、アプリケーションのモードに依存した処理を行います。
 *   - 文字列入力モード時(INPSTR_ API 群、インタラクティブモードの設定値入力中)は、INPSTR_u8InputByte()
 *     API に渡す。文字列が完了したときは vProcessInputString() を呼び出し、設定値の入力処理を
 *     行います。
 *   - 上記文字列入力ではない場合は、ModBusAscii_u8Parse() を呼び出します。この関数は + + + の
 *     入力判定および : で始まる書式を認識します。
 *   - 上記書式解釈中でない場合は、vProcessInputByte() を呼び出します。この関数はインタラクティブ
 *     モードにおける１文字入力コマンドを処理します。
 *
 */
void vHandleSerialInput() {
	// handle UART command
	while (!SERIAL_bRxQueueEmpty(sSerPort.u8SerialPort)) {
		int16 i16Char;

		if (!sSerSeqTx.bWaitComplete) {
			//前のコマンド処理中は、UART バッファから取り出さない
			i16Char = SERIAL_i16RxChar(sSerPort.u8SerialPort);
			DBGOUT(3, "bWaitComplete == FALSE"LB);
		} else {
			DBGOUT(3, "bWaitComplete == TRUE"LB);
			break;
		}

		// process
		if (i16Char >= 0 && i16Char <= 0xFF) {
			//DBGOUT(0, "[%02x]", i16Char);
			if (INPSTR_bActive(&sSerInpStr)) {
				// 文字列入力モード
				uint8 u8res = INPSTR_u8InputByte(&sSerInpStr, (uint8) i16Char);

				if (u8res == E_INPUTSTRING_STATE_COMPLETE) {
					vProcessInputString(&sSerInpStr);
				} else if (u8res == E_INPUTSTRING_STATE_CANCELED) {
					V_PRINT("(canceled)");
					u16HoldUpdateScreen = 64;
				}
				continue;
			}

			{
				// コマンド書式の系列解釈、および verbose モードの判定
				uint8 u8res = ModBusAscii_u8Parse(&sSerCmd, (uint8) i16Char);

				if (u8res != E_MODBUS_CMD_EMPTY) {
					V_PUTCHAR(i16Char);
				}

				if (u8res == E_MODBUS_CMD_COMPLETE
						|| u8res == E_MODBUS_CMD_LRCERROR) {
					// 解釈完了

					if (u8res == E_MODBUS_CMD_LRCERROR) {
						// command complete, but CRC error
						V_PRINT(LB "!INF LRC_ERR? (might be %02X)" LB,
								sSerCmd.u8lrc);
					}

					if (u8res == E_MODBUS_CMD_COMPLETE) {
						// process command
						vProcessSerialCmd(&sSerCmd);
					}

					continue;
				} else if (u8res != E_MODBUS_CMD_EMPTY) {
					if (u8res == E_MODBUS_CMD_VERBOSE_ON) {
						// verbose モードの判定があった
						vSerUpdateScreen();
					}

					if (u8res == E_MODBUS_CMD_VERBOSE_OFF) {
						vfPrintf(&sSerStream, "!INF EXIT INTERACTIVE MODE."LB);
					}

					// still waiting for bytes.
					continue;
				} else {
					; // コマンド解釈モードではない
				}
			}

			// Verbose モードのときは、シングルコマンドを取り扱う
			if (sSerCmd.bverbose) {
				vProcessInputByte(i16Char);
			}
		}
	}
}

/** @ingroup MASTER
 * １バイト入力コマンドの処理\n
 * - 設定値の入力が必要な項目の場合、INPSTR_vStart() を呼び出して文字列入力モードに遷移します。
 * - フラッシュへのセーブ時の手続きでは、sAppData.sConfig_UnSaved 構造体で入力が有ったものを
 *   sFlash.sData 構造体に格納しています。
 * - デバッグ用の確認コマンドも存在します。
 *
 * @param u8Byte 入力バイト
 */
static void vProcessInputByte(uint8 u8Byte) {
	static uint8 u8lastbyte;

	switch (u8Byte) {
	case 0x0d:
	case 'h':
	case 'H':
		// 画面の書き換え
		u16HoldUpdateScreen = 1;
		break;

	case 'a': // set application ID
		V_PRINT("Input Application ID (HEX:32bit): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_HEX, 8,
				E_APPCONF_APPID);
		break;

	case 'c': // チャネルの設定
		V_PRINT("Input Channel(s) (e.g. 11,16,21): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_STRING, 8,
				E_APPCONF_CHMASK);
		break;

	case 'i': // set application role
		V_PRINT("Input Device ID (DEC:1-100): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 3, E_APPCONF_ID);
		break;

	case 'x': // 出力の変更
		V_PRINT("Rf Power/Retry"
				LB "   YZ Y=Retry(0:default,F:0,1-9:count"
				LB "      Z=Power(3:Max,2,1,0:Min)"
				LB "Input: "
		);
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_HEX, 2,
				E_APPCONF_TX_POWER);
		break;

	case 'r': // ROOM 番号の変更
		V_PRINT("New room number (DEC:0-255): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 3, E_APPCONF_ROOM);
		break;

	case 't': // センサータイプ番号の変更
		V_PRINT("New sensor type number (DEC:1-99): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 2, E_APPCONF_STYPE);
		break;

/*
	case 't': // set application role
		V_PRINT("Input mode4,5 sleep dur[ms] (DEC:0,100-10000): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 5,
				E_APPCONF_SLEEP4);
		break;
*/

	case 'y': // set application role
		V_PRINT("Input mode7 sleep dur[s] (DEC:0-10000): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 5,
				E_APPCONF_SLEEP7);
		break;

	case 'f': // set application role
		V_PRINT("Input mode3 fps (DEC:4,8,16,32): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 2,
				E_APPCONF_FPS);
		break;

	case 'z': // PWMの駆動周波数の変更
		V_PRINT("Input PWM Hz (DEC:1-64000): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_STRING, 30,
				E_APPCONF_PWM_HZ);
		break;

	case 'b': // ボーレートの変更
		V_PRINT("Input baud rate (DEC:9600-230400): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_STRING, 10,
				E_APPCONF_BAUD_SAFE);
		break;

	case 'p': // パリティの変更
		V_PRINT("Input parity (N, E, O): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_STRING, 1,
				E_APPCONF_BAUD_PARITY);
		break;

	case 'o': // オプションビットの設定
		V_PRINT("Input option bits (HEX): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_HEX, 8,
				E_APPCONF_OPT);
		break;

	case 'S':
		// フラッシュへのデータ保存
		if (u8lastbyte == 'R') {
			// R S と連続入力された場合は、フラッシュエリアを消去する
			V_PRINT("!INF CLEAR SAVE AREA.");
			bFlash_Erase(FLASH_SECTOR_NUMBER - 1); // SECTOR ERASE

			vWait(1000000);
			vAHI_SwReset();
		} else {
			vConfig_SaveAndReset();
		}
		break;

	case 'R':
		vConfig_SetDefaults(&sAppData.sConfig_UnSaved);
		u16HoldUpdateScreen = 1;
		break;

	case '$':
		sAppData.u8DebugLevel++;
		if (sAppData.u8DebugLevel > 5)
			sAppData.u8DebugLevel = 0;

		V_PRINT("* set App debug level to %d." LB, sAppData.u8DebugLevel);
		break;

	case '@':
		_C {
			static uint8 u8DgbLvl;

			u8DgbLvl++;
			if (u8DgbLvl > 5)
				u8DgbLvl = 0;
			ToCoNet_vDebugLevel(u8DgbLvl);

			V_PRINT("* set NwkCode debug level to %d." LB, u8DgbLvl);
		}
		break;

	case '!':
		// リセット
		V_PRINT("!INF RESET SYSTEM.");
		vWait(1000000);
		vAHI_SwReset();
		break;

	case '#': // info
		_C {
			V_PRINT("*** ToCoNet(ver%08X) ***" LB, ToCoNet_u32GetVersion());
			V_PRINT("* AppID %08x, LongAddr, %08x, ShortAddr %04x, Tk: %d" LB,
					sToCoNet_AppContext.u32AppId, ToCoNet_u32GetSerial(),
					sToCoNet_AppContext.u16ShortAddress, u32TickCount_ms);
			if (sAppData.bFlashLoaded) {
				V_PRINT("** Conf "LB);
				V_PRINT("* AppId = %08x"LB, sAppData.sFlash.sData.u32appid);
				V_PRINT("* ChMsk = %08x"LB, sAppData.sFlash.sData.u32chmask);
				V_PRINT("* Ch=%d, Role=%d, Layer=%d"LB,
						sToCoNet_AppContext.u8Channel,
						sAppData.sFlash.sData.u8role,
						sAppData.sFlash.sData.u8layer);
			} else {
				V_PRINT("** Conf: none"LB);
			}
		}
		break;

	case 'V':
		vSerInitMessage();
		V_PRINT("---"LB);
		V_PRINT("ToCoNet lib version Core: %08x, Ext: %08x, Utils: %08x"LB,
				ToCoNet_u32GetVersion(), ToCoNet_u32GetVersion_LibEx(),
				ToCoNet_u32GetVersion_LibUtils());
		V_PRINT("ToCoNet Tick Counter: %d"LB, u32TickCount_ms);
		V_PRINT("Run Time: %d+%02d/64 sec"LB, sAppData.u32CtTimer0 >> 6,
				sAppData.u32CtTimer0 & 0x3F);
		V_PRINT(""LB);
		break;

#if DEBUG_WD
	case 'Z':
		V_PRINT("DISABLE OUTPUT PULSE"LB);
		bWDTest = FALSE;
		break;
#endif

#ifdef USE_I2C_LCD_TEST_CODE
		case '1':
		case '2':
		case '3':
		case '4':
		_C {
			bool_t bRes;
			bRes = bDraw2LinesLcd_ACM1602(astrLcdMsgs[u8Byte-'1'][0], astrLcdMsgs[u8Byte-'1'][1]);
			bRes = bDraw2LinesLcd_AQM0802A(astrLcdMsgs[u8Byte-'1'][0], astrLcdMsgs[u8Byte-'1'][1]);
			V_PRINT("I2C LCD = %d: %s,%s"LB, bRes, astrLcdMsgs[u8Byte-'1'][0], astrLcdMsgs[u8Byte-'1'][1]);
		}

		break;
#endif

	default:
		u8lastbyte = 0xFF;
		break;
	}

	// 一つ前の入力
	if (u8lastbyte != 0xFF) {
		u8lastbyte = u8Byte;
	}

}

/** @ingroup MASTER
 * 文字列入力モードの処理を行います。
 *
 */
static void vProcessInputString(tsInpStr_Context *pContext) {
	uint8 *pu8str = pContext->au8Data;
	uint8 u8idx = pContext->u8Idx;

	switch (pContext->u32Opt) {
	case E_APPCONF_APPID:
		_C {
			uint32 u32val = u32string2hex(pu8str, u8idx);

			uint16 u16h, u16l;
			u16h = u32val >> 16;
			u16l = u32val & 0xFFFF;

			if (u16h == 0x0000 || u16h == 0xFFFF || u16l == 0x0000
					|| u16l == 0xFFFF) {
				V_PRINT(
						"(ignored: 0x0000????,0xFFFF????,0x????0000,0x????FFFF can't be set.)");
			} else {
				sAppData.sConfig_UnSaved.u32appid = u32val;
			}

			V_PRINT(LB"-> %08X"LB, u32val);
		}
		break;

	case E_APPCONF_CHMASK:
		_C {
			// チャネルマスク（リスト）を入力解釈する。
			//  11,15,19 のように１０進数を区切って入力する。
			//  以下では区切り文字は任意で MAX_CHANNELS 分処理したら終了する。

			uint8 b = 0, e = 0, i = 0, n_ch = 0;
			uint32 u32chmask = 0; // 新しいチャネルマスク

			V_PRINT(LB"-> ");

			for (i = 0; i <= pContext->u8Idx; i++) {
				if (pu8str[i] < '0' || pu8str[i] > '9') {
					e = i;
					uint8 u8ch = 0;

					// 最低２文字あるときに処理
					if (e - b > 0) {
						u8ch = u32string2dec(&pu8str[b], e - b);
						if (u8ch >= 11 && u8ch <= 26) {
							if (n_ch) {
								V_PUTCHAR(',');
							}
							V_PRINT("%d", u8ch);
							u32chmask |= (1UL << u8ch);

							n_ch++;
							if (n_ch >= MAX_CHANNELS) {
								break;
							}
						}
					}
					b = i + 1;
				}

				if (pu8str[i] == 0x0) {
					break;
				}
			}

			if (u32chmask == 0x0) {
				V_PRINT("(ignored)");
			} else {
				sAppData.sConfig_UnSaved.u32chmask = u32chmask;
			}

			V_PRINT(LB);
		}
		break;

	case E_APPCONF_ID:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);
			V_PRINT(LB"-> ");
			if (u32val <= 0x7F) {
				sAppData.sConfig_UnSaved.u8id = u32val;
				V_PRINT("%d(0x%02x)"LB, u32val, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_TX_POWER:
		_C {
			uint32 u32val = u32string2hex(pu8str, u8idx);
			V_PRINT(LB"-> ");
			if ((u32val & 0xF) <= 3) {
				sAppData.sConfig_UnSaved.u8pow = u32val;
				V_PRINT("%02x"LB, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_ROOM:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);
			V_PRINT(LB"->");
			if (u32val <= 255) {
				sAppData.sConfig_UnSaved.u8room = u32val;
				V_PRINT("%d"LB, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_STYPE:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);
			V_PRINT(LB"->");
			if (u32val <= 10) {
				sAppData.sConfig_UnSaved.u8sensortype = u32val;
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

/*
	case E_APPCONF_SLEEP4:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);
			V_PRINT(LB"-> ");
			if (u32val == 0 || (u32val >= 100 && u32val <= 65534)) { // 0 はタイマーを使用しない
				sAppData.sConfig_UnSaved.u16SleepDur_ms = u32val;
				V_PRINT("%d"LB, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;
*/

	case E_APPCONF_SLEEP7:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);

			V_PRINT(LB"-> ");
			if (u32val <= 65534) { // 0 はタイマーを使用しない (
				sAppData.sConfig_UnSaved.u16SleepDur_s = u32val;
				V_PRINT("%d"LB, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_FPS:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);

			V_PRINT(LB"-> ");
			if (u32val == 4 || u32val == 8 || u32val == 16 || u32val == 32) {
				sAppData.sConfig_UnSaved.u8Fps = u32val;
				V_PRINT("%d"LB, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_PWM_HZ:
		_C {
			V_PRINT(LB"-> ");

			int i;
			#define NUM_LINE_SEP 4
			uint8 *p_tokens[NUM_LINE_SEP];
			uint8 u8n_tokens = u8StrSplitTokens(pu8str, p_tokens, NUM_LINE_SEP);

			if (u8n_tokens <= 1) {
				uint32 u32val = u32string2dec(pu8str, u8idx);

				if (u32val > 0 && u32val < 0xFFFF) {
					for (i = 0; i < 4; i++) {
						sAppData.sConfig_UnSaved.u32PWM_Hz[i] = u32val;
					}
					V_PRINT("%d for PWM1..4"LB, u32val);
				} else {
					V_PRINT("(ignored)"LB);
				}
			} else {
				for (i = 0; i < NUM_LINE_SEP && i < u8n_tokens; i++) {
					uint8 l = strlen((const char *)p_tokens[i]);

					if (i) {
						V_PRINT(",");
					}

					if ((u8n_tokens >= i + 1) && l) {
						uint32 u32val;

						u32val = u32string2dec(p_tokens[i], l);
						if (u32val > 0 && u32val < 0xFFFF) {
							sAppData.sConfig_UnSaved.u32PWM_Hz[i] = u32val;
							V_PRINT("%d", u32val);
						}
					}
				}
				V_PRINT(LB);
			}
		}
		break;

	case E_APPCONF_OPT:
		_C {
			uint32 u32val = u32string2hex(pu8str, u8idx);

			V_PRINT(LB"-> ");

			sAppData.sConfig_UnSaved.u32Opt = u32val;
			V_PRINT("0x%08X"LB, u32val);
		}
		break;

#if 0 // 意味が無いのでお蔵入り
		case E_APPCONF_SYS_HZ:
		_C {
			int i = 0;
			const uint16 u8tbl[] = {
				100, 200, 250, 400, 500, 1000, 0
			};

			uint32 u32val = u32string2dec(pu8str, u8idx);

			V_PRINT(LB"-> ");

			while(u8tbl[i]) {
				if (u8tbl[i] == u32val) {
					break;
				}
				i++;
			}

			if (u8tbl[i]) {
				sAppData.sConfig_UnSaved.u16Sys_Hz = u32val;
				V_PRINT("%d"LB, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;
#endif

	case E_APPCONF_BAUD_SAFE:
		_C {
			uint32 u32val = 0;

			if (pu8str[0] == '0' && pu8str[1] == 'x') {
				u32val = u32string2hex(pu8str + 2, u8idx - 2);
			}
			if (u8idx <= 6) {
				u32val = u32string2dec(pu8str, u8idx);
			}

			V_PRINT(LB"-> ");

			if (u32val) {
				sAppData.sConfig_UnSaved.u32baud_safe = u32val;
				if (u32val & 0x80000000) {
					V_PRINT("%x"LB, u32val);
				} else {
					V_PRINT("%d"LB, u32val);
				}
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_BAUD_PARITY:
		_C {
			V_PRINT(LB"-> ");

			if (pu8str[0] == 'N' || pu8str[0] == 'n') {
				sAppData.sConfig_UnSaved.u8parity = 0;
				V_PRINT("None"LB);
			} else if (pu8str[0] == 'O' || pu8str[0] == 'o') {
				sAppData.sConfig_UnSaved.u8parity = 1;
				V_PRINT("Odd"LB);
			} else if (pu8str[0] == 'E' || pu8str[0] == 'e') {
				sAppData.sConfig_UnSaved.u8parity = 2;
				V_PRINT("Even"LB);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	default:
		break;
	}

	// 一定時間待って画面を再描画
	u16HoldUpdateScreen = 96; // 1.5sec
}

/** @ingroup MASTER
 * シリアルから入力されたコマンド形式の電文を処理します。
 *
 * - 先頭バイトがアドレス指定で、0xDB 指定の場合、自モジュールに対しての指令となります。
 * - ２バイト目がコマンドで、0x80 以降を指定します。0x7F 以下は特別な処理は割り当てられていません。
 * - コマンド(0xDB向け)
 *   - SERCMD_ID_GET_MODULE_ADDRESS\n
 *     モジュールのシリアル番号を取得する
 * - コマンド(外部アドレス向け)
 *   - SERCMD_ID_REQUEST_IO_DATA\n
 *     IO状態の設定
 *   - それ以外のコマンドID\n
 *     通常送信 (ただし 0x80 以降は今後の機能追加で意味が変わる可能性あり)
 *
 * @param pSer シリアルコマンド入力の管理構造体
 */
static void vProcessSerialCmd(tsModbusCmd *pSer) {
	uint8 *p = pSer->au8data;

	uint8 u8addr; // 送信先論理アドレス
	uint8 u8cmd; // コマンド

	uint8 *p_end;
	p_end = &(pSer->au8data[pSer->u16len]); // the end points 1 byte after the data end.

	bool_t bTransmitRfPacket = FALSE;

	// COMMON FORMAT
	OCTET(u8addr); // [1] OCTET : 論理ID
	OCTET(u8cmd); // [1] OCTET : 要求番号

	DBGOUT(3, "* UARTCMD ln=%d cmd=%02x req=%02x %02x%0x2%02x%02x..." LB,
			pSer->u16len, u8addr, u8cmd, *p, *(p + 1), *(p + 2), *(p + 3));

	if (u8addr == SERCMD_ADDR_TO_MODULE) {
		/*
		 * モジュール自身へのコマンド (0xDB)
		 */
		switch (u8cmd) {
		case SERCMD_ID_GET_MODULE_ADDRESS:
			vModbOut_MySerial(&sSerStream);
			break;

		case SERCMD_ID_I2C_COMMAND:
/*
  シリアルでの I2C コマンド受信は禁止

#ifndef USE_I2C_PORT_AS_OTHER_FUNCTION
			if (!IS_APPCONF_OPT_PWM_MOVE_PORTS()) {
				vProcessI2CCommand(pSer->au8data, pSer->u16len, SERCMD_ADDR_TO_MODULE);
			}
#endif
*/
			break;

#if 0 // 各種設定コマンド(未実装)
			case SERCMD_ID_SET_NETWORK_CONFIG:
			/*
			 * 設定を読み出して Flash に書き込む。
			 */
			_C {
				bool_t bRet = FALSE;

				bRet = bModbIn_Config(pSer->au8data, &sAppData.sFlash.sData);

				// フラッシュへの書き込み
				if (bRet && bFlash_Write(&sAppData.sFlash, FLASH_SECTOR_NUMBER - 1, 0)) {
					vModbOut_AckNack(&sSerStream, TRUE);
					vWait(100000);
					vAHI_SwReset(); // リセットする
				} else {
					V_PRINT(LB "Failed to flash write...");
					vModbOut_AckNack(&sSerStream, FALSE);
				}
			}
			break;

			case SERCMD_ID_GET_NETWORK_CONFIG:
			if (sAppData.bFlashLoaded) {
				vModbOut_Config(&sSerStream, &sAppData.sFlash.sData);
			} else {
				vModbOut_AckNack(&sSerStream, FALSE);
			}
			break;
#endif

		default:
			break;
		}
	} else {
		/*
		 * 外部アドレスへの送信(IO情報の設定要求)
		 */
		if (u8cmd == SERCMD_ID_REQUEST_IO_DATA) {
			/*
			 * OCTET: 書式 (0x01)
			 * OCTET: 出力状態
			 * OCTET: 出力状態マスク
			 * BE_WORD: PWM1
			 * BE_WORD: PWM2
			 * BE_WORD: PWM3
			 * BE_WORD: PWM4
			 */
			uint8 u8format = G_OCTET();

			if (u8format == 0x01) {
				tsIOSetReq sIOreq;
				memset(&sIOreq, 0, sizeof(tsIOSetReq));

				sIOreq.u8IOports = G_OCTET();
				sIOreq.u8IOports_use_mask = G_OCTET();

				int i;

				for (i = 0; i < 4; i++) {
					sIOreq.au16PWM_Duty[i] = G_BE_WORD();
				}

				if (p_end < p)
					return; // v1.1.3 (終端チェック)

				DBGOUT(1, "SERCMD:IO REQ: %02x %02x %04x:%04x:%04x:%04x"LB,
						sIOreq.u8IOports, sIOreq.u8IOports_use_mask,
						sIOreq.au16PWM_Duty[0], sIOreq.au16PWM_Duty[1],
						sIOreq.au16PWM_Duty[2], sIOreq.au16PWM_Duty[3]);

				i16TransmitIoSettingRequest(u8addr, &sIOreq);
			}

			return;
		}

		/*
		 * 書式なしデータ送信
		 */
		if (sAppData.u8AppLogicalId != u8addr) {
			// 自分宛でないなら送る
			bTransmitRfPacket = TRUE;
		}
	}

	// 無線パケットを送信する
	if (bTransmitRfPacket) {
		bool_t bDoNotTransmit = FALSE;

		p = pSer->au8data; // バッファの先頭からそのまま送る
		uint16 u16len = p_end - p;

		DBGOUT(3, "* len = %d" LB, u16len);

		if (u16len > SERCMD_SER_PKTLEN * SERCMD_SER_PKTNUM || u16len <= 1) {
			// パケットが長過ぎる、または短すぎる
			bDoNotTransmit = TRUE;
		}

		if (!bDoNotTransmit) {
			// 送信リクエスト
			i16TransmitSerMsg(p, u16len, ToCoNet_u32GetSerial(),
					sAppData.u8AppLogicalId, p[0], FALSE,
					sAppData.u8UartReqNum++);
		}
	}
}

#if 0
/** @ingroup MASTER
 * I2C のコマンドを実行して、応答を返します。
 * 無線経由ので要求の場合は、応答は送信元へ無線パケットで戻されます。
 * アドレスが0xDBの場合は、要求は自身のモジュールで実行された上 UART に応答します。
 *
 * - 入力フォーマット
 *   - OCTET: ネットワークアドレス(宛先,0xDBは自身のモジュールで実行してUARTに出力)
 *   - OCTET: 0x88
 *   - OCTET: 要求番号
 *   - OCTET: コマンド (0x1: Write, 0x2: Read, 0x3: Write and Increment, 0x4: Write and Read)
 *   - OCTET: I2Cアドレス
 *   - OCTET: I2Cコマンド
 *   - OCTET: データサイズ (無い時は 0)
 *   - OCTET[N]: データ (データサイズが0のときは、本フィールドは無し)
 *
 * - 出力フォーマット
 *   - OCTET: ネットワークアドレス
 *   - OCTET: 0x89
 *   - OCTET: 要求番号、入力フォーマットの値がコピーされる
 *   - OCTET: コマンド (0x1: Write, 0x2: Read)
 *   - OCTET: 0:FAIL, 1:SUCCESS
 *   - OCTET: データサイズ (無い時は 0)
 *   - OCTET[N]: データ (データサイズが0のときは、本フィールドは無し)
 *
 * @param p 入力書式のバイト列
 * @param u16len バイト列長
 * @param u8AddrSrc 要求元のネットワークアドレス
 */
static void vProcessI2CCommand(uint8 *p, uint16 u16len, uint8 u8AddrSrc) {
	//uint8 *p_end = p + u16len;
	uint8 au8OutBuf[256 + 32];
	uint8 *q = au8OutBuf;

	bool_t bOk = TRUE;
	uint8 n;
	static volatile uint16 x;

	// 入力データの解釈
	uint8 u8Addr = G_OCTET();
	(void) u8Addr;

	uint8 u8Command = G_OCTET();
	if (u8Command != SERCMD_ID_I2C_COMMAND) {
		return;
	}

	uint8 u8ReqNum = G_OCTET();
	uint8 u8I2C_Oper = G_OCTET();
	uint8 u8I2C_Addr = G_OCTET();
	uint8 u8I2C_Cmd = G_OCTET();
	uint8 u8DataSize = G_OCTET();

	uint8 *pu8data = p;
	//uint8 *pu8data_end = p + u8DataSize;

#if 0
	if (pu8data_end != p_end) {
		DBGOUT(1, "I2CCMD: incorrect data."LB);
		return;
	}
#endif

	// 出力用のバッファを用意しておく
	S_OCTET(sAppData.u8AppLogicalId);
	S_OCTET(SERCMD_ID_I2C_COMMAND_RESP);
	S_OCTET(u8ReqNum);
	S_OCTET(u8I2C_Oper);
	//ここで q[0] 成功失敗フラグ, q[1] データサイズ, q[2]... データ
	q[0] = FALSE;
	q[1] = 0;

	DBGOUT(1, "I2CCMD: req#=%d Oper=%d Addr=%02x Cmd=%02x Siz=%d"LB, u8ReqNum,
			u8I2C_Oper, u8I2C_Addr, u8I2C_Cmd, u8DataSize);

	switch (u8I2C_Oper) {
	case 1:
		bOk &= bSMBusWrite(u8I2C_Addr, u8I2C_Cmd, u8DataSize,
				u8DataSize == 0 ? NULL : pu8data);
		break;

	case 2:
		if (u8DataSize > 0) {
			bOk &= bSMBusSequentialRead(u8I2C_Addr, u8DataSize, &(q[2]));
			if (bOk)
				q[1] = u8DataSize;
		} else {
			bOk = FALSE;
		}
		break;

	case 3:
		for (n = 0; n < u8DataSize; n++) {
			bOk &= bSMBusWrite(u8I2C_Addr, u8I2C_Cmd + n, 1, &pu8data[n]);
			for (x = 0; x < 16000; x++)
				; //wait (e.g. for memory device)
		}
		break;

	case 4:
		if (u8DataSize > 0) {
			bOk &= bSMBusWrite(u8I2C_Addr, u8I2C_Cmd, 0, NULL );
			if (bOk)
				bOk &= bSMBusSequentialRead(u8I2C_Addr, u8DataSize, &(q[2]));
			if (bOk)
				q[1] = u8DataSize;
		} else {
			bOk = FALSE;
		}
		break;

#ifdef USE_I2C_ACM1620
	case 0x21: // ACM1620
		bDraw2LinesLcd_ACM1602((const char *) pu8data,
				(const char *) (pu8data + 16));
		break;
#endif

#ifdef USE_I2C_AQM0802A
	case 0x22: // ACM1620
		bDraw2LinesLcd_AQM0802A((const char *) pu8data,
				(const char *) (pu8data + 8));
		break;
#endif

	default:
		DBGOUT(1, "I2CCMD: unknown operation(%d). "LB, u8I2C_Oper);
		return;
	}

	q[0] = bOk; // 成功失敗フラグを書き込む
	q = q + 2 + q[1]; // ポインタ q を進める（データ末尾+1)

	if (u8AddrSrc == SERCMD_ADDR_TO_MODULE) {
		vSerOutput_ModbusAscii(&sSerStream, u8AddrSrc, au8OutBuf[1],
				au8OutBuf + 2, q - au8OutBuf - 2);
	} else {
		i16TransmitSerMsg(au8OutBuf, q - au8OutBuf, ToCoNet_u32GetSerial(),
				sAppData.u8AppLogicalId, u8AddrSrc, FALSE,
				sAppData.u8UartReqNum++);
	}
}

#ifdef USE_I2C_ACM1620
/** @ingroup MASTER
 * ACM1620 LCD モジュールに文字列を書き出す。
 * - この処理は、一定時間処理をブロックし、IO判定などTIMER_0起点の動作などに影響を及ぼします。
 * - ２つのパラメータを両方とも NULL にすると、画面クリアを行います。
 *
 * @param puUpperRow 上段に書き込む文字列
 * @param puLowerRow 下段に書き込む文字列
 * @return
 */
static bool_t bDraw2LinesLcd_ACM1602(const char *puUpperRow,
		const char *puLowerRow) {
	bool_t bOk = TRUE;
	const uint8 *pu8data;
	const uint32 u32delay = 20000;

	// ディスプレーのクリア
	const uint8 au8data[] = { 0x01, 0x38, 0x0c, 0x06, 0x00 };
	pu8data = au8data;
	while (*pu8data) {
		bOk &= bSMBusWrite(0x50, 0x00, 1, (uint8*) pu8data);
		vWait(u32delay);

		pu8data++;
	}

	// 上段への書き込み
	if (puUpperRow) {
		uint8 u8data = 0x80; // 上段のアドレス指定
		int i = 0;

		bOk &= bSMBusWrite(0x50, 0x00, 1, &u8data);
		vWait(u32delay);

		pu8data = (uint8*) puUpperRow;

		while (*pu8data && i < 16) {
			bOk &= bSMBusWrite(0x50, 0x80, 1, (uint8*) pu8data);
			vWait(u32delay);

			pu8data++;
			i++;
		}
	}

	// 下段への書き込み
	if (puLowerRow) {
		uint8 u8data = 0xC0; // 下段のアドレス指定
		int i = 0;

		bOk &= bSMBusWrite(0x50, 0x00, 1, &u8data);
		vWait(u32delay);

		pu8data = (uint8*) puLowerRow;

		while (*pu8data && i < 16) {
			bOk &= bSMBusWrite(0x50, 0x80, 1, (uint8*) pu8data);
			vWait(u32delay);

			pu8data++;
			i++;
		}
	}

	return bOk;
}
#endif

#ifdef USE_I2C_AQM0802A
static bool_t bInit2LinesLcd_AQM0802A() {
	bool_t bOk = TRUE;
	const uint8 *pu8data;
	const uint32 u32delay = 2000;
	const uint8 u8addr = 0x3E;

	// ディスプレーのクリア
	const uint8 au8data[] = { 0x38, 0x39, 0x14, 0x70, 0x56, 0x6c, 0x00 };
	pu8data = au8data;
	while (*pu8data) {
		bOk &= bSMBusWrite(u8addr, 0x00, 1, (uint8*) pu8data);
		vWait(u32delay);

		pu8data++;
	}

	return bOk;
}

/** @ingroup MASTER
 * ACM1620 LCD モジュールに文字列を書き出す。
 * - この処理は、一定時間処理をブロックし、IO判定などTIMER_0起点の動作などに影響を及ぼします。
 * - ２つのパラメータを両方とも NULL にすると、画面クリアを行います。
 *
 * @param puUpperRow 上段に書き込む文字列
 * @param puLowerRow 下段に書き込む文字列
 * @return
 */
static bool_t bDraw2LinesLcd_AQM0802A(const char *puUpperRow,
		const char *puLowerRow) {
	bool_t bOk = TRUE;
	const uint8 *pu8data;
	const uint32 u32delay = 2000;
	const uint8 u8addr = 0x3E;

	static bool_t bInit;

	// ディスプレーのクリア
	if (!bInit) {
		bOk &= bInit2LinesLcd_AQM0802A();
		vWait(400000);
		bInit = TRUE;
	}

	const uint8 au8data2[] = { 0x38, 0x0c, 0x01, 0x06, 0x00 };
	pu8data = au8data2;
	while (*pu8data) {
		bOk &= bSMBusWrite(u8addr, 0x00, 1, (uint8*) pu8data);
		vWait(u32delay);

		pu8data++;
	}

	// 上段への書き込み
	if (puUpperRow) {
		uint8 u8data = 0x80; // 上段のアドレス指定
		int i = 0;

		bOk &= bSMBusWrite(u8addr, 0x00, 1, &u8data);
		vWait(u32delay);

		pu8data = (uint8*) puUpperRow;

		while (*pu8data && i < 8) {
			bOk &= bSMBusWrite(u8addr, 0x40, 1, (uint8*) pu8data);
			vWait(u32delay);

			pu8data++;
			i++;
		}
	}

	// 下段への書き込み
	if (puLowerRow) {
		uint8 u8data = 0xC0; // 下段のアドレス指定
		int i = 0;

		bOk &= bSMBusWrite(u8addr, 0x00, 1, &u8data);
		vWait(u32delay);

		pu8data = (uint8*) puLowerRow;

		while (*pu8data && i < 8) {
			bOk &= bSMBusWrite(u8addr, 0x40, 1, (uint8*) pu8data);
			vWait(u32delay);

			pu8data++;
			i++;
		}
	}

	return bOk;
}
#endif
#endif	// vProcessI2Command

/** @ingroup MASTER
 * 重複パケットの判定。タイムスタンプの比較で、ごく最近であって旧いパケットを除外する。
 *
 * - 注意点
 *   - DUPCHK_vInin() の初期化を行うこと
 *   - DUPCHK_bFIND(0,NULL) を一定周期で呼び出すこと
 *
 * @param pc 管理構造体
 * @param u32Addr
 * @param u16TimeStamp
 * @return TRUE:重複している FALSE:新しいパケットが来た
 */
static bool_t bCheckDupPacket(tsDupChk_Context *pc, uint32 u32Addr,
		uint16 u16TimeStamp) {
	uint32 u32Key;
	if (DUPCHK_bFind(pc, u32Addr, &u32Key)) {
		// 最後に受けたカウンタの値が得られるので、これより新しい
		uint16 u16Delta = ((uint16) u32Key - u16TimeStamp) & 0x7FFF; // 最上位ビットは設定されない
		if (u16Delta < 32) { // 32count=500ms, 500ms の遅延は想定外だが。
			// すでに処理したパケット
			return TRUE;
		}
	}

	// 新しいパケットである（時間情報を格納する）
	DUPCHK_vAdd(pc, u32Addr, u16TimeStamp);
	return FALSE;
}

/** @ingroup MASTER
 * IO 情報を送信します。
 *
 * - IO状態の変化、および１秒置きの定期送時に呼び出されます。
 *
 * - Packet 構造
 *   - OCTET: 識別ヘッダ(APP ID より生成)
 *   - OCTET: プロトコルバージョン(バージョン間干渉しないための識別子)
 *   - OCTET: 送信元論理ID
 *   - BE_DWORD: 送信元のシリアル番号
 *   - OCTET: 宛先論理ID
 *   - BE_WORD: 送信タイムスタンプ (64fps カウンタの値の下１６ビット, 約1000秒で１周する)
 *   - OCTET: 中継フラグ(中継したパケットは１、最初の送信なら０を格納）
 *   - BE_WORD: 電圧
 *   - OCTET: 温度 (int8型)  ※ TODO: 値が不正確。ADC 値から温度への変換式がメーカより開示されないため。
 *   - OCTET: ボタン (LSB から順に SW1 ... SW4, 1=Lo), 0x80ビットは通常送信の識別用フラグ
 *   - OCTET: ボタン変化 (LSB から順に SW1 ... SW4, 1=変化)
 *   - OCTET: ADC1 (MSB から 8bit)
 *   - OCTET: ADC2 (MSB から 8bit)
 *   - OCTET: ADC3 (MSB から 8bit)
 *   - OCTET: ADC4 (MSB から 8bit)
 *   - OCTET: ADC 詳細部 (MSB b8b7b6b5b4b3b2b1 LSB とすると b2b1 が ADC1 の LSB 2bit, 以下同様)
 *
 * - ADC 値の復元方法
 *   - 8bit ADC値の場合 16倍すると mV 値になります。
 *   - ADC詳細部の2bitを追加し 10bit ADC 値の場合 4 倍すると mV 値なります。
 *
 * @returns -1:ERROR, 0..255 CBID
 */
static int16 i16TransmitIoData(uint8 u8Quick, bool_t bRegular) {
	if (IS_APPCONF_ROLE_SILENT_MODE())
		return -1;

	int16 i16Ret = -1;
	tsTxDataApp sTx;
	memset(&sTx, 0, sizeof(sTx));

	uint8 *q = sTx.auData;

	int i;

	// ペイロードを構成
	S_OCTET(sAppData.u8AppIdentifier);
	S_OCTET(APP_PROTOCOL_VERSION);
	S_OCTET(sAppData.u8AppLogicalId); // アプリケーション論理アドレス
	S_BE_DWORD(ToCoNet_u32GetSerial());  // シリアル番号
	S_OCTET(IS_LOGICAL_ID_PARENT(sAppData.u8AppLogicalId) ? LOGICAL_ID_CHILDREN : LOGICAL_ID_PARENT); // 宛先

	S_BE_WORD((sAppData.u32CtTimer0 & 0x7FFF) + (u8Quick == TX_NODELAY_AND_QUICK_BIT ? 0x8000 : 0)); // タイムスタンプ

	// bQuick 転送する場合は MSB をセットし、優先パケットである処理を行う
	S_OCTET(0); // 中継フラグ

	S_BE_WORD(sAppData.sIOData_now.u16Volt); // 電圧

	// チップ内温度センサーの予定だったが・・・
#ifdef USE_I2C_PORT_AS_PWM_SPECIAL
	S_OCTET(u8PwmSpe_TxMode); //温度だがやめ
#else
	S_OCTET((uint8)((sAppData.sIOData_now.i16Temp + 50)/100)); //チップ内温度センサー(TWE-Liteでは正しく動作しない)
#endif

	// ボタンのビットマップ
	{
		int i;
		uint8 u8bm = 0;

		for (i = 0; i < 4; i++) {
			uint8 u8ct = sAppData.sIOData_now.au8Input[i] >> 4;
#ifdef USE_MONOSTICK
			if(i != 2){
				if (u8ct >= LOW_LATENCY_DELAYED_TRANSMIT_COUNTER - 3) { // カウンタ値が残っている場合は 1 を送る
					u8bm |= (1 << i);
				} else {
					u8bm |= (sAppData.sIOData_now.au8Input[i] & 1) ? (1 << i) : 0;
				}
			}else{
				u8bm = 0;
			}
#else
			if (u8ct >= LOW_LATENCY_DELAYED_TRANSMIT_COUNTER - 3) { // カウンタ値が残っている場合は 1 を送る
				u8bm |= (1 << i);
			} else {
				u8bm |= (sAppData.sIOData_now.au8Input[i] & 1) ? (1 << i) : 0;
			}
#endif
		}

		if (bRegular) u8bm |= 0x80; // MSB を設定(bRegularビット)

		S_OCTET(u8bm);
	}

	// ボタンのビットマップ使用フラグ (１度でもLoになったポートは１になる）
	{
		uint8 i, c = 0x0;

#ifdef USE_MONOSTICK
		for (i = 0; i < 4; i++) {
			if( i != 2 ){
				c |= (sAppData.sIOData_now.u32BtmUsed & (1UL << au8PortTbl_DIn[i])) ? (1 << i) : 0;
			}
		}
#else
		for (i = 0; i < 4; i++) {
			c |= (sAppData.sIOData_now.u32BtmUsed & (1UL << au8PortTbl_DIn[i])) ? (1 << i) : 0;
		}
#endif

		if (u8Quick == TX_NODELAY_AND_RESP_BIT) c |= 0x80;

		S_OCTET(c);
	}

	// ADC 部のエンコード
	uint8 u8LSBs = 0;
	for (i = 0; i < 4; i++) {
		// MSB 部分 (10bit目～3bit目まで)
		uint16 u16v = sAppData.sIOData_now.au16InputADC[i];
		u16v >>= 2; // ADC 値は 0...2400mV

		uint8 u8MSB = (u16v >> 2) & 0xFF;
		S_OCTET(u8MSB);

		// 下2bitを u8LSBs に詰める
		u8LSBs >>= 2;
		u8LSBs |= ((u16v << 6) & 0xC0); //
	}
	S_OCTET(u8LSBs); // 詳細ビット部分を記録

	sTx.u8Len = q - sTx.auData; // パケット長
	sTx.u8Cmd = TOCONET_PACKET_CMD_APP_USER_IO_DATA; // パケット種別

	// 送信する
	sTx.u32DstAddr = TOCONET_MAC_ADDR_BROADCAST; // ブロードキャスト
	sTx.u8Retry = sAppData.u8StandardTxRetry; // 再送

	// フレームカウントとコールバック識別子の指定
	sAppData.u16TxFrame++;
	sTx.u8Seq = (sAppData.u16TxFrame & 0xFF);
	sTx.u8CbId = sTx.u8Seq;

	{
		/* MAC モードでは細かい指定が可能 */
		sTx.bAckReq = FALSE;
		sTx.u32SrcAddr = sToCoNet_AppContext.u16ShortAddress;

		// 送信タイミングの調整
		switch (u8Quick) {
		case TX_NODELAY:
		case TX_NODELAY_AND_QUICK_BIT:
		case TX_NODELAY_AND_RESP_BIT:
			sTx.u16RetryDur = 0; // 再送間隔
			sTx.u16DelayMin = 0; // すみやかに送信する
			sTx.u16DelayMax = 0; // すみやかに送信する
			break;
		case TX_SMALLDELAY:
		case TX_REPONDING:
			sTx.u16RetryDur = 0; // 再送間隔
			sTx.u16DelayMin = 4; // 衝突を抑制するため送信タイミングを遅らせる
			sTx.u16DelayMax = 8; // 衝突を抑制するため送信タイミングを遅らせる
			break;
		default:
			sTx.u16RetryDur = 4; // 再送間隔
			sTx.u16DelayMin = 0; // 衝突を抑制するため送信タイミングを遅らせる
			sTx.u16DelayMax = 16; // 衝突を抑制するため送信タイミングにブレを作る(最大16ms)
			break;
		}

#ifdef USE_SLOW_TX
	    //ここから
	    if (u8Quick == 0x10) {
	      sTx.u8Retry = sAppData.u8StandardTxRetry; // 再送回数を３回とする
	      sTx.u16DelayMax = 100; // 初回送信は送信要求発行時～100ms の間（ランダムで決まる）
	      sTx.u16RetryDur = 20; // 20ms おきに再送する
	    }
	    //ここまで
#endif

		// 送信API
		if (ToCoNet_bMacTxReq(&sTx)) {
			if (sTx.u16DelayMax == 0) {
				// すぐに送る場合
				ToCoNet_Tx_vProcessQueue();
			}

			i16Ret = sTx.u8CbId;
			sAppData.sIOData_now.u32TxLastTick = u32TickCount_ms;
		}
	}

	return i16Ret;
}

/** @ingroup MASTER
 * IOデータを中継送信します。
 *
 * - パケット中の中継フラグのビットは、呼び出し前に設定されています。
 * - 衝突を抑制出来る程度の送信遅延、再送間隔を設定しています。
 *
 * @param pRx 受信したときのデータ
 * @return -1:Error, 0..255:CbId
 */
int16 i16TransmitRepeat(tsRxDataApp *pRx) {
	if (IS_APPCONF_ROLE_SILENT_MODE())
		return -1;

	int16 i16Ret = -1;

	tsTxDataApp sTx;
	memset(&sTx, 0, sizeof(sTx));

	// Payload
	memcpy(sTx.auData, pRx->auData, pRx->u8Len);
	sTx.u8Len = pRx->u8Len;

	// コマンド設定
	sTx.u8Cmd = pRx->u8Cmd; // パケット種別

	// 送信する
	sTx.u32DstAddr = TOCONET_MAC_ADDR_BROADCAST; // ブロードキャスト
	sTx.u8Retry = sAppData.u8StandardTxRetry; // ２回再送

	// フレームカウントとコールバック識別子の指定
	sAppData.u16TxFrame++;
	sTx.u8Seq = (sAppData.u16TxFrame & 0xFF);
	sTx.u8CbId = sTx.u8Seq;

	// 中継時の送信パラメータ
	sTx.bAckReq = FALSE;
	sTx.u32SrcAddr = sToCoNet_AppContext.u16ShortAddress;
	sTx.u16RetryDur = 8; // 再送間隔

	sTx.u16DelayMin = 16; // 衝突を抑制するため送信タイミングにブレを作る(最大16ms)
	sTx.u16DelayMax = 16; // 衝突を抑制するため送信タイミングにブレを作る(最大16ms)

	// 送信API
	if (ToCoNet_bMacTxReq(&sTx)) {
		i16Ret = sTx.u8CbId;
	}

	return i16Ret;
}

/** @ingroup MASTER
 * IO(DO/PWM)を設定する要求コマンドパケットを送信します。
 *
 * - Packet 構造
 *   - OCTET: 識別ヘッダ(APP ID より生成)
 *   - OCTET: プロトコルバージョン(バージョン間干渉しないための識別子)
 *   - OCTET: 送信元論理ID
 *   - BE_DWORD: 送信元のシリアル番号
 *   - OCTET: 宛先論理ID
 *   - BE_WORD: 送信タイムスタンプ (64fps カウンタの値の下１６ビット, 約1000秒で１周する)
 *   - OCTET: 中継フラグ
 *   - OCTET: 形式 (1固定)
 *   - OCTET: ボタン (LSB から順に SW1 ... SW4, 1=Lo)
 *   - OCTET: ボタン使用フラグ (LSB から順に SW1 ... SW4, 1=このポートを設定する)
 *   - BE_WORD: PWM1 (0..1024 or 0xffff) 0xffff を設定すると、このポートの設定をしない。
 *   - BE_WORD: PWM2 (0..1024 or 0xffff)
 *   - BE_WORD: PWM3 (0..1024 or 0xffff)
 *   - BE_WORD: PWM4 (0..1024 or 0xffff)
 *
 * @param u8DstAddr 送信先
 * @param pReq 設定データ
 * @return -1:Error, 0..255:CbId
 */
int16 i16TransmitIoSettingRequest(uint8 u8DstAddr, tsIOSetReq *pReq) {
	if (IS_APPCONF_ROLE_SILENT_MODE())
		return -1;

	int16 i16Ret, i;

	tsTxDataApp sTx;
	memset(&sTx, 0, sizeof(sTx));

	uint8 *q = sTx.auData;

	S_OCTET(sAppData.u8AppIdentifier);
	S_OCTET(APP_PROTOCOL_VERSION);
	S_OCTET(sAppData.u8AppLogicalId); // アプリケーション論理アドレス
	S_BE_DWORD(ToCoNet_u32GetSerial());  // シリアル番号
	S_OCTET(u8DstAddr); // 宛先
	S_BE_WORD(sAppData.u32CtTimer0 & 0xFFFF); // タイムスタンプ
	S_OCTET(0); // 中継フラグ

	S_OCTET(1); // パケット形式

	// DIO の設定
	S_OCTET(pReq->u8IOports);
	S_OCTET(pReq->u8IOports_use_mask);

	// PWM の設定
	for (i = 0; i < 4; i++) {
		S_BE_WORD(pReq->au16PWM_Duty[i]);
	}

	sTx.u8Len = q - sTx.auData; // パケット長
	sTx.u8Cmd = TOCONET_PACKET_CMD_APP_USER_IO_DATA_EXT; // パケット種別

	// 送信する
	sTx.u32DstAddr = TOCONET_MAC_ADDR_BROADCAST; // ブロードキャスト
	sTx.u8Retry = sAppData.u8StandardTxRetry; // 1回再送

	{
		/* 送信設定 */
		sTx.bAckReq = FALSE;
		sTx.u32SrcAddr = sToCoNet_AppContext.u16ShortAddress;
		sTx.u16RetryDur = 4; // 再送間隔
		sTx.u16DelayMax = 16; // 衝突を抑制するため送信タイミングにブレを作る(最大16ms)

		// 送信API
		if (ToCoNet_bMacTxReq(&sTx)) {
			i16Ret = sTx.u8CbId;
		}
	}

	return i16Ret;
}

/** @ingroup MASTER
 * SHT31 ステータス・レジスタの内容を圧縮して送る
 * @param	SHT31 のステータス・レジスタ
 * @return	圧縮された情報
 */
static uint8 getSHT31Status(uint16 status) {
	uint8 s = 0x00;
	if (status & 0x8000) s |= 0x80;
	if (status & 0x2000) s |= 0x40;
	if (status & 0x0800) s |= 0x20;
	if (status & 0x0400) s |= 0x10;
	if (status & 0x0010) s |= 0x08;
	if (status & 0x0002) s |= 0x02;
	if (status & 0x0001) s |= 0x01;
	return s;
}

/** @ingroup MASTER
 * シリアルメッセージを構成して送る。i16TransmitSerMsg を使用する。
 * 常に親機に送信する。
 */
static int16 i16TransmitSerData() {
	uint8 payload[9], *q = payload, *q_s = q;

	// 送信元の論理ID[0]
	// この ID は親機では表示されない(vReceiveSerMsg() を参照のこと)。
	// 親機に表示されるのはヘッダに埋め込まれた送信元論理IDである。
	S_OCTET(sAppData.u8AppLogicalId);

	// コマンド番号[1]
	S_OCTET(SERCMD_ID_I2C_MEASURE);

	// デバイス種別[2]
	S_OCTET(sAppData.sFlash.sData.u8sensortype);

	// 部屋番号[3]
	S_OCTET(sAppData.sFlash.sData.u8room);

	// ステータス[4]
	uint8 st8 = getSHT31Status(sAppData.u16status);
	S_OCTET(st8);

	// 温度[5]
	S_BE_WORD(sAppData.u16temperature);

	// 湿度[6]
	S_BE_WORD(sAppData.u16humidity);

	return i16TransmitSerMsg(q_s, (uint16)(q - q_s), ToCoNet_u32GetSerial(), sAppData.u8AppLogicalId,
		IS_LOGICAL_ID_PARENT(sAppData.u8AppLogicalId) ? LOGICAL_ID_CHILDREN : LOGICAL_ID_PARENT,
		0, sAppData.u8UartReqNum++);
}

/** @ingroup MASTER
 * シリアルメッセージの送信要求を行います。１パケットを分割して送信します。
 *
 * - Packet 構造
 *   - OCTET    : 識別ヘッダ(APP ID より生成)
 *   - OCTET    : プロトコルバージョン(バージョン間干渉しないための識別子)
 *   - OCTET    : 送信元個体識別論理ID
 *   - BE_DWORD : 送信元シリアル番号
 *   - OCTET    : 送信先シリアル番号
 *   - OCTET    : 送信タイムスタンプ (64fps カウンタの値の下１６ビット, 約1000秒で１周する)
 *   - OCTET    : 送信フラグ(リピータ用)
 *   - OCTET    : 要求番号
 *   - OCTET    : パケット数(total)
 *   - OCTET    : パケット番号 (0...total-1)
 *   - BE_WORD  : 本パケットのバッファ位置
 *   - OCTET    : 本パケットのデータ長
 *
 * @param p ペイロードのデータへのポインタ
 * @param u16len ペイロード長
 * @param bRelay 中継フラグ TRUE:中継する
 * @return -1:失敗, 0:成功
 */
static int16 i16TransmitSerMsg(uint8 *p, uint16 u16len, uint32 u32AddrSrc,
		uint8 u8AddrSrc, uint8 u8AddrDst, uint8 u8RelayLv, uint8 u8Req) {
	if (IS_APPCONF_ROLE_SILENT_MODE())
		return -1;

	// パケットを分割して送信する。
	tsTxDataApp sTx;
	memset(&sTx, 0, sizeof(sTx));
	uint8 *q; // for S_??? macros

	// 処理中のチェック（処理中なら送信せず失敗）
	if (sSerSeqTx.bWaitComplete) {
		return -1;
	}

	// sSerSeqTx は分割パケットの管理構造体
	sSerSeqTx.u8IdSender = sAppData.u8AppLogicalId;
	sSerSeqTx.u8IdReceiver = u8AddrDst;

	sSerSeqTx.u8PktNum = (u16len - 1) / SERCMD_SER_PKTLEN + 1;
	sSerSeqTx.u16DataLen = u16len;
	sSerSeqTx.u8Seq = sSerSeqTx.u8SeqNext; // パケットのシーケンス番号（アプリでは使用しない）
	sSerSeqTx.u8SeqNext = sSerSeqTx.u8Seq + sSerSeqTx.u8PktNum; // 次のシーケンス番号（予め計算しておく）
	sSerSeqTx.u8ReqNum = u8Req; // パケットの要求番号（この番号で送信系列を弁別する）
	sSerSeqTx.bWaitComplete = TRUE;
	sSerSeqTx.u32Tick = u32TickCount_ms;
	memset(sSerSeqTx.bPktStatus, 0, sizeof(sSerSeqTx.bPktStatus));

	DBGOUT(3, "* >>> Transmit(req=%d) Tick=%d <<<" LB, sSerSeqTx.u8ReqNum,
			u32TickCount_ms & 65535);

	sTx.u8Cmd = TOCONET_PACKET_CMD_APP_DATA; // data packet.
	sTx.u8Retry = sAppData.u8StandardTxRetry;
	sTx.u16RetryDur = sSerSeqTx.u8PktNum * 10; // application retry

	int i;
	for (i = 0; i < sSerSeqTx.u8PktNum; i++) {
		q = sTx.auData;
		sTx.u8Seq = sSerSeqTx.u8Seq + i;
		sTx.u8CbId = sTx.u8Seq; // callback will reported with this ID

		// ペイロードを構成
		S_OCTET(sAppData.u8AppIdentifier);
		S_OCTET(APP_PROTOCOL_VERSION);
		S_OCTET(u8AddrSrc); // アプリケーション論理アドレス
		S_BE_DWORD(u32AddrSrc);  // シリアル番号
		S_OCTET(sSerSeqTx.u8IdReceiver); // 宛先
		S_BE_WORD(sAppData.u32CtTimer0 & 0xFFFF); // タイムスタンプ

		S_OCTET(u8RelayLv); // 中継レベル

		S_OCTET(sSerSeqTx.u8ReqNum); // request number
		S_OCTET(sSerSeqTx.u8PktNum); // total packets
		S_OCTET(i); // packet number
		S_BE_WORD(i * SERCMD_SER_PKTLEN); // offset

		uint8 u8len_data =
				(u16len >= SERCMD_SER_PKTLEN) ? SERCMD_SER_PKTLEN : u16len;
		S_OCTET(u8len_data);

		memcpy(q, p, u8len_data);
		q += u8len_data;

		sTx.u8Len = q - sTx.auData;

		// あて先など
		sTx.u32DstAddr = TOCONET_MAC_ADDR_BROADCAST; // ブロードキャスト

		if (sAppData.eNwkMode == E_NWKMODE_MAC_DIRECT) {
			sTx.u32SrcAddr = sToCoNet_AppContext.u16ShortAddress;
			sTx.bAckReq = FALSE;
			sTx.u8Retry = sAppData.u8StandardTxRetry;

			ToCoNet_bMacTxReq(&sTx);
		}

		p += u8len_data;
		u16len -= SERCMD_SER_PKTLEN;
	}

	return 0;
}

/** @ingroup MASTER
 * IO状態パケットの受信処理を行います。
 *
 * - 受信したデータに格納されるIO設定要求に従いIO値(DO/PWM)を設定します。
 * - 受信したデータを UART に出力します。
 * - 中継機の場合、中継パケットを送信します。
 * - 低遅延で送信されてきたパケットの TimeStamp の MSB には１が設定される。
 *   このパケットは、タイムスタンプによる重複除去アルゴリズムとは独立して
 *   処理される。
 * - IOの設定は本受信関数でのみ行われる。
 *
 * @param pRx 受信データ
 */
static void vReceiveIoData(tsRxDataApp *pRx) {
	int i, j;
	uint8 *p = pRx->auData;

	uint8 u8AppIdentifier = G_OCTET();
	if (u8AppIdentifier != sAppData.u8AppIdentifier)
		return;

	uint8 u8PtclVersion = G_OCTET();
	if (u8PtclVersion != APP_PROTOCOL_VERSION)
		return;

	uint8 u8AppLogicalId = G_OCTET();

	uint32 u32Addr = G_BE_DWORD();

	uint8 u8AppLogicalId_Dest = G_OCTET();
	(void) u8AppLogicalId_Dest;

	uint16 u16TimeStamp = G_BE_WORD();

	/* 重複の確認を行う */
	bool_t bQuick = u16TimeStamp & 0x8000 ? TRUE : FALSE; // 優先パケット（全部処理する）
	u16TimeStamp &= 0x7FFF;
	if (bQuick == FALSE
			&& bCheckDupPacket(&sDupChk_IoData, u32Addr, u16TimeStamp)) {
		return;
	}
	static uint32 u32LastQuick;
	if (bQuick) {
		if ((u32TickCount_ms - u32LastQuick) < 20) {
			// Quickパケットを受けて一定期間未満のパケットは無視する
			return;
		} else {
			u32LastQuick = u32TickCount_ms; // タイムスタンプを巻き戻す
		}
	}

	// 中継フラグ
	uint8 u8TxFlag = G_OCTET();

	// 中継の判定 (レベルに達していなければ中継する）
	if (sAppData.u8Mode == E_IO_MODE_ROUTER || (sAppData.u8Mode == E_IO_MODE_CHILD && IS_APPCONF_OPT_ROUTING_CHILD())) {
		if (u8TxFlag < sAppData.u8max_hops) {
			// リピータの場合はここで中継の判定
			*(p - 1) = *(p - 1) + 1; // 中継済みフラグのセット
			// 中継する
			i16TransmitRepeat(pRx); // 中継パケットの送信
		}

		// 専業中継機の場合は、ここで終了
		if (sAppData.u8Mode == E_IO_MODE_ROUTER ) {
			return;
		}
	}

	// 親機子機の判定
	if ((IS_LOGICAL_ID_PARENT(u8AppLogicalId)
			&& IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId))
	|| (IS_LOGICAL_ID_CHILD(u8AppLogicalId) && IS_LOGICAL_ID_PARENT(sAppData.u8AppLogicalId)) ){
		; // 親機⇒子機、または子機⇒親機への伝送
	} else {
		return;
	}

	/* 電圧 */
	uint16 u16Volt = G_BE_WORD();

	/* 温度 */
#ifdef USE_I2C_PORT_AS_PWM_SPECIAL
	u8PwmSpe_RxMode = G_OCTET();
#else
	int8 i8Temp = (int8)G_OCTET();
	(void)i8Temp;
#endif

	/* BUTTON */
	uint8 u8ButtonState = G_OCTET();
	bool_t bRegular = !!(u8ButtonState & 0x80); // 通常送信パケットであることを意味する
	uint8 u8ButtonChanged = G_OCTET();
	bool_t bRespReq = !!(u8ButtonChanged & 0x80); // 速やかな応答を要求する

	if (sAppData.prPrsEv == vProcessEvCoreSlpRecv) {
		// 間欠受信モードでは、無条件に全ポートを書き換える /////////////////////////////
		//   以下の動作パターンを想定
		//     子機が間欠受信で DO1=LO に設定
		//     親機が電源OFF
		//     その後、親機のDI1=HI に戻る
		//     親機が電源ON
		//   この場合、親機のボタン変更マスク DI1=未使用 と設定され、子機DO1 が HI に戻る
		//   事が無いため。
		//
		//   通常モードでも同様の現象は発生するが、用途要件によってどちらが良いかは一意に
		//   決められないため、そのままとする。

		u8ButtonChanged = 0x0F;
	}

	// ポートの値を設定する（変更フラグのあるものだけ）
	for (i = 0, j = 1; i < 4; i++, j <<= 1) {
		if (u8ButtonChanged & j) {
			vDoSet_TrueAsLo(au8PortTbl_DOut[i], u8ButtonState & j);
#ifdef USE_I2C_LCD_TEST_CODE
			if ((u8ButtonState & j) && sAppData.sIOData_now.au8Output[i] == 0) {
				// 押し下げ時に限って処理する
				bDraw2LinesLcd_ACM1602(astrLcdMsgs[i][0], astrLcdMsgs[i][1]);
				bDraw2LinesLcd_AQM0802A(astrLcdMsgs[i][0], astrLcdMsgs[i][1]);
			}
#endif
			sAppData.sIOData_now.au8Output[i] = u8ButtonState & j;
		}
	}

	/* ADC 値 */
	for (i = 0; i < 4; i++) {
		// 8bit scale
		sAppData.sIOData_now.au16OutputDAC[i] = G_OCTET();
		if (sAppData.sIOData_now.au16OutputDAC[i] == 0xFF) {
			sAppData.sIOData_now.au16OutputDAC[i] = 0xFFFF;
		} else {
			// 10bit scale に拡張
			sAppData.sIOData_now.au16OutputDAC[i] <<= 2;
		}
	}
	uint8 u8DAC_Fine = G_OCTET();
	for (i = 0; i < 4; i++) {
		if (sAppData.sIOData_now.au16OutputDAC[i] != 0xFFFF) {
			// 下２ビットを復旧
			sAppData.sIOData_now.au16OutputDAC[i] |= (u8DAC_Fine & 0x03);
		}
		u8DAC_Fine >>= 2;
	}

	// ADC 値を PWM の DUTY 比に変換する
	// 下は 5%, 上は 10% を不感エリアとする。
	for (i = 0; i < 4; i++) {
		uint16 u16Adc = sAppData.sIOData_now.au16OutputDAC[i];
		u16Adc <<= 2; // 以下の計算は 12bit 精度
		if (u16Adc > ADC_MAX_THRES) { // 最大レンジの 98% 以上なら、未定義。
			sAppData.sIOData_now.au16OutputPWMDuty[i] = 0xFFFF;
		} else {
			// 10bit+1 スケール正規化
#ifdef USE_I2C_PORT_AS_PWM_SPECIAL
			int32 iR = (uint32) u16Adc * 2 * 1125 / u16Volt; // スケールは 0～Vcc/2 なので 2倍する

			// CカーブをBに戻す
			if (iR > 1024)
				iR = 1024;
			iR = au16_AtoBcurve[iR];

			sAppData.sIOData_now.au16OutputPWMDuty[i] = iR;
#else
			int32 iS;
			if (IS_APPCONF_OPT_ADC_TO_PWM_RAW_OUT()) { // ADC の絶対電圧で PWM の DUTY を決める
				// 1800mV で正規化する
				iS = (uint32)u16Adc * 1024 / 1800;
				if (iS >= 1024) {
					iS = 1024;
				}
			} else { // ADC 電圧を Vcc で相対化して PWM の DUTY を決める
				int32 iR = (uint32) u16Adc * 2 * 1024 / u16Volt; // スケールは 0～Vcc/2 なので 2倍する
				// y = 1.15x - 0.05 の線形変換
				//   = (115x-5)/100 = (23x-1)/20 = 1024*(23x-1)/20/1024 = 51.2*(23x-1)/1024 ~= 51*(23x-1)/1024
				// iS/1024 = 51*(23*iR/1024-1)/1024
				// iS      = (51*23*iR - 51*1024)/1024
				iS = 51 * 23 * iR - 51 * 1024;
				if (iS <= 0) {
					iS = 0;
				} else {
					iS >>= 10; // 1024での割り算
					if (iS >= 1024) { // DUTY は 0..1024 で正規化するので最大値は 1024。
						iS = 1024;
					}
				}
			}
			sAppData.sIOData_now.au16OutputPWMDuty[i] = iS;
#endif

		}
	}

	// PWM の再設定
#ifdef USE_I2C_PORT_AS_PWM_SPECIAL
	if (!u8PwmSpe_RxMode) {
#endif
		for (i = 0; i < 4; i++) {
			if (sAppData.sIOData_now.au16OutputPWMDuty[i] != 0xFFFF) {
				sTimerPWM[i].u16duty =
						_PWM(sAppData.sIOData_now.au16OutputPWMDuty[i]);
				if (sTimerPWM[i].bStarted) {
					vTimerStart(&sTimerPWM[i]); // DUTY比だけ変更する
				}
			}
		}
#ifdef USE_I2C_PORT_AS_PWM_SPECIAL
	} else {
		if (u8PwmSpe_RxMode >= 1 && u8PwmSpe_TxMode == u8PwmSpe_RxMode) {
			for (i = 0; i < 4; i++) {
				if (sAppData.sIOData_now.au16OutputPWMDuty[i] != 0xFFFF) {
					if (sAppData.sIOData_now.au16OutputPWMDuty[i] >= 1024) {
						sPwmSpe_Context.u16Ct_Total[i] = 0;
					} else {
						if (u8PwmSpe_RxMode == 1) {
							sPwmSpe_Context.u16Ct_Total[i] =
									(sAppData.sIOData_now.au16OutputPWMDuty[i])
											/ 16 + 4;
							sPwmSpe_Context.u16Ct_Active[i] =
									sPwmSpe_Context.u16Ct_Total[i] / 2;
						} else {
							sPwmSpe_Context.u16Ct_Total[i] =
									(sAppData.sIOData_now.au16OutputPWMDuty[i])
											/ 8 + 8;
							sPwmSpe_Context.u16Ct_Active[i] =
									sPwmSpe_Context.u16Ct_Total[i] / 2;
						}
					}
				}
			}
		}
	}
#endif

	/* タイムスタンプ */
	sAppData.sIOData_now.u32RxLastTick = u32TickCount_ms;

	/* UART 出力 */
	if (!sSerCmd.bverbose) {
		if (IS_APPCONF_OPT_REGULAR_PACKET_NO_DISP() && bRegular) {
			; // 通常パケットの場合の出力抑制設定
		} else {
			// 以下のようにペイロードを書き換えて UART 出力
			pRx->auData[0] = pRx->u8Len; // １バイト目はバイト数
			pRx->auData[2] = pRx->u8Lqi; // ３バイト目(もともとは送信元の LogicalID) は LQI

			vSerOutput_ModbusAscii(&sSerStream, u8AppLogicalId,
					SERCMD_ID_INFORM_IO_DATA, pRx->auData, pRx->u8Len);
		}
	}

	/* 子機から速やかなデータ要求 */
	if (IS_LOGICAL_ID_PARENT(sAppData.u8AppLogicalId) && bRespReq) {
		//DBGOUT(0, LB"RESP PKT! (%d)", bRespReq);
		i16TransmitIoData(TX_REPONDING, FALSE);
	}
}

/** @ingroup MASTER
 * IO状態の設定要求を行う UART メッセージから送信されてきたパケットの処理を行います。
 * vReceiveIoData() と大まかな処理は同じですが、PWMの設定に違いが有るので独立した
 * 関数に記述しています。
 *
 * @param pRx 受信したときのデータ
 */
static void vReceiveIoSettingRequest(tsRxDataApp *pRx) {
	int i, j;
	uint8 *p = pRx->auData;

	uint8 u8AppIdentifier = G_OCTET();
	if (u8AppIdentifier != sAppData.u8AppIdentifier)
		return;

	uint8 u8PtclVersion = G_OCTET();
	if (u8PtclVersion != APP_PROTOCOL_VERSION)
		return;

	uint8 u8AppLogicalId = G_OCTET();

	uint32 u32Addr = G_BE_DWORD();

	uint8 u8AppLogicalId_Dest = G_OCTET();

	uint16 u16TimeStamp = G_BE_WORD();

	/* 重複の確認を行う */
	if (bCheckDupPacket(&sDupChk_IoData, u32Addr, u16TimeStamp)) {
		return;
	}

	// 中継の判定 (レベルに達していなければ中継する）
	uint8 u8TxFlag = G_OCTET();

	if (sAppData.u8Mode == E_IO_MODE_ROUTER
			|| (sAppData.u8Mode == E_IO_MODE_CHILD && IS_APPCONF_OPT_ROUTING_CHILD())
	) {
		if (u8TxFlag < sAppData.u8max_hops) {
			// リピータの場合はここで中継の判定
			*(p - 1) = *(p - 1) + 1; // 中継済みフラグのセット
			// 中継する
			i16TransmitRepeat(pRx);
		}

		if (sAppData.u8Mode == E_IO_MODE_ROUTER) {
			return;
		}
	}

	// 親機子機の判定
	if (IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId)) {
		// 子機の場合は、任意の送り主から受けるが、送り先が CHILDREN(120) またはアドレスが一致している事
		if (!(u8AppLogicalId_Dest == sAppData.u8AppLogicalId
				|| u8AppLogicalId_Dest == LOGICAL_ID_CHILDREN)) {
			return;
		}
	} else if (IS_LOGICAL_ID_PARENT(sAppData.u8AppLogicalId)) {
		// 親機の場合は、子機からの送信である事
		if (!(u8AppLogicalId_Dest == LOGICAL_ID_PARENT
				&& IS_LOGICAL_ID_CHILD(u8AppLogicalId))) {
			return;
		}
	} else {
		// それ以外は処理しない
		return;
	}

	/* 書式 */
	uint8 u8Format = G_OCTET();

	if (u8Format == 1) {
		/* BUTTON */
		uint8 u8ButtonState = G_OCTET();
		uint8 u8ButtonChanged = G_OCTET();
		// ポートの値を設定する（変更フラグのあるものだけ）
		for (i = 0, j = 1; i < 4; i++, j <<= 1) {
			if (u8ButtonChanged & j) {
				vDoSet_TrueAsLo(au8PortTbl_DOut[i], u8ButtonState & j);
				sAppData.sIOData_now.au8Output[i] = u8ButtonState & j;
			}
		}

		for (i = 0; i < 4; i++) {
			uint16 u16Duty = G_BE_WORD();
			if (u16Duty <= 1024) {
				sAppData.sIOData_now.au16OutputPWMDuty[i] = u16Duty;
			} else {
				sAppData.sIOData_now.au16OutputPWMDuty[i] = 0xFFFF;
			}
		}

		DBGOUT(1, "RECV:IO REQ: %02x %02x %04x:%04x:%04x:%04x"LB, u8ButtonState,
				u8ButtonChanged, sAppData.sIOData_now.au16OutputPWMDuty[0],
				sAppData.sIOData_now.au16OutputPWMDuty[1],
				sAppData.sIOData_now.au16OutputPWMDuty[2],
				sAppData.sIOData_now.au16OutputPWMDuty[3]);

		// PWM の再設定
		for (i = 0; i < 4; i++) {
			if (sAppData.sIOData_now.au16OutputPWMDuty[i] != 0xFFFF) {
				sTimerPWM[i].u16duty =
						_PWM(sAppData.sIOData_now.au16OutputPWMDuty[i]);
				if (sTimerPWM[i].bStarted)
					vTimerStart(&sTimerPWM[i]); // DUTY比だけ変更する
			}
		}
	}

	/* UART 出力 */
#if 0
	if (!sSerCmd.bverbose) {
		// 以下のようにペイロードを書き換えて UART 出力
		pRx->auData[0] = pRx->u8Len;// １バイト目はバイト数
		pRx->auData[2] = pRx->u8Lqi;// ３バイト目(もともとは送信元の LogicalID) は LQI

		vSerOutput_ModbusAscii(&sSerStream, u8AppLogicalId, SERCMD_ID_INFORM_IO_DATA, pRx->auData, pRx->u8Len);
	}
#endif
}

/** @ingroup MASTER
 * シリアルメッセージの受信処理を行います。分割パケットが全部受信できた時点で UART に出力します。
 * tsRxDataApp *pRx 受信パケット構造体
 */
static void vReceiveSerMsg(tsRxDataApp *pRx) {
	uint8 *p = pRx->auData;

	/* ヘッダ情報の読み取り */
	uint8 u8AppIdentifier = G_OCTET();
	if (u8AppIdentifier != sAppData.u8AppIdentifier) {
		return;
	}
	uint8 u8PtclVersion = G_OCTET();
	if (u8PtclVersion != APP_PROTOCOL_VERSION) {
		return;
	}
	uint8 u8AppLogicalId = G_OCTET();
	(void) u8AppLogicalId;
	uint32 u32Addr = G_BE_DWORD();
	uint8 u8AppLogicalId_Dest = G_OCTET();
	uint16 u16TimeStamp = G_BE_WORD();
	(void) u16TimeStamp;
	uint8 u8TxFlag = G_OCTET();

	/* ここから中身 */
	uint8 u8req = G_OCTET();
	uint8 u8pktnum = G_OCTET();
	uint8 u8idx = G_OCTET();
	uint16 u16offset = G_BE_WORD();
	uint8 u8len = G_OCTET();

	/* 宛先によって処理するか決める */
	if (IS_LOGICAL_ID_REPEATER(sAppData.u8AppLogicalId)
				|| (sAppData.u8Mode == E_IO_MODE_CHILD && IS_APPCONF_OPT_ROUTING_CHILD())) {
			// リピータ機は一旦受け取る。
			;
	} else if (IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId)) {
		if (!(u8AppLogicalId_Dest == sAppData.u8AppLogicalId
				|| u8AppLogicalId_Dest == LOGICAL_ID_CHILDREN)) {
			return;
		}
	} else if (IS_LOGICAL_ID_PARENT(sAppData.u8AppLogicalId)) {
		if (!(u8AppLogicalId_Dest == LOGICAL_ID_PARENT
				&& IS_LOGICAL_ID_CHILD(u8AppLogicalId))) {
			return;
		}
	} else {
		return;
	}

	// 受信パケットのチェック。
	//  - 分割パケットが混在したような場合は、新しい系列で始める。
	//    複数のストリームを同時受信できない！
	bool_t bNew = FALSE;
	if (sSerSeqRx.bWaitComplete) {
		// exceptional check
		if (u32TickCount_ms - sSerSeqRx.u32Tick > 2000) {
			// time out check
			bNew = TRUE;
		}
		if (u8req != sSerSeqRx.u8ReqNum) {
			// different request number is coming.
			bNew = TRUE;
		}
		if (u32Addr != sSerSeqRx.u32SrcAddr) {
			// packet comes from different nodes. (discard this one!)
			bNew = TRUE;
		}
	} else {
		// 待ち状態ではないなら新しい系列
		bNew = TRUE;
	}

	if (bNew) {
		// treat this packet as new, so clean control buffer.
		memset(&sSerSeqRx, 0, sizeof(sSerSeqRx));
	}

	if (!sSerSeqRx.bWaitComplete) {
		// 新しいパケットを受信した

		// 最初にリクエスト番号が適切かどうかチェックする。
		uint32 u32key;
		if (DUPCHK_bFind(&sDupChk_SerMsg, u32Addr, &u32key)) {
			int iPrev = u32key, iNow = u8req;

			if (iNow == iPrev || (uint8) (iNow - iPrev) > 0x80) { //v1.0.4 iPrev == 255 で正しく判定していなかった
			// 最近受信したものより新しくないリクエスト番号の場合は、処理しない
				bNew = FALSE;
			}
		}

		if (bNew) {
			sSerSeqRx.bWaitComplete = TRUE;
			sSerSeqRx.u32Tick = u32TickCount_ms;
			sSerSeqRx.u32SrcAddr = u32Addr;
			sSerSeqRx.u8PktNum = u8pktnum;
			sSerSeqRx.u8ReqNum = u8req;

			sSerSeqRx.u8IdSender = u8AppLogicalId;
			sSerSeqRx.u8IdReceiver = u8AppLogicalId_Dest;

			DUPCHK_vAdd(&sDupChk_SerMsg, sSerSeqRx.u32SrcAddr, u8req);
		}
	}

	if (sSerSeqRx.bWaitComplete) {
		if (u16offset + u8len <= sizeof(au8SerBuffRx)
				&& u8idx < sSerSeqRx.u8PktNum) {
			// check if packet size and offset information is correct,
			// then copy data to buffer and mark it as received.
			if (!sSerSeqRx.bPktStatus[u8idx]) {
				sSerSeqRx.bPktStatus[u8idx] = 1;
				memcpy(au8SerBuffRx + u16offset, p, u8len);
			}

			// the last packet indicates full data length.
			if (u8idx == sSerSeqRx.u8PktNum - 1) {
				sSerSeqRx.u16DataLen = u16offset + u8len;
			}

			// 中継パケットのフラグを格納する
			if (u8TxFlag) {
				if (u8TxFlag > sSerSeqRx.bRelayPacket) {
					sSerSeqRx.bRelayPacket = u8TxFlag;
				}
			}
		}

		// check completion
		int i;
		for (i = 0; i < sSerSeqRx.u8PktNum; i++) {
			if (sSerSeqRx.bPktStatus[i] == 0)
				break;
		}
		if (i == sSerSeqRx.u8PktNum) {
			// 分割パケットが全て届いた！
#if 0
			i16TransmitSerMsgAck(); // アプリケーションACKを返す
#endif

			bool_t bRelay = (sAppData.u8Mode == E_IO_MODE_ROUTER
					|| (sAppData.u8Mode == E_IO_MODE_CHILD && IS_APPCONF_OPT_ROUTING_CHILD()) );

			if (bRelay) {
				// 中継
				if (sSerSeqRx.bRelayPacket < sAppData.u8max_hops) {
					i16TransmitSerMsg(au8SerBuffRx, sSerSeqRx.u16DataLen,
							u32Addr, u8AppLogicalId, au8SerBuffRx[0], sSerSeqRx.bRelayPacket + 1,
							sSerSeqRx.u8ReqNum);
				}
			}

			if (sAppData.u8Mode != E_IO_MODE_ROUTER) { // 中継機は処理しない
#if 0
				// I2C コマンドの実行を禁止する
				if (au8SerBuffRx[1] == SERCMD_ID_I2C_COMMAND) {
					// I2C の処理
#ifndef USE_I2C_PORT_AS_OTHER_FUNCTION
					if (!IS_APPCONF_OPT_PWM_MOVE_PORTS()) {
						vProcessI2CCommand(au8SerBuffRx, sSerSeqRx.u16DataLen, sSerSeqRx.u8IdSender);
					}
#endif
				} else {
#endif
					// 受信データの出力
					vSerOutput_ModbusAscii(&sSerStream,
							// これには困りました...
							// [0] に送信元論理IDを入れておけばとりあえず辻褄はあう
							sSerSeqRx.u8IdSender, // v1.2.1 １バイト目は送り主に変更。他のコマンドなどとの整合性のため。
							au8SerBuffRx[1], au8SerBuffRx + 2,
							sSerSeqRx.u16DataLen - 2);
#if 0
				}
#endif
			}

			memset(&sSerSeqRx, 0, sizeof(sSerSeqRx));
		}
	}
}

/** @ingroup MASTER
 * ADCサンプルの平均化処理を行います。
 * @param pu16k 平均化データを格納した配列
 * @param u8Scale 平均化数スケーラ (2^u8Scale ヶで平均化)
 */
static uint16 u16GetAve(uint16 *pu16k, uint8 u8Scale) {
	int k, kmax;
	uint32 u32Ave = 0;
	kmax = 1 << u8Scale;

	for (k = 0; k < kmax; k++) {
		uint16 v = pu16k[k];

		if (v == 0xFFFF) {
			// 入力NGデータなので処理しない。
			u32Ave = 0xFFFFFFFF;
			break;
		}

		u32Ave += v;
	}
	if (u32Ave != 0xFFFFFFFF) {
		// 平均値が得られる。
		u32Ave >>= u8Scale; // ４で割る
	}
	return u32Ave & 0xFFFF;
}

/** @ingroup MASTER
 * ADC 入力値が有った場合の演算処理を行います。
 *
 * - ADC 値は過去４回の値で平均化して使用します。
 * - sAppData.sIOData_now.u8HistIdx >= 4 で、初回の平均化処理は完了しています。
 * - au16InputADC_LastTx[] に対して変化があるという判定は以下に基づきます。
 *   - 初回データ
 *   - 最後に送信した時と比べて ADC_DELTA_COARSE [mV] 以上の差を検知
 *   - 一定時間経過した後 ADC_DELTA_FINE[mV] 以上の差を検知
 * - 電圧値はHIST_VOLT_COUNT回で平均化します。電圧値は測定した時の負荷状況など
 *   で大きく変わり、変動が大きいためです。
 *
 * @return TRUE:変化判定あり FALSE:変化なし
 */
static bool_t bUpdateAdcValues() {
	bool_t bUpdated = FALSE;
	int i, k;

	// ADC が完了したので内部データに保存する
	sAppData.sIOData_now.au16InputADC[0] =
			(uint16) sAppData.sObjADC.ai16Result[TEH_ADC_IDX_ADC_1];
	sAppData.sIOData_now.au16InputADC[1] =
			(uint16) sAppData.sObjADC.ai16Result[TEH_ADC_IDX_ADC_3];
	sAppData.sIOData_now.au16InputADC[2] =
			(uint16) sAppData.sObjADC.ai16Result[TEH_ADC_IDX_ADC_2];
	sAppData.sIOData_now.au16InputADC[3] =
			(uint16) sAppData.sObjADC.ai16Result[TEH_ADC_IDX_ADC_4];
	sAppData.sIOData_now.i16Temp =
			sAppData.sObjADC.ai16Result[TEH_ADC_IDX_TEMP];
	sAppData.sIOData_now.u16Volt =
			(uint16) sAppData.sObjADC.ai16Result[TEH_ADC_IDX_VOLT];

#if 0
	DBGOUT(0, LB"%04d/%04d/%04d/%04d",
			sAppData.sIOData_now.au16InputADC[0],
			sAppData.sIOData_now.au16InputADC[1],
			sAppData.sIOData_now.au16InputADC[2],
			sAppData.sIOData_now.au16InputADC[3]
	);
#endif

	// 履歴情報を保存する
	//  初期値は 0xFF なので、最初は ++ を実行して 0 からカウントする。
	//  途中で 127 で 63 に巻き戻す。4以上なら初回実行済みである判定のため。
	sAppData.sIOData_now.u8HistIdx++;
	if (sAppData.sIOData_now.u8HistIdx >= 127) {
		sAppData.sIOData_now.u8HistIdx = 63;
	}

	k = sAppData.sIOData_now.u8HistIdx & 0x3;
	for (i = 0; i < 4; i++) {
		if (IS_ADC_RANGE_OVER(sAppData.sIOData_now.au16InputADC[i])
				|| IS_APPCONF_OPT_DISABLE_ADC() // ADC無効化のフラグ
				) {
			sAppData.sIOData_now.au16InputADC[i] = 0xFFFF;
			sAppData.sIOData_now.au16InputADC_History[i][k] = 0xFFFF;
		} else {
			sAppData.sIOData_now.au16InputADC_History[i][k] =
					sAppData.sIOData_now.au16InputADC[i];
		}
	}
	sAppData.sIOData_now.au16Volt_History[sAppData.sIOData_now.u8HistIdx
			& (HIST_VOLT_COUNT - 1)] = sAppData.sIOData_now.u16Volt;

	// 電圧を先に平均化しておく
	if (sAppData.sIOData_now.u8HistIdx >= HIST_VOLT_COUNT) {
		sAppData.sIOData_now.u16Volt = u16GetAve(
				sAppData.sIOData_now.au16Volt_History, HIST_VOLT_SCALE);
	}

	// ADC1...4 の平均化処理
	for (i = 0; i < 4; i++) {
		// 過去４サンプルの平均値を保存する
		if (sAppData.sIOData_now.u8HistIdx >= 4) {
			sAppData.sIOData_now.au16InputADC[i] = u16GetAve(
					sAppData.sIOData_now.au16InputADC_History[i], 2);
		}

		// 判定0:そもそもADC値が適正でないなら処理しない。
		if (sAppData.sIOData_now.au16InputADC[i] == 0xFFFF) {
			continue;
		}

		// 判定1：送信前データが存在しない場合。
		if (sAppData.sIOData_now.au16InputADC_LastTx[i] == 0xFFFF) {
			continue;
		}

		// 判定2：最終送信データとの差(粗)
		int iDelta = abs(
				(int) sAppData.sIOData_now.au16InputADC_LastTx[i]
						- (int) sAppData.sIOData_now.au16InputADC[i]);
		if (iDelta > ADC_DELTA_COARSE) {
			bUpdated = TRUE;
			continue;
		}

		// 判定3:最終送信データとの差(細), 経過時間が 300ms 以上
		if (iDelta > ADC_DELTA_FINE
				&& (u32TickCount_ms - sAppData.sIOData_now.u32TxLastTick
						> ADC_TIMEOUT_TO_FINE_CHECK)) {
			bUpdated = TRUE;
			continue;
		}
	}

#ifdef USE_PWM4_AS_BAT_MONITOR
	if (sAppData.sIOData_now.u16Volt != 0xFFFF) {
		if (sAppData.sIOData_now.u16Volt <= 2300) {
			sAppData.sIOData_now.au16InputADC[3] = 0;
		} else if (sAppData.sIOData_now.u16Volt <= 2500) {
			sAppData.sIOData_now.au16InputADC[3] = sAppData.sIOData_now.u16Volt / 4;
		} else {
			sAppData.sIOData_now.au16InputADC[3] = sAppData.sIOData_now.u16Volt / 2;
		}
	} else {
		sAppData.sIOData_now.au16InputADC[3] = 0xFFFF;
	}
#endif

	return bUpdated;
}

/** @ingroup FLASH
 * フラッシュ設定構造体をデフォルトに巻き戻します。
 * - ここでシステムのデフォルト値を決めています。
 *
 * @param p 構造体へのアドレス
 */
static void vConfig_SetDefaults(tsFlashApp *p) {
	p->u32appid = APP_ID;
	p->u32chmask = CHMASK;
	p->u8ch = CHANNEL;
	p->u8pow = 3;

	p->u8role = E_APPCONF_ROLE_MAC_NODE;
	p->u8layer = 1;

	p->u8room = 60;
	p->u8sensortype = 0x4;
	//p->u16SleepDur_ms = 1000;
	p->u16SleepDur_s = (MODE7_SLEEP_DUR_ms / 1000);
	p->u8Fps = 32;

	p->u32PWM_Hz[0] = 1000;
	p->u32PWM_Hz[1] = 1000;
	p->u32PWM_Hz[2] = 1000;
	p->u32PWM_Hz[3] = 1000;

	p->u32baud_safe = UART_BAUD_SAFE;
	p->u8parity = 0; // none

#ifdef USE_MONOSTICK
	p->u32Opt = E_APPCONF_OPT_DISABLE_ADC;  // ToCoStick では AI 入力は未接続なので無効化しておく
	p->u8id = 121; // ToCoStick では親機をデフォルトにする
#else
	p->u8id = 0;
	p->u32Opt = 0;
#endif
}

/** @ingroup FLASH
 * フラッシュ設定構造体を全て未設定状態に設定します。未設定状態は全て 0xFF と
 * なり、逆にいえば 0xFF, 0xFFFF といった設定値を格納できます。
 *
 * @param p 構造体へのアドレス
 */
static void vConfig_UnSetAll(tsFlashApp *p) {
	memset(p, 0xFF, sizeof(tsFlashApp));
}

/** @ingroup FLASH
 * フラッシュ(またはEEPROM)に保存し、モジュールをリセットする
 */
static void vConfig_SaveAndReset() {
	tsFlash sFlash = sAppData.sFlash;

	if (sAppData.sConfig_UnSaved.u32appid != 0xFFFFFFFF) {
		sFlash.sData.u32appid = sAppData.sConfig_UnSaved.u32appid;
	}
	if (sAppData.sConfig_UnSaved.u32chmask != 0xFFFFFFFF) {
		sFlash.sData.u32chmask = sAppData.sConfig_UnSaved.u32chmask;
	}
	if (sAppData.sConfig_UnSaved.u8id != 0xFF) {
		sFlash.sData.u8id = sAppData.sConfig_UnSaved.u8id;
	}
	if (sAppData.sConfig_UnSaved.u8ch != 0xFF) {
		sFlash.sData.u8ch = sAppData.sConfig_UnSaved.u8ch;
	}
	if (sAppData.sConfig_UnSaved.u8pow != 0xFF) {
		sFlash.sData.u8pow = sAppData.sConfig_UnSaved.u8pow;
	}
	if (sAppData.sConfig_UnSaved.u8layer != 0xFF) {
		sFlash.sData.u8layer = sAppData.sConfig_UnSaved.u8layer;
	}
	if (sAppData.sConfig_UnSaved.u8role != 0xFF) {
		sFlash.sData.u8role = sAppData.sConfig_UnSaved.u8role;
	}
	if (sAppData.sConfig_UnSaved.u8room != 0xFF) {
		sFlash.sData.u8room = sAppData.sConfig_UnSaved.u8room;
	}
	if (sAppData.sConfig_UnSaved.u8sensortype != 0xFF) {
		sFlash.sData.u8sensortype = sAppData.sConfig_UnSaved.u8sensortype;
	}
/*
	if (sAppData.sConfig_UnSaved.u16SleepDur_ms != 0xFFFF) {
		sFlash.sData.u16SleepDur_ms = sAppData.sConfig_UnSaved.u16SleepDur_ms;
	}
*/
	if (sAppData.sConfig_UnSaved.u16SleepDur_s != 0xFFFF) {
		sFlash.sData.u16SleepDur_s = sAppData.sConfig_UnSaved.u16SleepDur_s;
	}
	if (sAppData.sConfig_UnSaved.u8Fps != 0xFF) {
		sFlash.sData.u8Fps = sAppData.sConfig_UnSaved.u8Fps;
	}
	{ int i;
		for (i = 0; i < 4; i++) {
			if (sAppData.sConfig_UnSaved.u32PWM_Hz[i] != 0xFFFFFFFF) {
				sFlash.sData.u32PWM_Hz[i] = sAppData.sConfig_UnSaved.u32PWM_Hz[i];
			}
		}
	}
	if (sAppData.sConfig_UnSaved.u32baud_safe != 0xFFFFFFFF) {
		sFlash.sData.u32baud_safe = sAppData.sConfig_UnSaved.u32baud_safe;
	}
	if (sAppData.sConfig_UnSaved.u8parity != 0xFF) {
		sFlash.sData.u8parity = sAppData.sConfig_UnSaved.u8parity;
	}
	if (sAppData.sConfig_UnSaved.u32Opt != 0xFFFFFFFF) {
		sFlash.sData.u32Opt = sAppData.sConfig_UnSaved.u32Opt;
	}

	sFlash.sData.u32appkey = APP_ID;
	sFlash.sData.u32ver = VERSION_U32;

	bool_t bRet = bFlash_Write(&sFlash, FLASH_SECTOR_NUMBER - 1, 0);
	V_PRINT("!INF FlashWrite %s"LB, bRet ? "Success" : "Failed");
	vConfig_UnSetAll(&sAppData.sConfig_UnSaved);
	vWait(100000);

	V_PRINT("!INF RESET SYSTEM...");
	vWait(1000000);
	vAHI_SwReset();
}

/** @ingroup MASTER
 * スリープ状態に遷移します。
 *
 * @param u32SleepDur_ms スリープ時間[ms]
 * @param bPeriodic TRUE:前回の起床時間から次のウェイクアップタイミングを計る
 * @param bDeep TRUE:RAM OFF スリープ
 */
static void vSleep(uint32 u32SleepDur_ms, bool_t bPeriodic, bool_t bDeep) {
	// print message.

	if (sAppData.u8Mode != E_IO_MODE_CHILD_SLP_10SEC) {
		// IO 情報の保存
		memcpy(&sAppData.sIOData_reserve, &sAppData.sIOData_now, sizeof(tsIOData));

		// PWM の停止
		int i;
		i = 0;
		for (; i < 4; i++) {
			vTimerStop(&sTimerPWM[i]);
			vTimerDisable(&sTimerPWM[i]);
		}
	}

	// PWM (DO0/1ピン) の後始末 (出力設定にして、両方Hiに設定する）
	//   この手続を踏まないとハングアップする ;-(
	bAHI_DoEnableOutputs(TRUE);
	vAHI_DoSetDataOut(0x3, 0x0);

	// stop interrupt source, if interrupt source is still running.
	vAHI_DioInterruptEnable(0, u32PortInputMask); // 割り込みの解除）

#if defined(I2C_SMBUS)
	// I2C をオフにする
	vAHI_SiMasterDisable();
	vPortDisablePullup(PORT_I2C_SCK);
	vPortDisablePullup(PORT_I2C_SDA);
#endif
#if defined(I2C_PORT)
	vI2C_Init();
#endif
	// UART をオフにする
	vAHI_UartDisable(UART_PORT_MASTER);
	vTimerStop(&sTimerApp);
	vTimerDisable(&sTimerApp);

	// 解放したポートは入力に設定する
	uint32 mask = 0
#if defined(I2C_SMBUS) || defined(I2C_PORT)
		| (1UL << PORT_I2C_SCK)
		| (1UL << PORT_I2C_SDA);
#endif
	u32PortInputMask |= (1UL << PORT_UART_TX)
		| (1UL << PORT_UART_RX)
		| (1UL << PORT_OUT1)
		| (1UL << PORT_OUT2)
		| (1UL << PORT_OUT3)
		| (1UL << PORT_OUT4);

	// set UART Rx port as interrupt source
	vAHI_DioSetDirection(u32PortInputMask | mask, 0); // set as input

	// pullup を有効にする(SCK, SDAは無効)
	vAHI_DoSetPullup(u32PortInputMask, mask);

	(void) u32AHI_DioInterruptStatus(); // clear interrupt register
	//vAHI_DioWakeEnable(u32PortInputMask, 0); // also use as DIO WAKE SOURCE
	//vAHI_DioWakeEdge(0, u32PortInputMask); // 割り込みエッジ（立下りに設定）
	vAHI_DioWakeEnable(0, u32PortInputMask); // DISABLE DIO WAKE SOURCE

	// wake up using wakeup timer as well.
	ToCoNet_vSleep(E_AHI_WAKE_TIMER_0, u32SleepDur_ms, bPeriodic, bDeep); // PERIODIC RAM OFF SLEEP USING WK0
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
