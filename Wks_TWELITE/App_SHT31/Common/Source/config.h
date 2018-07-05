/* Copyright (C) 2017 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-*J,*E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT).                                                   */

#ifndef  CONFIG_H_INCLUDED
#define  CONFIG_H_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <AppHardwareApi.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/* Serial Configuration */
#define UART_BAUD			115200UL //!< UART のボーレート（デフォルト）
#define UART_BAUD_SAFE		38400UL //!< UART のボーレート（他の設定）
#define UART_PARITY_ENABLE	E_AHI_UART_PARITY_DISABLE //!< パリティは none
#define UART_PARITY_TYPE 	E_AHI_UART_EVEN_PARITY //!< E_AHI_UART_PARITY_ENABLE 時の指定
#define UART_STOPBITS 		E_AHI_UART_1_STOP_BIT //!< ストップビットは１

/* Specify which serial port to use when outputting debug information */
#define UART_PORT_MASTER    E_AHI_UART_0 //!< UARTポートの指定

/* Specify the PAN ID and CHANNEL to be used by tags, readers and gateway */
#define APP_ID_ASSIGNED     0x67720102 //!< アプリケーションID。同じIDでないと通信しない。
#define APP_ID              APP_ID_ASSIGNED //!< アプリケーションID。同じIDでないと通信しない。
//#define APP_ID              0x67720202 //!< アプリケーションID。同じIDでないと通信しない。

#if APP_ID != APP_ID_ASSIGNED
# warning "Custom APP_ID used"
#endif

//#define CHANNEL             17
//#define CHMASK              ((1UL << 11) | (1UL << 17) | (1UL << 25))
#define CHANNEL 18 //!< 使用するチャネル
#define CHMASK (1UL << CHANNEL) //!< チャネルマスク（３つまで指定する事が出来る）

// Coordinator specific settings
#define PARENT_ADDR     	0x8001

// SERIAL BUFFERS
#define SERCMD_SER_PKTLEN 80 //!< シリアルメッセージのデータ部の最大バイト数
#define SERCMD_SER_PKTNUM 13 //!< シリアルメッセージの最大送信パケット数
#define SERCMD_MAXPAYLOAD (SERCMD_SER_PKTLEN*SERCMD_SER_PKTNUM) //!< シリアルメッセージのバッファサイズ

// ADC
#define ADC_MAC_MV 2470 //!< ADCの最大電圧（実際は bandgap 電圧によって変わる）
#define ADC_MAX_THRES (2050) //!< ADC の振り切れ判定値
#define IS_ADC_RANGE_OVER(c) ((c) >= ADC_MAX_THRES) //!< ADCの振り切れ判定
#define ADC_DELTA_COARSE 64 //!< ADCの変化を判定する幅（粗）  0-2V スケールを想定し、その 5%=100mV
#define ADC_DELTA_FINE (ADC_DELTA_COARSE/2) //!< ADCの変化を判定する幅（細）
#define ADC_TIMEOUT_TO_FINE_CHECK 300 //!< ADCの変化判定を細かくする時間判定

#undef IGNORE_ADC_CHAGNE //!< ADC の変化判定を無効にする（デバッグ用）
#ifdef IGNORE_ADC_CHAGNE
# warning "IGNORE_ADC_CHANGE is defined... Undef this for official release."
#endif

// スリープ間隔
#define MODE4_SLEEP_DUR_ms 1000UL
#define MODE7_SLEEP_DUR_ms 60000UL

/**
 * ボタン押し下げ時連続送信モード
 *
 * - DI=G 検知時に連続送信し、DI=H になった後も１秒間(ON_PRESS_TRANSMIT_KEEP_TX_ms)継続送信する。
 * - 受信側は、500ms(ON_PRESS_TRANSMIT_RESET_ms) 無線電波を受信しなかった場合、DO=H に戻す
 */
#define ON_PRESS_TRANSMIT

/**
 * ボタン押し下げ時連続送信モード時の、無受信時に DO=H に戻すまでの時間[ms]
 */
#define ON_PRESS_TRANSMIT_RESET_ms 500

/**
 * ボタン押し下げ時連続送信モード時の、DI=H に戻った後に継続送信する時間 [ms]
 */
#define ON_PRESS_TRANSMIT_KEEP_TX_ms 1000

/**
 * 低レイテンシモード時で割り込みタイミングから、再度送信する
 */
#define LOW_LATENCY_DELAYED_TRANSMIT_COUNTER 7

/**
 * I2C をチップ内臓機能で実現するか、ソフトウェアハンドシェアリングで
 * 実現するか決める
 */
#undef I2C_SMBUS
#define I2C_PORT

/**
 * ToCoStick 用
 *   - デフォルトで親機になる
 *   - オプションビット 0x00000020 を設定する
 */
#ifdef USE_MONOSTICK
# warning "USE_MONOSTICK is defined... Special Build."
#endif

/**
 * LCDモジュールの対応
 */
#define USE_I2C_ACM1620 //!< ACM1620 をサポートする
#define USE_I2C_AQM0802A //!< AQM0802A をサポートする

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
typedef struct {
	uint8 u8IdSender;
	uint8 u8IdReceiver;
	bool_t bPktStatus[SERCMD_SER_PKTNUM];
	uint8 u8PktNum;
	uint16 u16DataLen;
	uint8 u8Seq, u8SeqNext;
	uint8 u8ReqNum;
	bool_t bWaitComplete;
	bool_t bRelayPacket; //!< 中継パケットが含まれる？この場合再中継しない。

	uint32 u32Tick;
	uint32 u32SrcAddr;
} tsSerSeq;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif  /* CONFIG_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
