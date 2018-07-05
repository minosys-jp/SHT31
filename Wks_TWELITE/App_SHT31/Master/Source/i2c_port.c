#include "i2c_port.h"
#include "utils.h"
#include "ToCoNet.h"
#include "common.h"

/**
 * I2C バスを初期化する
 */
void vI2C_Init() {
    vPortAsInput(PORT_I2C_SCK);
    vPortAsInput(PORT_I2C_SDA);
    vPortDisablePullup(PORT_I2C_SCK);
    vPortDisablePullup(PORT_I2C_SDA);
}

/**
 * I2C バスをリセットする
 */
void vI2C_BusReset() {
    int i;
    vPortAsInput(PORT_I2C_SDA);
    for (i = 0; i < 10; ++i) {
        vPortSetLo(PORT_I2C_SCK);
        vPortAsOutput(PORT_I2C_SCK);
        vWait(I2C_WAIT);
        vPortAsInput(PORT_I2C_SCK);
        vWait(I2C_WAIT);
    }
}

/**
 * スタートビットを送信する
 */
static void vI2C_StartBit() {
    vPortAsInput(PORT_I2C_SCK);
    vPortAsInput(PORT_I2C_SDA);
    vWait(I2C_WAIT);

    vPortSetLo(PORT_I2C_SDA);
    vPortAsOutput(PORT_I2C_SDA);
    vWait(I2C_WAIT);

    vPortSetLo(PORT_I2C_SCK);
    vPortAsOutput(PORT_I2C_SCK);
    vWait(I2C_WAIT);
}

/**
 * ストップビットを送信する
 */
static void vI2C_StopBit() {
    vPortSetLo(PORT_I2C_SDA);
    vPortAsOutput(PORT_I2C_SDA);
    vPortAsInput(PORT_I2C_SCK);
    vWait(I2C_WAIT);

    vPortAsInput(PORT_I2C_SDA);
    vWait(I2C_WAIT);
}

/**
 * 8ビットデータを送信する
 * @param   8ビットデータ
 * @return  TRUE: ACK, FALSE: NACK
 */
static bool_t bI2C_Write8(uint8 data) {
    int i;
    volatile bool_t bIO;

    for (i = 0; i < 8; ++i, data <<= 1) {
        if (data & 0x80) {
            vPortAsInput(PORT_I2C_SDA);
        } else {
            vPortSetLo(PORT_I2C_SDA);
            vPortAsOutput(PORT_I2C_SDA);
        }
        vPortAsInput(PORT_I2C_SCK);
        vWait(I2C_WAIT);
        vPortSetLo(PORT_I2C_SCK);
        vPortAsOutput(PORT_I2C_SCK);
        vWait(I2C_WAIT);
    }

    // ACK/NACK 受信
    vPortAsInput(PORT_I2C_SDA);
    vPortAsInput(PORT_I2C_SCK);
    vWait(I2C_WAIT);
    bIO = bPortRead(PORT_I2C_SDA);
    vPortSetLo(PORT_I2C_SCK);
    vPortAsOutput(PORT_I2C_SCK);
    vWait(I2C_WAIT);
    return bIO;
}

/**
 * 8ビットデータを受信する
 * @param   pdata 受信データの格納先
 * @param   ack ホストが送信する ACK/NACK
 */
static void vI2C_Read8(uint8 *pdata, bool_t ack) {
    int i;
    volatile bool_t bIO;
    volatile uint8 result = 0x00;

    vPortAsInput(PORT_I2C_SDA);
    for (i = 0; i < 8; ++i) {
        vPortAsInput(PORT_I2C_SCK);
        vWait(I2C_WAIT);
        bIO = bPortRead(PORT_I2C_SDA);
        vPortSetLo(PORT_I2C_SCK);
        vPortAsOutput(PORT_I2C_SCK);
        vWait(I2C_WAIT);
        result <<= 1;
        result |= bIO ? 0 : 1;
    }

    // ACK/NACK 送信
    if (ack) {
        vPortSetLo(PORT_I2C_SDA);
        vPortAsOutput(PORT_I2C_SDA);
    } else {
        vPortAsInput(PORT_I2C_SDA);
    }
    vPortAsInput(PORT_I2C_SCK);
    vWait(I2C_WAIT);
    vPortSetLo(PORT_I2C_SCK);
    vPortAsOutput(PORT_I2C_SCK);
    vWait(I2C_WAIT);
    *pdata = result;
}

/**
 * SCK ストレッチを検出する
 * @return TRUE: slave ready, FALSE: slave busy
 */
bool_t bI2C_SlaveReady() {
    return bPortRead(PORT_I2C_SCK) ? FALSE: TRUE;
}

/**
 * データ列を I2C 送信する
 * @param   addr    宛先 I2C アドレス
 * @param   len     データ長
 * @param   pbuf    送信データ
 * @return  TRUE: ack, FALSE: nack
 */
bool_t bI2C_WriteData(uint8 addr, uint8 len, uint8 *pbuf) {
    bool_t ack;

    vI2C_StartBit();
    do {
        addr <<= 1; // write command
        ack = bI2C_Write8(addr);
        while (ack && len-- > 0) {
            ack = bI2C_Write8(*(pbuf++));
        }
    } while (0);
    vI2C_StopBit();
    return ack;
}

/**
 * データ列を受信する
 * @param   addr    送信元 I2C  アドレス
 * @param   len     受信データ長
 * @param   pbuf    受信データ格納先
 * @return  TRUE: ack, FALSE: nack
 */
bool_t bI2C_ReadData(uint8 addr, uint8 len, uint8 *pbuf) {
    bool_t ack;

    vI2C_StartBit();
    do {
        addr <<= 1; // read command
        addr |= 1;
        ack = bI2C_Write8(addr);
        if (!ack) break;
        while (len-- > 0) {
            vI2C_Read8(pbuf, (len == 0)? FALSE : TRUE);
            ++pbuf;
        }
    } while (0);
    vI2C_StopBit();
    return ack;
}
