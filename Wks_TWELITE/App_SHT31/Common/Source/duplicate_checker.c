/* Copyright (C) 2017 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-*J,*E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT).                                                   */

#include <string.h>
#include "duplicate_checker.h"

/** @ingroup DUPCHK
 * 重複チェックの初期化
 * @param p 管理構造体
 */
void DUPCHK_vInit(tsDupChk_Context *p) {
	memset(p, 0, sizeof(tsDupChk_Context));
}


/** @ingroup DUPCHK
 * リストの探索とタイムアウト処理を行う。
 *
 * @param p 管理構造体
 * @param u32Addr 探索アドレス。０の場合はタイムアウト処理のみ行う
 * @param pu32Key キー情報
 * @return
 */
bool_t DUPCHK_bFind(tsDupChk_Context *p, uint32 u32Addr, uint32 *pu32Key) {
	//bool_t bRet = FALSE;
	int i;
	for(i = 0; i < DUPCHK_MAX_HISTORY; i++) {
		if (u32TickCount_ms - p->au32ScanListTick[i] > DUPCHK_TIMEOUT) {
			p->au32ScanListTick[i] = 0;
			p->au32ScanListAddr[i] = 0;
		} else {
			if (u32Addr && (u32Addr == p->au32ScanListAddr[i])) {
				*pu32Key = p->au32ScanListKey[i];
				return TRUE;
			}
		}
	}
	return FALSE;
}

/** @ingroup DUPCHK
 * リストへの追加を行う。
 *
 * @param p 管理構造体
 * @param u32Addr 探索アドレス
 * @param u32Key キー情報
 * */
void DUPCHK_vAdd(tsDupChk_Context *p, uint32 u32Addr, uint32 u32Key) {
	int i, idxPrimary = DUPCHK_MAX_HISTORY, idxSecondary = DUPCHK_MAX_HISTORY, idxOldest = DUPCHK_MAX_HISTORY;
	uint32 u32TickDelta = 0;
	for(i = 0; i < DUPCHK_MAX_HISTORY; i++) {
		if (p->au32ScanListAddr[i] == u32Addr) {
			// 同じアドレスが見つかったら、こちらに登録する（アルゴリズム上無いはずだが）
			idxPrimary = i;
		}
		if (p->au32ScanListAddr[i] == 0) {
			// 空のエントリ
			idxSecondary = i;
		}

		if (p->au32ScanListAddr[i]) {
			// 一番古いエントリ
			uint32 u32diff = u32TickCount_ms - p->au32ScanListTick[i];
			if (u32diff >= u32TickDelta) {
				idxOldest = i;
				u32TickDelta = u32diff;
			}
		}
	}

	if (idxPrimary < DUPCHK_MAX_HISTORY) {
		i = idxPrimary;
	} else if (idxSecondary < DUPCHK_MAX_HISTORY){
		i = idxSecondary;
	} else {
		i = idxOldest;
	}

	if (i < DUPCHK_MAX_HISTORY) {
		p->au32ScanListAddr[i] = u32Addr;
		p->au32ScanListTick[i] = u32TickCount_ms;
		p->au32ScanListKey[i] = u32Key;
	}
}

