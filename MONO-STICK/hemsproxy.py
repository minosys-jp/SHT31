#!/usr/bin/python
# -*- coding: utf-8 -*-

import json, datetime, time, struct, binascii, serial, urllib

def readSensorType(data):
	if (len(data) == 0):
		return False
	if (data[0] != ":"):
		return False
	dt = data[1:7]

	# 先頭をバイトに変換する
	ss = struct.Struct(">BBB")
	dt = binascii.unhexlify(dt.rstrip())
	parsed = ss.unpack(dt)

	if (parsed[2] == 0x01):
		# SHT11 temperature/humidity sensor
		return "sht11";
	elif (parsed[2] == 0x02):
		# MAX31855 K thurmocouple temperature sensor
		return "max31855"
	elif (parsed[2] == 0x03):
		# AM2302 temperature/humidity sensor
		return "am2302"
	elif (parsed[2] == 0x04):
		# SHT31 temperature/humidity sensor
		return "sht31";
	else:
		return False

def parseSHT11(data):
	# 先頭の : を取り除く
	if data[0] != ":":
		return False
	data = data[1:]

	# バイトデータに変換する
	ss = struct.Struct(">BBBBBBHHB")
	data = binascii.unhexlify(data.rstrip())
	parsed = ss.unpack(data);

	# 温度データを温度に変換
	temp = -39.6 + 0.01 * parsed[6]

	# 湿度データを湿度に変換
	hum = -2.0468 + 0.0367 * parsed[7] + -1.5955e-6 * parsed[7] * parsed[7]
	hum = (temp - 25.0) * (0.01 + 0.00008 * parsed[7]) + hum;

	# 現在時刻
	now = datetime.datetime.now()
	current = now.strftime("%Y-%m-%d %H:%M:%S")

	# 結果を返す
	result = {
		"type" : "sht11",
		"datetime" : current,
		"current" : int(time.mktime(now.timetuple())),
		"from" : parsed[0],
		"presence" : parsed[4],
		"status" : parsed[5],
		"temperature" : temp,
		"humidity" : hum
	}
	return result;

def parseSHT31(data):
	# 先頭の : を取り除く
	if data[0] != ":":
		return False
	data = data[1:]

	# バイトデータに変換する
	# [0]: 送信元論理ID
	# [1]: 0x87 固定
	# [2]: センサータイプ(0x04固定)
	# [3]: 部屋番号
	# [4]: ステータス
	# [5]: 温度
	# [6]: 湿度
	# [7]: チェックサム(足し算の2の補数)
	ss = struct.Struct(">BBBBBHHB")
	data = binascii.unhexlify(data.rstrip())
	parsed = ss.unpack(data);

	# 温度データを温度に変換
	temp = -45.0 + 175.0 * parsed[5] / (2 ** 16 - 1)

	# 湿度データを湿度に変換
	hum = 100.0 * parsed[6] / (2 ** 16 - 1)

	# 現在時刻
	now = datetime.datetime.now()
	current = now.strftime("%Y-%m-%d %H:%M:%S")

	# 結果を返す
	result = {
		"type" : "sht31",
		"datetime" : current,
		"current" : int(time.mktime(now.timetuple())),
		"from" : parsed[3],	# v1.3.1 への対応
		"presence" : 128,
		"status" : parsed[4],
		"temperature" : temp,
		"humidity" : hum
	}
	return result;

def parseMAX31855(data):
	# 先頭の : を取り除く
	if data[0] != ":":
		return False
	data = data[1:]

	# バイトデータに変換する
	ss = struct.Struct(">BBBBlB")
	data = binascii.unhexlify(data.rstrip())
	parsed = ss.unpack(data);

	# 温度データを温度に変換
	temp = (parsed[4] >> 18) * 0.25

	# error ビットを解釈
	error = parsed[4] & 0x07

	# 現在時刻
	now = datetime.datetime.now()
	current = now.strftime("%Y-%m-%d %H:%M:%S")

	# 結果を返す
	result = {
		"type" : "max31855",
		"datetime" : current,
		"current" : int(time.mktime(now.timetuple())),
		"from" : parsed[0],
		"presence" : 128,
		"status" : error,
		"temperature" : temp,
		"humidity": 0,
	}
	return result;

def parseAM2302(data):
	# 先頭の : を取り除く
	if data[0] != ":":
		return False
	data = data[1:]

	# バイトデータに変換する
	ss = struct.Struct(">BBBBBHHB")
	data = binascii.unhexlify(data.rstrip())
	parsed = ss.unpack(data);

	# 温度データを温度に変換
	if (parsed[5] & 0x8000):
		temp = ((parsed[5] & 0x7fff) / 10.0) * -1.0
	else:
		temp = parsed[5] / 10.0

	# 湿度データを湿度に変換
	hum = parsed[6] / 10.0

	# センサー状態
	status = parsed[4]

	# 現在時刻
	now = datetime.datetime.now()
	current = now.strftime("%Y-%m-%d %H:%M:%S")

	# 結果を返す
	result = {
		"type" : "am2302",
		"datetime" : current,
		"current" : int(time.mktime(now.timetuple())),
		"from" : parsed[0],
		"status" : status,
		"temperature" : temp,
		"humidity" : hum
	}
	return result;

def dataPoster(js):
	url = urllib.URLopener();
	#appenddb.php の URL を指定してください
	url.open("http://<your server>/appenddb.php", js);
	url.close()

# 前回読み取ったシステムタイム値
lasttime = time.time()
queue = [];

# /dev/ttyUSB0 を開く
s = serial.Serial(port = "/dev/ttyUSB0", baudrate = 115200, timeout = 30)

while True:
	# 1行読み取る
	data = s.readline()

	# sensor 種別を確認する
	type = readSensorType(data);

	# 解釈する
	parsed = False
	if (type == "sht11"):
		parsed = parseSHT11(data)
	elif (type == "max31855"):
		parsed = parseMAX31855(data)
	elif (type == "am2302"):
		parsed = parseAM2302(data)
	elif (type == "sht31"):
		parsed = parseSHT31(data)

	# 書き込みファイルを開く
	if parsed != False:
		f = open("/tmp/" + type + ".txt", "a")
		js = json.dumps(parsed, sort_keys=True)
		f.write(js)
		f.write("\n")
		f.close()
		queue.append(js)

	# タイムアウトしたら送信する
	ctime = time.time()
	if (ctime - lasttime >= 60):
		if (len(queue) > 0):
			dump = "[" + ",".join(queue) + "]"
			dataPoster(dump)
		queue = []
		lasttime = ctime
		
else:
	True

# COM を閉じる
s.close()
