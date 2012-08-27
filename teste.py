from xbee import ZigBee
import serial
from struct import unpack, pack
from datetime import datetime

ser = serial.Serial('/dev/tty.usbserial-A600ezSM', 38400)

xbee = ZigBee(ser, escaped=True)

addr = "\x00\x13\xA2\x00\x40\x6F\xBA\xE5"

def decodeFloat(var):
	text = ""
	for i in range(0, len(var)):
		text += var[i]
	return unpack('f', text)[0]

def encodeLong(var):
	var = unpack('<L', pack('<l', var))[0]
	v1 = var & 0xff
	v2 = (var >> 8) & 0xff
	v3 = (var >> 16) & 0xff
	v4 = (var >> 24) & 0xff
	return chr(v1) + chr(v2) + chr(v3) + chr(v4)

def encodeInt(var):
	#var = unpack('<h', pack('<h', var))[0]
	v1 = var & 0xff
	v2 = (var >> 8) & 0xff
	return chr(v1) + chr(v2)


#xbee.send("tx", frame_id="\x01", dest_addr_long=addr, dest_addr="\xff\xfe", data="\x53\x00" + encodeLong(78100))
#xbee.send("tx", frame_id="\x01", dest_addr_long=addr, dest_addr="\xff\xfe", data="\x51" + encodeInt(10))
#xbee.send("tx", frame_id="\x01", dest_addr_long=addr, dest_addr="\xff\xfe", data="\x52\x00\x13\xa2\x00\x40\x66\x5d\xb3")
xbee.send("tx", frame_id="\x01", dest_addr_long=addr, dest_addr="\xff\xfe", data="\x01")
while True:
	try:
		response = xbee.wait_read_frame()
		print response
	except KeyboardInterrupt:
		break
	if (response.get("rf_data")):
		print str(datetime.now())
		print str(decodeFloat(response.get("rf_data")[0] + response.get("rf_data")[1] + response.get("rf_data")[2] + response.get("rf_data")[3])) + "mb"
		print str(decodeFloat(response.get("rf_data")[8] + response.get("rf_data")[9] + response.get("rf_data")[10] + response.get("rf_data")[11])) + "%"
		print str(decodeFloat(response.get("rf_data")[4] + response.get("rf_data")[5] + response.get("rf_data")[6] + response.get("rf_data")[7])) + "C"
		print str(decodeFloat(response.get("rf_data")[12] + response.get("rf_data")[13] + response.get("rf_data")[14] + response.get("rf_data")[15])) + "C"
		break
		
        
ser.close()
