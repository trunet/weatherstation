#include <EEPROM.h>

#include <XBee.h>

#include <Wire.h>

#include <BMP085.h>
#include <DHT22.h>

#define uint  unsigned int
#define ulong unsigned long
#define MAX_LINE_SIZE 42

#define ISCONFVALUE 82 // Value to check if calibration is saved on EEPROM

#define EEPROM_ADDR_SEND_EACH 1
#define DEFAULT_SEND_EACH 60 //seconds

#define EEPROM_ADDR_SEND_ADDR 3
#define DEFAULT_SEND_ADDR_MSB 0x0013a200 // PC XBEE ADDRESS MSB
#define DEFAULT_SEND_ADDR_LSB 0x40665db3 // PC XBEE ADDRESS LSB

#define EEPROM_ADDR_QFE_QNH 11
#define DEFAULT_QFE_QNH 1 // default to QFE(altitude)

#define EEPROM_ADDR_CALIB 12
#define DEFAULT_CALIB 78100 // 781.00 meters

// XBee
XBee xbee = XBee();

uint8_t payload[MAX_LINE_SIZE] = {};
uint8_t payloadPointer = 0;

//XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x40665db3);
//ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
// Response
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();

unsigned long lastPacket, lastCapture;

BMP085 dps = BMP085(); //I2C: SCL on A5, SDA on A4

#define DHT22_PIN 5 //D5
DHT22 myDHT22(DHT22_PIN);

#define LDR_PIN 0 //A0

#define XBEE_SLEEP_PIN 4 //D4
#define BATTERY_VOLTAGE_PIN 2 //A2
#define SOLARPANEL_VOLTAGE_PIN 3 //A3
//===================================
//Weather Sensors
//===================================
#define PIN_ANEMOMETER 2     // D2
#define INT_ANEMOMETER  0
#define PIN_RAIN 3     // D3
#define INT_RAIN 1
#define PIN_VANE 1     // A1

volatile ulong numRevsAnemometer = 0; // Incremented in the interrupt
volatile ulong numClicksRain = 0;
volatile float WindSpeed = 0; //final result of calculation of windspeed
volatile float Precipitation = 0;
volatile int WindDir = 0;

// ADC readings:
#define NUMDIRS 8
prog_uint16_t adc[] PROGMEM = {26, 45, 77, 118, 161, 196, 220, 256};

// These directions match 1-for-1 with the values in adc, but
// will have to be adjusted as noted above. Modify 'dirOffset'
// to which direction is 'away' (it's West here).
prog_char string_0[] PROGMEM = " W";
prog_char string_1[] PROGMEM = "NW";
prog_char string_2[] PROGMEM = " N";
prog_char string_3[] PROGMEM = "SW";
prog_char string_4[] PROGMEM = "NE";
prog_char string_5[] PROGMEM = " S";
prog_char string_6[] PROGMEM = "SE";
prog_char string_7[] PROGMEM = " E";
PROGMEM const char *strVals[] = {
  string_0,
  string_1,
  string_2,
  string_3,
  string_4,
  string_5,
  string_6,
  string_7 };

byte dirOffset=0;
byte wd;

long bmp085Temperature = 0, bmp085Pressure = 0;
DHT22_ERROR_t errorCode;
float batteryVoltage, solarPanelVoltage = 0;

int sendEach;

byte qfe = 0;
int32_t calib = 0;

void setup() {
  xbee.begin(38400);
  
  pinMode(XBEE_SLEEP_PIN, OUTPUT);
  digitalWrite(XBEE_SLEEP_PIN, LOW); // wakes xbee
  
  byte isConfigured;
  isConfigured = EEPROM.read(0);
  if (isConfigured == ISCONFVALUE) {
    loadFromEEPROM();
  } else {
    sendEach = DEFAULT_SEND_EACH;
    qfe = DEFAULT_QFE_QNH;
    calib = DEFAULT_CALIB;
    
    EEPROM_writeInt(EEPROM_ADDR_SEND_EACH, sendEach);
    EEPROM_writeAddr(EEPROM_ADDR_SEND_ADDR, DEFAULT_SEND_ADDR_MSB, DEFAULT_SEND_ADDR_LSB);
    EEPROM_write(EEPROM_ADDR_QFE_QNH, qfe);
    EEPROM_writeLong(EEPROM_ADDR_CALIB, calib);
    EEPROM.write(0, ISCONFVALUE);
  }
  
  Wire.begin();
  delay(1000);
  
  pinMode(PIN_ANEMOMETER, INPUT);
  digitalWrite(PIN_ANEMOMETER, HIGH);
  //these two lines basically set the internal pull-up resistor in pin2;
  attachInterrupt(INT_ANEMOMETER, countAnemometer, FALLING);
  
  pinMode(PIN_RAIN, INPUT);
  digitalWrite(PIN_RAIN, HIGH);
  attachInterrupt(INT_RAIN, countRain, FALLING);
  
  //dps.init();     // QFE (Field Elevation above ground level) is set to 0 meters.
  //dps.init(MODE_STANDARD, 101850, false);  // 101850Pa = 1018.50hPa, false = using Pa units
  //dps.init(MODE_ULTRA_HIGHRES, 3000, true); // 30 meters, true = using meter units
  //dps.init(MODE_ULTRA_HIGHRES, 80500, true); // 805 meters
  if (qfe == 0) {
    dps.init(MODE_ULTRA_HIGHRES, calib, false);
  } else {
    dps.init(MODE_ULTRA_HIGHRES, calib, true);
  }
  myDHT22.readData();
  myDHT22.getTemperatureC();
  myDHT22.getHumidity();
  
  lastPacket = 0;
  lastCapture = 0;
}

void loop() {
  ZBTxRequest zbTx;
  
  if ((lastCapture == 0) || (millis() - lastCapture >= 60000UL)  || lastCapture == 0) {
    digitalWrite(XBEE_SLEEP_PIN, LOW); // wakes xbee
    dps.getTemperature(&bmp085Temperature); 
    dps.getPressure(&bmp085Pressure);
    
    errorCode = myDHT22.readData();
    
    batteryVoltage = analogRead(BATTERY_VOLTAGE_PIN);
    batteryVoltage = (batteryVoltage * 0.0032) * 2;
    solarPanelVoltage = analogRead(SOLARPANEL_VOLTAGE_PIN);
    solarPanelVoltage = (solarPanelVoltage * 0.0032) * 2;
    callback(60);
    lastCapture = millis();
    digitalWrite(XBEE_SLEEP_PIN, HIGH); // sleep xbee
  }
  
  xbee.readPacket();
  if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
      digitalWrite(XBEE_SLEEP_PIN, LOW); // wakes xbee
      xbee.getResponse().getZBRxResponse(rx);
      if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
        dps.getTemperature(&bmp085Temperature); 
        dps.getPressure(&bmp085Pressure);
        
        errorCode = myDHT22.readData();
        
        batteryVoltage = analogRead(BATTERY_VOLTAGE_PIN);
        batteryVoltage = (batteryVoltage * 0.0032) * 2;
        solarPanelVoltage = analogRead(SOLARPANEL_VOLTAGE_PIN);
        solarPanelVoltage = (solarPanelVoltage * 0.0032) * 2;
        
        zbTx = ZBTxRequest(rx.getRemoteAddress64(), payload, sizeof(payload));
        byte type = rx.getData(0);
        switch (type) {
          case 0x01:
            genWeatherPacket();
            xbee.send(zbTx);
            break;
          case 0x02:
            clearPayload();
            addToPayload((float)(bmp085Pressure)/100);
            xbee.send(zbTx);
            break;
          case 0x03:
            clearPayload();
            addToPayload((float)(bmp085Temperature)/10);
            xbee.send(zbTx);
            break;
          case 0x04:
            clearPayload();
            addToPayload((float)myDHT22.getHumidity());
            xbee.send(zbTx);
            break;
          case 0x05:
            clearPayload();
            addToPayload((float)myDHT22.getTemperatureC());
            xbee.send(zbTx);
            break;
          case 0x06:
            clearPayload();
            addToPayload((int)analogRead(LDR_PIN));
            xbee.send(zbTx);
            break;
          case 0x07:
            clearPayload();
            addToPayload((float)batteryVoltage);
            xbee.send(zbTx);
            break;
          case 0x08:
            clearPayload();
            addToPayload((float)solarPanelVoltage);
            xbee.send(zbTx);
            break;
          case 0x09:
            clearPayload();
            char tempChar[2];
            strcpy_P(tempChar, (char*)pgm_read_word(&(strVals[wd])));
            payload[payloadPointer++] = tempChar[0];
            payload[payloadPointer++] = tempChar[1];
            xbee.send(zbTx);
            break;
          case 0x10:
            clearPayload();
            addToPayload((float)WindSpeed);
            xbee.send(zbTx);
            break;
          case 0x11:
            clearPayload();
            addToPayload((float)Precipitation);
            xbee.send(zbTx);
            break;
          case 0x51:
            #ifdef _DEBUG_
            Serial.println();
            Serial.print("Received Set DELAY packet with ");
            Serial.print(rx.getDataLength(), DEC);
            Serial.println(" bytes");
            #endif
            if (rx.getDataLength() == 3) {
              int value = 0;
              byte* p = (byte*)(void*)&value;
              for (int i = 1; i <= sizeof(value); i++)
                *p++ = rx.getData(i);
              EEPROM_writeInt(EEPROM_ADDR_SEND_EACH, value);
              sendEach = EEPROM_readInt(EEPROM_ADDR_SEND_EACH);
            } else {
              clearPayload();
              addToPayload(EEPROM_readInt(EEPROM_ADDR_SEND_EACH));
              xbee.send(zbTx);
            }
            break;
          case 0x52:
            if (rx.getDataLength() == 9) {
              #ifdef _DEBUG_
              Serial.print("MSB:");
              for (byte i=1; i<=4; i++) {
                Serial.print(rx.getData(i), HEX);
                Serial.print(" ");
              }
              Serial.print(", LSB:");
              for (byte i=1; i<=4; i++) {
                Serial.print(rx.getData(i+4), HEX);
                Serial.print(" ");
              }
              Serial.println();
              #endif
              uint32_t msb = 0;
              uint32_t lsb = 0;
              byte* p = (byte*)(void*)&msb;
              byte* q = (byte*)(void*)&lsb;
              for (int i = 1; i <= sizeof(msb); i++)
                *p++ = rx.getData(i);
              for (int i = 1; i <= sizeof(lsb); i++)
                *q++ = rx.getData(i+4);
              EEPROM_writeAddr(EEPROM_ADDR_SEND_ADDR, msb, lsb);
            } else if (rx.getDataLength() == 2) {
              uint32_t msb = 0;
              uint32_t lsb = 0;
              EEPROM_writeAddr(EEPROM_ADDR_SEND_ADDR, msb, lsb);
            } else {
              clearPayload();
              uint32_t msb = 0;
              uint32_t lsb = 0;
              EEPROM_readAddr(EEPROM_ADDR_SEND_ADDR, &msb, &lsb);
              addToPayload(msb);
              addToPayload(lsb);
              xbee.send(zbTx);
            }
            break;
          case 0x53:
            if (rx.getDataLength() == 6) {
              qfe = rx.getData(1);
              byte* p = (byte*)(void*)&calib;
              for (int i = 3; i <= sizeof(calib)+2; i++)
                *p++ = rx.getData(i-1);
              EEPROM_write(EEPROM_ADDR_QFE_QNH, qfe);
              EEPROM_writeLong(EEPROM_ADDR_CALIB, calib);
              if (qfe == 0) {
                dps.init(MODE_ULTRA_HIGHRES, calib, false);
              } else {
                dps.init(MODE_ULTRA_HIGHRES, calib, true);
              }
            }
            break;
        }
      }
      digitalWrite(XBEE_SLEEP_PIN, HIGH); // sleep xbee
    }
  }
  
  if ((millis() - lastPacket) > ((long)sendEach * 1000L) || lastPacket == 0) {
    dps.getTemperature(&bmp085Temperature); 
    dps.getPressure(&bmp085Pressure);
    
    errorCode = myDHT22.readData();
    
    batteryVoltage = analogRead(BATTERY_VOLTAGE_PIN);
    batteryVoltage = (batteryVoltage * 0.0032) * 2;
    solarPanelVoltage = analogRead(SOLARPANEL_VOLTAGE_PIN);
    solarPanelVoltage = (solarPanelVoltage * 0.0032) * 2;
    
    lastPacket = millis();
    genWeatherPacket();
    
    uint32_t msb = 0;
    uint32_t lsb = 0;
    if (EEPROM_readAddr(EEPROM_ADDR_SEND_ADDR, &msb, &lsb)) {
      digitalWrite(XBEE_SLEEP_PIN, LOW); // wake xbee
      #ifdef _DEBUG_
      Serial.print("Trying to send to MSB:");
      Serial.print(msb, HEX);
      Serial.print(", LSB:");
      Serial.println(lsb, HEX);
      #endif
      XBeeAddress64 addr64 = XBeeAddress64(msb, lsb);
      zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
      xbee.send(zbTx);
      digitalWrite(XBEE_SLEEP_PIN, HIGH); // sleep xbee
    }
  }
}

void countAnemometer() {
   numRevsAnemometer++;
}

void countRain() {
  numClicksRain++;
}

void callback(byte seconds){
  /*the ISR for the timer interrupt, called every 5 seconds*/
  //check how many times the switch was closed on this period? this value is on numRevsAnemometer
  WindSpeed = (numRevsAnemometer/ seconds) * 2.4; //2.4 K/h for one switch closure per second
  numRevsAnemometer = 0;
  //Precipitation = 0.28*(numClicksRain/seconds);
  Precipitation = 0.28*numClicksRain;
  //0.2794 mm per contact closure; this step gives number of clicks per minute unit = mm per minute
  numClicksRain = 0;
  
  int val;
  byte x, reading;
  val = analogRead(PIN_VANE);
  val >>=2;                        // Shift to 255 range
  reading = val;

  // Look the reading up in directions table. Find the first value
  // that's >= to what we got.
  for (x=0; x<NUMDIRS; x++) {
     if (pgm_read_word_near(adc + x) >= reading)
        break;
  }
  wd = (x + dirOffset) % 8;   // Adjust for orientation
}

void genWeatherPacket() {
  clearPayload();
  addToPayload((float)(bmp085Pressure)/100);
  addToPayload((float)(bmp085Temperature)/10);
  addToPayload((float)myDHT22.getHumidity());
  addToPayload((float)myDHT22.getTemperatureC());
  addToPayload((int)analogRead(LDR_PIN));
  addToPayload((float)batteryVoltage);
  addToPayload((float)solarPanelVoltage);
  char tempChar[2];
  strcpy_P(tempChar, (char*)pgm_read_word(&(strVals[wd])));
  payload[payloadPointer++] = tempChar[0];
  payload[payloadPointer++] = tempChar[1];
  addToPayload((float)WindSpeed);
  addToPayload((float)Precipitation);
}

void addToPayload(float f) {
  byte * b = (byte *) &f;
  payload[payloadPointer++] = b[0];
  payload[payloadPointer++] = b[1];
  payload[payloadPointer++] = b[2];
  payload[payloadPointer++] = b[3];
}

void addToPayload(int i) {
  byte * b = (byte *) &i;
  payload[payloadPointer++] = b[0];
  payload[payloadPointer++] = b[1];
}

void addToPayload(uint32_t i) {
  byte * b = (byte *) &i;
  payload[payloadPointer++] = b[0];
  payload[payloadPointer++] = b[1];
  payload[payloadPointer++] = b[2];
  payload[payloadPointer++] = b[3];
}

void clearPayload() {
  for (byte i=0; i<sizeof(payload); i++) {
    payload[i] = '\0';
  }
  payloadPointer = 0;
}

void loadFromEEPROM() {
    sendEach = EEPROM_readInt(EEPROM_ADDR_SEND_EACH);
    qfe = EEPROM_read(EEPROM_ADDR_QFE_QNH);
    calib = EEPROM_readLong(EEPROM_ADDR_CALIB);
}

void EEPROM_writeFloat(int ee, float value) {
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
    EEPROM.write(ee++, *p++);
}

float EEPROM_readFloat(int ee) {
  float value = 0.0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(ee++);
  return value;
}

void EEPROM_writeInt(int ee, int value) {
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
    EEPROM.write(ee++, *p++);
}

byte EEPROM_read(int ee) {
  return EEPROM.read(ee);
}

void EEPROM_write(int ee, byte value) {
  return EEPROM.write(ee, value);
}

int EEPROM_readInt(int ee) {
  int value = 0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(ee++);
  return value;
}

void EEPROM_writeLong(int ee, long value) {
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
    EEPROM.write(ee++, *p++);
}

long EEPROM_readLong(int ee) {
  long value = 0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(ee++);
  return value;
}

void EEPROM_writeAddr(int ee, uint32_t msb, uint32_t lsb) {
  //Serial.println("Saving ADDR to EEPROM");
  byte* p = (byte*)(void*)&msb;
  //Serial.print("MSB:");
  for (int i = 0; i < 4; i++) {
    //Serial.print(*p, HEX);
    //Serial.print(" ");
    EEPROM.write(ee++, *p++);
  }
  p = (byte*)(void*)&lsb;
  //Serial.print("LSB:");
  for (int i = 0; i < 4; i++) {
    //Serial.print(*p, HEX);
    //Serial.print(" ");
    EEPROM.write(ee++, *p++);
  }
  //Serial.println();
}

bool EEPROM_readAddr(int ee, uint32_t* msb, uint32_t* lsb) {
  //int value = 0;
  //byte* p = (byte*)(void*)&value;
  //for (int i = 0; i < sizeof(value); i++)
  //  *p++ = EEPROM.read(ee++);
  //return value;
  
  uint32_t msbvalue = 0;
  uint32_t lsbvalue = 0;
  
  byte* p = (byte*)(void*)&msbvalue;
  //Serial.print("Getting ADDR64 from EEPROM address ");
  //Serial.println(ee, DEC);
  //Serial.print("MSB:");
  for (int i = 0; i < 4; i++) {
    *p = EEPROM.read(ee++);
    //Serial.print(*msb, HEX);
    //Serial.print(" ");
    *p++;
  }
  *msb = msbvalue;
  //Serial.print("LSB:");
  p = (byte*)(void*)&lsbvalue;
  for (int i = 0; i < 4; i++) {
    *p = EEPROM.read(ee++);
    //Serial.print(*lsb, HEX);
    //Serial.print(" ");
    *p++;
  }
  //Serial.println();
  *lsb = lsbvalue;
  
  uint32_t tmp_a, tmp_b, tmp_c, tmp_d;
  tmp_a = (*msb & 0xff000000) >> 24;
  tmp_b = (*msb & 0x00ff0000) >> 8;
  tmp_c = (*msb & 0x0000ff00) << 8 ;
  tmp_d = (*msb & 0x000000ff) << 24;
  *msb = tmp_d | tmp_c |tmp_b | tmp_a;
  tmp_a = (*lsb & 0xff000000) >> 24;
  tmp_b = (*lsb & 0x00ff0000) >> 8;
  tmp_c = (*lsb & 0x0000ff00) << 8 ;
  tmp_d = (*lsb & 0x000000ff) << 24;
  *lsb = tmp_d | tmp_c |tmp_b | tmp_a;
  
  //Serial.print("MSB:");
  //Serial.print(msbvalue, HEX);
  //Serial.print(", LSB:");
  //Serial.println(lsbvalue, HEX);
  
  if (msbvalue == 0 && lsbvalue == 0)
    return false;
  return true;
}
