/*
  Arduino Nano Every
  Sensors:
     bme280/680    i2c
     neo6mv2/YIC   Serial1;
     mpu6050   i2c

*/
// Add libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>


// create objects
Adafruit_BME680 bme; // I2C
MPU6050 mpu;
TinyGPSPlus gps;
SoftwareSerial e5(9, 8);

//Messages
String bme_id = "*1";
String mpu_id = "*2";
String gps_id = "*3";
String sys_id = "*4";
String button_id = "*5";
int sensor_connections[5];
bool exist = false;
bool join = false;

char charMsg[50];
volatile int params[4] = {1000, 1000, 1000, 1000};

unsigned long millissendBME = 0;
unsigned long millissendIMU = 0;
unsigned long millissendGPS = 0;
unsigned long millissendSYS = 0;
volatile unsigned long intervalsendBME = 10000;
volatile unsigned long intervalsendIMU = 1000;
volatile unsigned long intervalsendGPS = 1000;
volatile unsigned long intervalsendSYS = 10000;
int counter = 0;
const int pinButton = 10;
int lastButton = 1;
int currentButton = 1;
int countClick = 0;
unsigned long millisButton = 0;
unsigned long millisInterval = 0;
unsigned long millisLong = 0;
unsigned long intervalLong = 4000;
unsigned long intervalInterval = 1500;

int pinLeds[] = {5, 4, 6};
#define LED_CORRECT 1

struct LEDS {
  int rgb[6];
  int pos;
  unsigned long millisBlink;
  int active;
};

byte colorsLed[4][3] = {{0, 255, 255}, {255, 0, 255}, {255, 255, 0}, {255, 255, 255}};
LEDS LEDS1[] = {
  {{0, 0, 0, 0, 0, 0}, 0, 0, 1},  // 0 - not working
  {{0, 1, 0, 0, 0, 0}, 1, 0, 0}, // 1 - pair - green blink
  {{1, 1, 1, 0, 0, 0}, 1, 0, 0}, // 2 - battery charge   - white blink
  {{1, 0, 0, 0, 0, 0}, 1, 0, 0}, // 3 - battery low - red blink
  {{0, 0, 1, 0, 0, 1}, 0, 0, 0},  // 4 - acivated - blue glow
  {{1, 0, 0, 1, 0, 0}, 0, 0, 0},  // 5 - not activeted - red glow
  {{0, 0, 1, 0, 0, 0}, 0, 0, 0},  // 6 - search wifi  - blue blink
  {{0, 1, 0, 0, 1, 0}, 0, 0, 0},  // 7 - wait click button  - green
};
int tekLeds = 0;

float h, t, p, g;
float bat, fan;
float faltitude;
float flat, flon;
int statusgps = 0;
String flat_s, flon_s;
int nsatellites;
float angleX = 0;
float angleY = 0;
float angleZ = 0;
float angleNorth = 0;
float angleXold = 300;
float angleYold = 300;
float angleZold = 300;
float angleNorthold = 300;

boolean startDMP = false;
unsigned long millisk = 0;
String uid = "EETN00104220300064";


const float toDeg = 180.0 / M_PI;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
bool newData = false;

const byte MPU9255_I2C_ADDR = 0x68;
const byte MPU9255_MAG_I2C_ADDR = 0x0C;
//// Регистры датчика MPU-9255:
const byte USER_CTRL = 0x6A; // { - | FIFO_EN | I2C_MST_EN | I2C_IF_DIS | - | FIFO_RST | I2C_MST_RST | SIG_COND_RST }
const byte PWR_MGMT_1 = 0x6B; // { H_RESET | SLEEP | CYCLE | GYRO_STANDBY | PD_PTAT | CLKSEL[2:0] }
const byte INT_PIN_CFG = 0x37; // { ACTL | OPEN | LATCH_INT_EN | INT_ANYRD_2CLEAR | ACTL_FSYNC | FSYNC_INT_MODE_EN | BYPASS_EN | - }
const byte CNTL1 = 0x0A; // { 0 | 0 | 0 | BIT | MODE3 | MODE2 | MODE1 | MODE0 }

#define SEALEVELPRESSURE_HPA (1013.25)


String inputString = "";

boolean firstByte = false;

boolean stringComplete = false;

boolean sendData = true;

unsigned long millist = 0;
int stled = 0;


static int at_send_check_response(char *p_ack, int timeout_ms, char *p_cmd, ...)
{
  int ch;
  int num = 0;
  int index = 0;
  int startMillis = 0;
  va_list args;
  static char recv_buf[256];
  memset(recv_buf, 0, sizeof(recv_buf));
  va_start(args, p_cmd);
  e5.print(p_cmd);
  va_end(args);
  delay(200);
  startMillis = millis();

  if (p_ack == NULL)
    return 0;

  do
  {
    while (e5.available() > 0)
    {
      ch = e5.read();
      recv_buf[index++] = ch;
      Serial.print((char)ch);
      delay(2);
    }

    if (strstr(recv_buf, p_ack) != NULL)
    {
      return 1;
    }

  } while (millis() - startMillis < timeout_ms);

  Serial.println();
  return 0;
}



void setup()
{
  int check = 0;
  Serial.begin(9600);
  inputString.reserve(30);
  Serial1.begin(9600);    // gps
  e5.begin(9600);
  delay(1000);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++)
    {
      pinMode(pinLeds[j], OUTPUT);
      analogWrite(pinLeds[j], colorsLed[i][j]);
    }
    delay(1000);
  }
  pinMode(pinButton, INPUT_PULLUP);
  bme.begin();

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  Wire.begin();
  mpu.initialize();
  initDMP();
  Write(PWR_MGMT_1, 0x8);
  Write(INT_PIN_CFG, 0x02);
  WriteMag(CNTL1, 0x12);

  at_send_check_response("+MODE: TEST", 1000, "AT+MODE=TEST\r\n");
  check = at_send_check_response("+AT: OK", 100, "AT\r\n");
  if (check)
  {
    exist = true;
    at_send_check_response("+MODE: TEST", 1000, "AT+MODE=TEST\r\n");
    at_send_check_response("+TEST: RFCFG", 1000, "AT+TEST=?");
    at_send_check_response("+TEST: RFCFG", 1000, "AT+TEST=RFCFG,866,SF12,250,12,15,14,ON,OFF,OFF\r\n");
    delay(200);
  }
  else
  {
    exist = false;
    Serial.println("LoRa-E5 not connected");
  }

  digitalWrite(13, HIGH);
  delay(2000);
  digitalWrite(13, LOW);
}

void loop()
{
  int ret = 0;

  if (millis() - millist > 1000) {
    digitalWrite(13, stled);
    stled = 1 - stled;
    millist = millis();
  }
  serialEvent();
  if (stringComplete) {
    parse_string(inputString);
    for (int i = 0; i < 5; i++) {
    }
    run_from_get();
    inputString = "";
    stringComplete = false;
  }
  //   ******************** leds
  if (millis() - LEDS1[tekLeds].millisBlink >= 500) {
    for (int i = 0; i < 3; i++) {
      analogWrite(pinLeds[i], abs(LEDS1[tekLeds].rgb[3 * LEDS1[tekLeds].pos + i] - LED_CORRECT));
    }
    LEDS1[tekLeds].pos = (LEDS1[tekLeds].pos + 1) % 2;
    LEDS1[tekLeds].millisBlink = millis();
  }
  // ********************** button
  int res = getButton();
  if (res > 0) {
    counter++;
    String numRes = String(res) + ";" + String(counter) +  ";";
    char dataConvert[numRes.length() + 1];
    char data[numRes.length() + 1];
    numRes.toCharArray(dataConvert, numRes.length());
    memset(data, 0, sizeof(data));
    sprintf(data, "AT+TEST=TXLRSTR,\"%s;$\"\r\n", dataConvert);
    Serial.println();
    ret = 0;
    ret = at_send_check_response("TX DONE", 2000, data);
  }
  //************** bme
  if (! bme.performReading()) {
    Serial.println("No Reading");
    sensor_connections[1] = 0;
  }
  if (millis() - millissendBME >= intervalsendBME) {
    counter = counter + 1;
    h = bme.readHumidity();
    t = bme.readTemperature();
    p = bme.readPressure();
    g = bme.readGas() / 1000.0;
    String bme680 = bme_id + ";" + String(h) + ";" +
                    String(t) + ";"
                    + String(round(p)) + ";" + String(g) + ";" + String(counter) + ";";

    char dataConvert[bme680.length() + 1];
    char data[bme680.length() + 1];
    bme680.toCharArray(dataConvert, bme680.length());
    memset(data, 0, sizeof(data));
    sprintf(data, "AT+TEST=TXLRSTR,\"%s;$\"\r\n", dataConvert);
    Serial.println();
    ret = 0;
    ret = at_send_check_response("TX DONE", 2000, data);
    delay(2000);
    millissendBME = millis();

    sensor_connections[1] = 1;
  }
  //************** system
  if (millis() - millissendSYS >= intervalsendSYS) {
    counter++;
    bat = round(analogRead(A0) * 1.0 / 1023 * 1190) / 10;
    fan = 34;
    String sys = sys_id + ";" + uid + ";" +
                 String(bat) + ";"
                 + String(fan) + ";" + String(counter) + ";";
    char dataConvert[sys.length() + 1];
    char data[sys.length() + 1];
    sys.toCharArray(dataConvert, sys.length());
    memset(data, 0, sizeof(data));
    sprintf(data, "AT+TEST=TXLRSTR,\"%s;$\"\r\n", dataConvert);
    Serial.println();
    ret = 0;
    ret = at_send_check_response("TX DONE", 2000, data);
    delay(2000);
    millissendSYS = millis();
    //digitalWrite(13, LOW);
  }
  //*********************** mpu6050
  if (startDMP || mpu.testConnection()) {
    if (millis() - millissendIMU > intervalsendIMU) {
      int xh = ReadMag(0x04);
      int xl = ReadMag(0x03);
      int yh = ReadMag(0x06);
      int yl = ReadMag(0x05);
      int zh = ReadMag(0x08);
      int zl = ReadMag(0x07);
      int status = ReadMag(0x09);

      // Собираем значения:
      int x = word(xh, xl);
      int y = word(yh, yl);
      int z = word(zh, zl);

      angleNorth = GetAzimuth( x, y);
      angleNorthold = angleNorth;

      // mpu6050
      if (startDMP == true) {
        getAngles();
        counter++;
        String dIMU = mpu_id + ";" + String(round(angleX * 10) / 10) + ";" +
                      String(round(angleY * 10) / 10) + ";" +
                      String(round(angleZ * 10) / 10) + ";" +
                      String(round(angleNorth * 10) / 10) + ";" + String(counter) + ";";
        char dataConvert[dIMU.length() + 1];
        char data[dIMU.length() + 1];
        dIMU.toCharArray(dataConvert, dIMU.length());
        memset(data, 0, sizeof(data));
        sprintf(data, "AT+TEST=TXLRSTR,\"%s;$\"\r\n", dataConvert);
        Serial.println();
        ret = 0;
        ret = at_send_check_response("TX DONE", 2000, data);
        angleXold = angleX;
        angleYold = angleY;
        angleZold = angleZ;
      }
      sensor_connections[2] = 1;
      delay(2000);
      millissendIMU = millis();
    }
  }
  else
  {
    sensor_connections[2] = 0;
  }
  // ******************* gps
  if (millis() - millissendGPS >= intervalsendGPS) {
    for (unsigned long start = millis(); millis() - start < 1000;)
    {
      while (Serial1.available())
      {
        if (gps.encode(Serial1.read())) // Did a new valid sentence come in?
          newData = true;
      }
    }
    if (newData)
    {
      if (gps.location.isValid()) {
        counter++;
        faltitude = gps.altitude.meters();
        nsatellites = gps.satellites.value();
        flat_s = String(gps.location.lat(), 8);
        flon_s = String(gps.location.lng(), 8);

        String GPS = gps_id + ";" + flat_s + ";" +
                     flon_s + ";" +
                     String(nsatellites) + ";" +
                     String(faltitude) + ";" + String(counter) + ";";
        char dataConvert[GPS.length() + 1];
        char data[GPS.length() + 1];
        GPS.toCharArray(dataConvert, GPS.length());
        memset(data, 0, sizeof(data));
        sprintf(data, "AT+TEST=TXLRSTR,\"%s;$\"\r\n", dataConvert);
        Serial.println();
        ret = 0;
        ret = at_send_check_response("TX DONE", 2000, data);
        statusgps = 1;
      }
    }
    else if (statusgps == 1) {
      counter++;
      String GPS = gps_id + ";" + flat_s + ";" +
                   flon_s + ";" +
                   String(nsatellites) + ";" +
                   String(faltitude) + ";" + String(counter) + ";";
      //Serial.println(GPS);
      char dataConvert[GPS.length() + 1];
      char data[GPS.length() + 1];
      GPS.toCharArray(dataConvert, GPS.length());
      memset(data, 0, sizeof(data));
      sprintf(data, "AT+TEST=TXLRSTR,\"%s;$\"\r\n", dataConvert);
      Serial.println();
      ret = 0;
      ret = at_send_check_response("TX DONE", 2000, data);
    }
    delay(2000);
    millissendGPS = millis();
  }
}
