#include<SoftwareSerial.h>
SoftwareSerial e5(5, 4);
#include <TinyGPSPlus.h>
TinyGPSPlus gps;
bool newData = false;
String latitude;
double flat2 = 0;
String longitude;
double flon2 = 0;
int semaphore = 0;

static int at_send_check_response(char *p_ack, int timeout_ms, char *p_cmd, ...)
{
  int ch;
  int num = 0;
  int index = 0;
  int startMillis = 0;
  va_list args;
  int inByte = 0;
  //static char recv[256];
  static char recv_buf[256];
  memset(recv_buf, 0, sizeof(recv_buf));
  va_start(args, p_cmd);
  e5.print(p_cmd);
  Serial.print(p_cmd);
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
      //}
      delay(2);
    }

    if (strstr(recv_buf, p_ack) != NULL)
    {
      return 1;
    }

  } while (millis() - startMillis < timeout_ms);
  return 0;
}

static void recv_prase()
{
  static char recv_buf[256];
  char ch;
  int index = 0;
  int rssi = 0;
  int snr = 0;
  int len = 0;
  memset(recv_buf, 0, sizeof(recv_buf));
  while (e5.available() > 0)
  {
    ch = e5.read();
    recv_buf[index++] = ch;
    //Serial.print((char)ch);
    delay(2);
  }
  if (index)
  {
    char *p_start = NULL;
    char data[128];       // To hold the received bytes as characters

    int bytes_len = 0;
    p_start = strstr(recv_buf, "RSSI:");
    if (p_start && (1 == sscanf(p_start, "RSSI:%d", &rssi)))
    {
      Serial.print("RSSI;"); Serial.println(rssi);
    }

    p_start = strstr(recv_buf, "SNR:");
    if (p_start && (1 == sscanf(p_start, "SNR:%d", &snr)))
    {
      Serial.print("SNR;"); Serial.println(snr);
    }

    p_start = strstr(recv_buf, "LEN:");
    if (p_start && (1 == sscanf(p_start, "LEN:%d", &len)))
    {
      Serial.print("LEN;"); Serial.println(len);
    }


    p_start = strstr(recv_buf, "+TEST: RX");

    if (p_start && (1 == sscanf(p_start, "+TEST: RX \"%s\"", &data)))
    {
      for (int i = 0; i < sizeof(data); i++) {
        if (int(data[i + 1]) == 0) {
          bytes_len = i;
          break;
        }
      }
    }

    // Convert the characters to a byteArray
    int message_len = bytes_len / 2 + 1;
    byte out[message_len];
    auto getNum = [](char c) {
      return c > '9' ? c - 'A' + 10 : c - '0';
    };
    for (int x = 0, y = 0; x < bytes_len; ++x, ++y)
      out[y] = (getNum(data[x++]) << 4) + getNum(data[x]);
    out[message_len] = '\0';

    //Print the received bytes
    for (int i = 0; i < sizeof(out) - 1; i++)
    {
    }
    //Serial.println();
    char str[(sizeof out) + 1];
    memcpy(str, out, sizeof out);
    str[sizeof out] = 0;
    String strC = String(str);
    char datac[(sizeof out) + 1];
    strC.toCharArray(datac, sizeof out + 1);

    int checkNumber = datac[1] - '0';
    if (checkNumber == 3)
    {
      String latitude = strC.substring(3, 12);
      flat2 = latitude.toDouble();
      String longitude = strC.substring(15, 24);
      flon2 = longitude.toDouble();
      semaphore++;
    }
    Serial.println(str);
  }
}

void gpsCalc()
{
  while (Serial1.available())
  {
    if (gps.encode(Serial1.read())) // Did a new valid sentence come in?
      //Serial.print("SOS2 ");
      newData = true;
  }

  if (newData && semaphore > 0)
  {
    if (gps.location.isValid()) {
      double flat1 = gps.location.lat();
      double flon1 = gps.location.lng();
      double distancePoints =  gps.distanceBetween(flat1, flon1, flat2, flon2);
      Serial.print("Distance;"); Serial.println(distancePoints);
      semaphore--;
    }
  }

}


double round_to_dp(double in_value, int decimal_place )
{
  double multiplier = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplier ) / multiplier;
  return in_value;
}


void setup(void)
{
  Serial.begin(9600);
  e5.begin(9600);
  Serial1.begin(9600);    // gps
  Serial.print("Serial2 LOCAL TEST\r\n");
  at_send_check_response("+AT: OK", 100, "AT\r\n");
  at_send_check_response("+MODE: TEST", 1000, "AT+MODE=TEST\r\n");
  at_send_check_response("+TEST: RFCFG", 1000, "AT+TEST=?");
  at_send_check_response("+TEST: RFCFG", 1000, "AT+TEST=RFCFG,866,SF12,250,12,15,14,ON,OFF,OFF\r\n");
  at_send_check_response("+TEST: RXLRPKT", 1000, "AT+TEST=RXLRPKT\r\n");
  delay(200);
}



void loop(void)
{

  recv_prase();
  gpsCalc();
}
