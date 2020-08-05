#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_CCS811.h>

//User Modified Part
#define wifi_ssid     "CU_2.4G_22AE"    
#define wifi_psw      "3056248abc"     
#define clientIDstr   "example"
#define timestamp     "999"
#define ProductKey    "a16jkRAEsuF"
#define DeviceName    "Smart_House"
#define DeviceSecret  "55272b4f1d2337f07d1ce2c8bfe92b55"
#define password      "77be2efc9e5ce386aaff31aca3608a141bac7ddc"



//Logic Preset
#define OFF           0
#define ON            1
#define MUTE          2
#define KEEP_OFF      2
#define KEEP_ON       3

#define AC_ON   digitalWrite(ACPin,HIGH)
#define AC_OFF  digitalWrite(ACPin,LOW)

#define Fan_ON      digitalWrite(FanPin,HIGH)
#define Fan_OFF     digitalWrite(FanPin,LOW)

#define Buzzer_ON   digitalWrite(BuzzerPin,HIGH)
#define Buzzer_OFF  digitalWrite(BuzzerPin,LOW)

#define Pump_ON     digitalWrite(PumpPin,HIGH)
#define Pump_OFF    digitalWrite(PumpPin,LOW)

#define AC_ON_val     (28.00)
#define AC_OFF_val    (26.50)
#define Gas_ON_val      400
#define Gas_OFF_val     150
#define Light_ON_val      400
#define Light_OFF_val     200
#define Pump_ON_val       900
#define Pump_OFF_val      400
#define eCO2_ON_val        900
#define eCO2_OFF_val       800 

//Status Pool
float RoomTemp;
int   AC = OFF;
int   Buzzer = OFF;
int   GasDetector = 0;
int   Fan = OFF;
int   LightDetector = 0;
int   Light = OFF;
int   Curtain = ON;
int   SoilHumi = 0;
int   Pump = OFF;
int   eCO2 = 0;
int   TVOC = 0;

//ATcmd Format
#define AT                    "AT\r"
#define AT_OK                 "OK"
#define AT_REBOOT             "AT+REBOOT\r"
#define AT_ECHO_OFF           "AT+UARTE=OFF\r"
#define AT_MSG_ON             "AT+WEVENT=ON\r"

#define AT_WIFI_START         "AT+WJAP=%s,%s\r"
#define AT_WIFI_START_SUCC    "+WEVENT:STATION_UP"

#define AT_MQTT_AUTH          "AT+MQTTAUTH=%s&%s,%s\r"
#define AT_MQTT_CID           "AT+MQTTCID=%s|securemode=3\\,signmethod=hmacsha1\\,timestamp=%s|\r"
#define AT_MQTT_SOCK          "AT+MQTTSOCK=%s.iot-as-mqtt.cn-shanghai.aliyuncs.com,1883\r"

#define AT_MQTT_AUTOSTART_OFF "AT+MQTTAUTOSTART=OFF\r"
#define AT_MQTT_ALIVE         "AT+MQTTKEEPALIVE=500\r"
#define AT_MQTT_START         "AT+MQTTSTART\r"
#define AT_MQTT_START_SUCC    "+MQTTEVENT:CONNECT,SUCCESS"
#define AT_MQTT_PUB_SET       "AT+MQTTPUB=/sys/%s/%s/thing/event/property/post,1\r"
#define AT_MQTT_PUB_ALARM_SET "AT+MQTTPUB=/sys/%s/%s/thing/event/GasAlarm/post,1\r"
#define AT_MQTT_PUB_DATA      "AT+MQTTSEND=%d\r"
#define JSON_DATA_PACK        "{\"id\":\"100\",\"version\":\"1.0\",\"method\":\"thing.event.property.post\",\"params\":{\"RoomTemp\":%d.%02d,\"AC\":%d,\"Fan\":%d,\"Buzzer\":%d,\"GasDetector\":%d}}\r"
#define JSON_DATA_PACK_2      "{\"id\":\"110\",\"version\":\"1.0\",\"method\":\"thing.event.property.post\",\"params\":{\"LightDetector\":%d,\"Curtain\":%d,\"Light\":%d,\"SoilHumi\":%d,\"Pump\":%d,\"eCO2\":%d,\"TVOC\":%d}}\r"
#define JSON_DATA_PACK_ALARM  "{\"id\":\"110\",\"version\":\"1.0\",\"method\":\"thing.event.GasAlarm.post\",\"params\":{\"GasDetector\":%d}}\r"
#define AT_MQTT_PUB_DATA_SUCC "+MQTTEVENT:PUBLISH,SUCCESS"
#define AT_MQTT_UNSUB         "AT+MQTTUNSUB=2\r"
#define AT_MQTT_SUB           "AT+MQTTSUB=2,/sys/%s/%s/thing/service/property/set,1\r"
#define AT_MQTT_SUB_SUCC      "+MQTTEVENT:2,SUBSCRIBE,SUCCESS"

#define AT_BUZZER_MUTE           "\"Buzzer\":2"

//Pin Map
#define ACPin                 2
#define BuzzerPin             3
#define PumpPin               4
#define CurtainOpenPin        5
#define CurtainClosePin       6
#define Light1Pin             7
#define Light2Pin             8
#define Light3Pin             9
#define FanPin                11

#define GasPin                A0
#define SoilHumiPin           A1
#define LightPin              A2

#define DEFAULT_TIMEOUT       10   //seconds
#define BUF_LEN               100
#define BUF_LEN_DATA          190

char      ATcmd[BUF_LEN];
char      ATbuffer[BUF_LEN];
char      ATdata[BUF_LEN_DATA];

Adafruit_BME280 bme;
unsigned  bme280_status;
Adafruit_CCS811 ccs;
unsigned  ccs811_status;


void setup() {
  //Serial Initial
  Serial3.begin(115200);
  
  //Pin Initial
  Pin_init();
  
  //Sensor Initial
  
  bme280_status = bme.begin();
  while(!bme280_status)bme280_status = bme.begin();
       
  ccs811_status = ccs.begin();
  while(!ccs811_status)ccs811_status = ccs.begin();
  
  while(!ccs.available());
  float temp = ccs.calculateTemperature();
  ccs.setTempOffset(temp - 25.0);
  
  
  BEEP(1);
  
  //Cloud Initial
  while(1)
  {
    if(!WiFi_init())continue;
    BEEP(2);
    if(!Ali_connect())continue;
    break;
  }
  BEEP(3);
}

void loop() {
  
  //SensorCollect
  
  
  RoomTemp = bme.readTemperature();
  GasDetector = analogRead(GasPin);
  LightDetector = analogRead(LightPin);
  SoilHumi = analogRead(SoilHumiPin);
  
  if(ccs.available()){
    if(!ccs.readData()){
      eCO2 = ccs.geteCO2();
      TVOC = ccs.getTVOC();
    }
  }

  //Logic Process
  //1.RoomTemp       
  if((RoomTemp > AC_ON_val)&&(AC == OFF))
  {
    AC = ON;
    AC_ON;
  }
  if((RoomTemp < AC_OFF_val)&&(AC == ON))
  {
    AC = OFF;
    AC_OFF;
  }
  //2.Gas&PM2.5
  if(((GasDetector > Gas_ON_val)||(eCO2 > eCO2_ON_val))&&(Fan == OFF))
  {
    Fan = ON;
    Fan_ON;
  }
  if((GasDetector > Gas_ON_val)&&(Buzzer == OFF))
  {
    Buzzer = ON;
    Buzzer_ON;
    
    cleanBuffer(ATcmd,BUF_LEN);
    snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_ALARM_SET,ProductKey,DeviceName);
    bool mainflag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);

    cleanBuffer(ATdata,BUF_LEN_DATA);
    int mainlen = snprintf(ATdata,BUF_LEN_DATA,JSON_DATA_PACK_ALARM,GasDetector);

    cleanBuffer(ATcmd,BUF_LEN);
    snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,mainlen-1);
    mainflag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
    if(mainflag) mainflag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);
  }
  if((GasDetector < Gas_OFF_val)&&(Buzzer != OFF))
  {
    Buzzer = OFF;
    Buzzer_OFF;
  }
  if((GasDetector < Gas_OFF_val)&&(eCO2 < eCO2_OFF_val)&&(Fan == ON))
  {
    Fan = OFF;
    Fan_OFF;
  }
  //3.Light
  if((LightDetector > Light_ON_val)&&(Curtain == ON))
  {
    Curtain = OFF;
    Curtain_OFF();
    Light_ON();
    Light = ON;
  }

  if((LightDetector < Light_OFF_val)&&(Curtain == OFF))
  {
    Curtain = ON;
    Curtain_ON();
    Light_OFF();
    Light = OFF;
  }
  //4.SoilHumi
  if((SoilHumi > Pump_ON_val)&&(Pump == OFF))
  {
    Pump = ON;
    Pump_ON;
  }
  if((SoilHumi < Pump_OFF_val)&&(Pump == ON))
  {
    Pump = OFF;
    Pump_OFF;
  }

  //Upload
  Upload();

  //MsgReceive
  if(check_send_cmd(AT,AT_BUZZER_MUTE,DEFAULT_TIMEOUT))Buzzer_mute();
}

bool Upload()
{
  bool flag;
  int inte1,frac1;
  int len;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_SET,ProductKey,DeviceName);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
   
  
  cleanBuffer(ATdata,BUF_LEN_DATA);

  inte1 = (int)(RoomTemp);
  frac1 = (RoomTemp - inte1) * 100;
  
  len = snprintf(ATdata,BUF_LEN_DATA,JSON_DATA_PACK,inte1,frac1,AC,Fan,Buzzer,GasDetector);
  
  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
  flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
  if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);
  
  
//  delay(500);
  
  cleanBuffer(ATdata,BUF_LEN_DATA);
  len = snprintf(ATdata,BUF_LEN_DATA,JSON_DATA_PACK_2,LightDetector,Curtain,Light,SoilHumi,Pump,eCO2,TVOC);

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
  flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
  if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);

  return flag;
}

bool Ali_connect()
{
  bool flag;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_AUTH,DeviceName,ProductKey,password);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_CID,clientIDstr,timestamp);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_SOCK,ProductKey);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  flag = check_send_cmd(AT_MQTT_AUTOSTART_OFF,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  flag = check_send_cmd(AT_MQTT_ALIVE,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  flag = check_send_cmd(AT_MQTT_START,AT_MQTT_START_SUCC,20);
  if(!flag)return false;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_SET,ProductKey,DeviceName);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  //flag = check_send_cmd(AT_MQTT_UNSUB,AT_OK,DEFAULT_TIMEOUT);
  //if(!flag)return false;
  
  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_SUB,ProductKey,DeviceName);
  flag = check_send_cmd(ATcmd,AT_MQTT_SUB_SUCC,DEFAULT_TIMEOUT);
  if(!flag)BEEP(4);
  return flag;
}

bool WiFi_init()
{
  bool flag;

  flag = check_send_cmd(AT,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;
  
  flag = check_send_cmd(AT_REBOOT,AT_OK,20);
  if(!flag)return false;
  delay(5000);

  flag = check_send_cmd(AT_ECHO_OFF,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  flag = check_send_cmd(AT_MSG_ON,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;
  
  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_WIFI_START,wifi_ssid,wifi_psw);
  flag = check_send_cmd(ATcmd,AT_WIFI_START_SUCC,20);
  return flag;
}

bool check_send_cmd(const char* cmd,const char* resp,unsigned int timeout)
{
  int i = 0;
  unsigned long timeStart;
  timeStart = millis();
  cleanBuffer(ATbuffer,BUF_LEN);
  Serial3.print(cmd);
  Serial3.flush();
  while(1)
  {
    while(Serial3.available())
    {
      ATbuffer[i++] = Serial3.read();
      if(i >= BUF_LEN)i = 0;
    }
    if(NULL != strstr(ATbuffer,resp))break;
    if((unsigned long)(millis() - timeStart > timeout * 1000)) break;
  }
  
  if(NULL != strstr(ATbuffer,resp))return true;
  return false;
}

void cleanBuffer(char *buf,int len)
{
  for(int i = 0;i < len;i++)
  {
    buf[i] = '\0';
  } 
}

void Pin_init()
{
  pinMode(ACPin,OUTPUT);
  digitalWrite(ACPin,LOW);
  pinMode(BuzzerPin,OUTPUT);
  digitalWrite(BuzzerPin,LOW);
  pinMode(PumpPin,OUTPUT);
  digitalWrite(PumpPin,LOW);
  pinMode(CurtainOpenPin,OUTPUT);
  digitalWrite(CurtainOpenPin,LOW);
  pinMode(CurtainClosePin,OUTPUT);
  digitalWrite(CurtainClosePin,LOW);
  pinMode(Light1Pin,OUTPUT);
  digitalWrite(Light1Pin,LOW);
  pinMode(Light2Pin,OUTPUT);
  digitalWrite(Light2Pin,LOW);
  pinMode(Light3Pin,OUTPUT);
  digitalWrite(Light3Pin,LOW);
  pinMode(FanPin,OUTPUT);
  digitalWrite(FanPin,LOW);
  Curtain_ON();
}

void BEEP(int b_time)
{
  for(int i = 1;i <= b_time;i++)
  { 
    digitalWrite(BuzzerPin,HIGH);
    delay(100);
    digitalWrite(BuzzerPin,LOW);
    delay(100);
  }
}
void Buzzer_mute()
{
  Buzzer_OFF;
  Buzzer = MUTE;
}
void Curtain_ON()
{
  digitalWrite(CurtainOpenPin,HIGH);
  delay(200);
  digitalWrite(CurtainOpenPin,LOW);
}

void Curtain_OFF()
{
  digitalWrite(CurtainClosePin,HIGH);
  delay(200);
  digitalWrite(CurtainClosePin,LOW);
}

void Light_ON()
{
  digitalWrite(Light1Pin,HIGH);
  digitalWrite(Light2Pin,HIGH);
  digitalWrite(Light3Pin,HIGH);
}

void Light_OFF()
{
  digitalWrite(Light1Pin,LOW);
  digitalWrite(Light2Pin,LOW);
  digitalWrite(Light3Pin,LOW);
}
