#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#define SIGNAL_TIMEOUT 1000
#define DATA_STIME 50 //The time between each data send cycle
#define DATA_PTIME 1000 //The time between each data send cycle
// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t receiverMacAddress[] = {0x48,0xE7,0x29,0x95,0xED,0xC0}; //0x48,0xE7,0x29,0x95,0xED,0xC0 Test board ESP32
                                                                //0xA0,0xA3,0xB3,0x28,0x6C,0x5C Drone ESP32
unsigned long lastRecvTime = 0;
unsigned long time_prev = 0;
int cnt = 0;
bool signal_send;

struct PacketData{
  byte xAxisValue;
  byte yAxisValue;
 
  byte switch1Value;
  byte switch2Value;

  byte potValue;
};
PacketData data;

struct GPSData{
  float LatitudeValue;
  float LongtitudeValue;
  float AltitudeValue;
  int HourValue;
  int MinuteValue;
  int SecondValue;
  float gx;
  float gy;
  float gz;
  float ax;
  float ay;
  float az;
  int thrust;
  int flm;
  int frm;
  int rlm;
  int rrm;
};
GPSData gpsData;

esp_now_peer_info_t peerInfo;

void SerialDataPrint();

//This function is used to map 0-4095 joystick value to 0-254. hence 127 is the center value which we send.
//It also adjust the deadband in joystick.
//Jotstick values range from 0-4095. But its center value is not always 2047. It is little different.
//So we need to add some deadband to center value. in our case 1800-2200. Any value in this deadband range is mapped to center 127.
int mapAndAdjustJoystickDeadBandValues(int value, bool reverse)
{
  if (value >= 2200)
  {
    value = map(value, 2200, 4095, 127, 254);
  }
  else if (value <= 1800)
  {
    value = (value == 0 ? 0 : map(value, 1800, 0, 127, 0));  
  }
  else
  {
    value = 127;
  }

  if (reverse)
  {
    value = 254 - value;
  }
  //Serial.println(value);  
  return value;
}

//Create flip flops to delay sending time
void FlipFlop() {
  if (cnt == DATA_STIME){
    cnt = 0;
    signal_send = true;
  }
  else{
    cnt = cnt + 1;
    signal_send = false;
  }
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // Serial.print("\r\nLast Packet Send Status:\t ");
  // Serial.println(status);
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  if (len == 0)
  {
    return;
  }
  memcpy(&gpsData, incomingData, sizeof(gpsData));
      
  lastRecvTime = millis(); 
}

void setup() 
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else
  {
    Serial.println("Succes: Initialized ESP-NOW");
  }

  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  else
  {
    Serial.println("Succes: Added peer");
  } 

  pinMode(18,INPUT_PULLUP);
  pinMode(19,INPUT_PULLUP);
  esp_now_register_recv_cb(OnDataRecv);  
}
 
void loop() 
{
  data.xAxisValue    = mapAndAdjustJoystickDeadBandValues(analogRead(32), false);
  data.yAxisValue    = mapAndAdjustJoystickDeadBandValues(analogRead(33), false);
  data.switch1Value   = !digitalRead(18);
  data.switch2Value   = !digitalRead(19);
  data.potValue = map(analogRead(35), 0, 4095, 0, 180); // Read the pot, map the reading from [0, 4095] to [0, 180]
  
  FlipFlop();

  unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) {
    Serial.println("Error Receiving the data");
  }

  if (signal_send){
    esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &data, sizeof(data));
      if (result != ESP_OK) {
        Serial.println("Error sending the data");
      }    
  }
  
  SerialDataPrint();
}

void SerialDataPrint()
{
  if ((millis() - time_prev >= DATA_PTIME))
  {
    time_prev = millis(); 
    //Print angle and rate of angle change
    Serial.print(gpsData.ax);
    Serial.print("\t");
    Serial.print(gpsData.ay);
    Serial.print("\t");
    Serial.print(gpsData.az);
    Serial.print("\t");
    Serial.print(gpsData.gx);
    Serial.print("\t");
    Serial.print(gpsData.gy);
    Serial.print("\t");
    Serial.print(gpsData.gz);
    Serial.print("\t");
    //Print ESC signal sent to each motor
    Serial.print(gpsData.thrust);
    Serial.print("\t");
    Serial.print(gpsData.flm);
    Serial.print("\t");
    Serial.print(gpsData.frm);
    Serial.print("\t");
    Serial.print(gpsData.rlm);
    Serial.print("\t");
    Serial.print(gpsData.rrm);
    Serial.print("\t");
    //Print Joystick value
    Serial.print(data.xAxisValue);
    Serial.print("\t");
    Serial.print(data.yAxisValue);
    Serial.print("\t");
    //Print GPS data
    Serial.print(gpsData.LatitudeValue);
    Serial.print("\t");
    Serial.print(gpsData.LongtitudeValue);
    Serial.print("\t");
    Serial.print(gpsData.AltitudeValue);
    Serial.print("\t");
    Serial.print(gpsData.HourValue);
    Serial.print("\t");
    Serial.print(gpsData.MinuteValue);
    Serial.print("\t");
    Serial.print(gpsData.SecondValue);
    Serial.print("\t");
    Serial.println();  
  }
}