#include <Arduino.h>

#include <config.h>

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <SoftwareSerial.h>
#include <PubSubClient.h>
#include <TimeLib.h>

#include <EspMQTTClient.h>
#include "Roomba.h"

// Roomba setup
Roomba roomba(&Serial, Roomba::Baud115200);
SoftwareSerial mySerial(SERIAL_RX, SERIAL_TX); // (RX, TX. inverted, buffer)

String ClientIP;

int bautRaid = 115200;



//webServer
ESP8266WebServer server(80);
MDNSResponder mdns;
WiFiClient wifiClient;

//Handles the Roomba Comments
void roomba_distance(){
  mySerial.write(142);
  delay(50);
  mySerial.write(19);
  delay(50);
  if(mySerial.available()){
    Serial.println("...");
    Serial.print(Serial.read());
  }
  String data = String(Serial.read());
  Serial.println("Entfernung zurückgelegt"); // EN Distance traveled  
}

void roomba_wakeup(){
  digitalWrite(Wake_Pin,HIGH);
  delay(50);
  digitalWrite(Wake_Pin,LOW);
  delay(500);
  digitalWrite(Wake_Pin, HIGH);
}

void roomba_start(){
  roomba_wakeup();
  Serial.println("Starting");
  mySerial.write(128);
  delay(50);
  mySerial.write(131);
  delay(50);
  mySerial.write(135);
  Serial.println("Ich Startete mit dem Putzen!");
  //handle_root();
}


void roomba_max(){
  roomba_wakeup();
  Serial.println("Start");
  mySerial.write(128);
  delay(50);
  mySerial.write(131);
  delay(50);
  mySerial.write(136);
  Serial.println("Volle Putzkraft"); //EN: Full cleaner
}
void roomba_spot(){
  roomba_wakeup();
  mySerial.write(128);
  delay(50);
  mySerial.write(131);
  delay(50);
  mySerial.write(134);
}
void roomba_stop(){
  roomba_wakeup();
  mySerial.write(128);
  delay(50);
  mySerial.write(131);
  delay(50);
  mySerial.write(133);
}

void roomba_dock(){
  roomba_wakeup();
  mySerial.write(128);
  delay(50);
  mySerial.write(131);
  delay(50);
  mySerial.write(143);
}

void esp_restart(){
  ESP.restart();
}

// Roomba state
typedef struct {
  // Sensor values
  int16_t distance;
  uint8_t chargingState;
  uint16_t voltage;
  int16_t current;
  // Supposedly unsigned according to the OI docs, but I've seen it
  // underflow to ~65000mAh, so I think signed will work better.
  int16_t charge;
  uint16_t capacity;

  // Derived state
  bool cleaning;
  bool docked;

  int timestamp;
  bool sent;
} RoombaState;

RoombaState roombaState = {};
// Roomba sensor packet
uint8_t roombaPacket[100];
uint8_t sensors[] = {
  Roomba::SensorDistance, // PID 19, 2 bytes, mm, signed
  Roomba::SensorChargingState, // PID 21, 1 byte
  Roomba::SensorVoltage, // PID 22, 2 bytes, mV, unsigned
  Roomba::SensorCurrent, // PID 23, 2 bytes, mA, signed
  Roomba::SensorBatteryCharge, // PID 25, 2 bytes, mAh, unsigned
  Roomba::SensorBatteryCapacity // PID 26, 2 bytes, mAh, unsigned
};

float readADC(int samples) {
  // Basic code to read from the ADC
  int adc = 0;
  for (int i = 0; i < samples; i++) {
    delay(1);
    adc += analogRead(A0);
  }
  adc = adc / samples;
  float mV = adc * ADC_VOLTAGE_DIVIDER;
  return mV;
}
bool parseRoombaStateFromStreamPacket(uint8_t *packet, int length, RoombaState *state) {
  state->timestamp = millis();
  int i = 0;
  while (i < length) {
    switch(packet[i]) {
      case Roomba::Sensors7to26: // 0
        i += 27;
        break;
      case Roomba::Sensors7to16: // 1
        i += 11;
        break;
      case Roomba::SensorVirtualWall: // 13
        i += 2;
        break;
      case Roomba::SensorDistance: // 19
        state->distance = packet[i+1] * 256 + packet[i+2];
        i += 3;
        break;
      case Roomba::SensorChargingState: // 21
        state->chargingState = packet[i+1];
        i += 2;
        break;
      case Roomba::SensorVoltage: // 22
        state->voltage = packet[i+1] * 256 + packet[i+2];
        i += 3;
        break;
      case Roomba::SensorCurrent: // 23
        state->current = packet[i+1] * 256 + packet[i+2];
        i += 3;
        break;
      case Roomba::SensorBatteryCharge: // 25
        state->charge = packet[i+1] * 256 + packet[i+2];
        i += 3;
        break;
      case Roomba::SensorBatteryCapacity: //26
        state->capacity = packet[i+1] * 256 + packet[i+2];
        i += 3;
        break;
      case Roomba::SensorBumpsAndWheelDrops: // 7
        i += 2;
        break;
      case 128: // Unknown
        i += 2;
        break;
      default:
        return false;
        break;
    }
  }
  return true;
}

void readSensorPacket() {
  uint8_t packetLength;
  bool received = roomba.pollSensors(roombaPacket, sizeof(roombaPacket));
  if (received) {
    RoombaState rs = {};
    bool parsed = parseRoombaStateFromStreamPacket(roombaPacket, packetLength, &rs);
    if (parsed) {
      roombaState = rs;
      roombaState.cleaning = false;
      roombaState.docked = false;
      if (roombaState.current < -400) {
        roombaState.cleaning = true;
      } else if (roombaState.current > -50) {
        roombaState.docked = true;
      }
    } 
  }
}











void setup() {
  Serial.begin(bautRaid);
  mySerial.begin(bautRaid);
  pinMode(SERIAL_RX, INPUT);
  pinMode(SERIAL_TX, OUTPUT);
  pinMode(Wake_Pin, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(Wake_Pin, LOW);
  WiFi.begin(SSID, SSID_PASSWORD);
  WiFi.setHostname(HOSTNAME);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connectetd");

  server.begin();



  roomba.start();
  delay(100);

  // Reset stream sensor values
  roomba.stream({}, 0);
  delay(100);
 

  // Request sensor stream
  roomba.stream(sensors, sizeof(sensors));
}

void loop() {
  if (mySerial.available()){
    Serial.print(mySerial.read());
  }
  server.handleClient();
}
