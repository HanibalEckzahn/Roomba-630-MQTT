
#include "EspMQTTClient.h"
#include "string"


//Version
String roombotVersion = "0.1.3";
String WMode = "1";

//WIFI Settings
#define HOSTNAME "Roomba"
#define SSID "Alice im WLAN"
#define SSID_PASSWORD "string"

EspMQTTClient mqttClient(
  "SSID",
  "SSID_PASSWORD",
  "192.168.1.100",
  "HOSTNAME"
);

// WIFI AP 
const char *APssid = "Roombot";
const char *APpassword = "momo2021";

//MQTT Settings

#define MQTT_Server "192.168.x.x"
#define MQTT_PORT "1888"
#define MQTT_User "xxxx"
#define MQTT_Password "password"

//ESP-Pin Settings
#define SERIAL_RX     D5  // pin for SoftwareSerial RX
#define SERIAL_TX     D6  // pin for SoftwareSerial TX
#define Wake_Pin      D1  // pin for Wakeup Roomba/Reset

