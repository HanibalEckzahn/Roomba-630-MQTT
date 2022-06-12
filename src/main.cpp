#include <Arduino.h>

#include <config.h>

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <SoftwareSerial.h>
#include <PubSubClient.h>
#include <TimeLib.h>

#include <EspMQTTClient.h>
#include <Roomba.h>



SoftwareSerial mySerial(SERIAL_RX, SERIAL_TX); // (RX, TX. inverted, buffer)

String ClientIP;

int bautRaid = 115200;



//webServer
ESP8266WebServer server(80);
MDNSResponder mdns;
WiFiClient wifiClient;



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
}

void loop() {
  if (mySerial.available()){
    Serial.print(mySerial.read());
  }
  server.handleClient();
}

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

