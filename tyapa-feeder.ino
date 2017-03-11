#include <ESP8266WiFi.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiType.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>

#include <PubSubClient.h>

#include <Stepper2.h>
#include "config.h"
#include "ctype.h"

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;
const char* mqttServer = MQTT_SERVER;
const char* mqttClientId = MQTT_CLIENT;
const char* mqttUser = MQTT_USER;
const char* mqttPass = MQTT_PASS;
const char* mqttSubTopic = MQTT_SUB_TOPIC;
const char* mqttPubTopic = MQTT_PUB_TOPIC;
const char* mqttPubMessage = "Yum yum yum!";
const int rpm = 15; // max rpm on 28BYJ-48
int pinOut[4] = { D1, D2, D5, D6 };
long lastMsg = 0;
char msg[50];
int value = 0;

Stepper2 myStepper(pinOut);
WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  setupMqtt();
  setupMotor();
  setupWifi();
}

void setupMotor() {
  myStepper.setSpeed(rpm);
}

void setupWifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    Serial.print(WiFi.status());
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setupMqtt() {
  client.setServer(mqttServer, 1883);
  client.setCallback(callback);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqttClientId, mqttUser, mqttPass)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe(mqttSubTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Serial print incoming message
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  
  // Perform an action
  if (isDigit((char)payload[0])) {
    int amount = payload[0] - '0';
    turn(amount);
    client.publish(mqttPubTopic, mqttPubMessage);
    
    Serial.print("Publish message: ");
    Serial.println(mqttPubTopic);
    Serial.print(mqttPubMessage);
    Serial.println();
  }
}

void turn(int fullTurns) {
  Serial.print("Turning motor to ");
  Serial.print(fullTurns);
  Serial.println();
  myStepper.setDirection(0);
  myStepper.turn(fullTurns);
  myStepper.stop();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  client.loop();
}
