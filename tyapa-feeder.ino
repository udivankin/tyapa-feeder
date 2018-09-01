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

#include "config.h"
#include "ctype.h"

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* mqttServer = MQTT_SERVER;
const int mqttPort = MQTT_PORT;
const char* mqttClientId = MQTT_CLIENT_ID;
const char* mqttUser = MQTT_USER;
const char* mqttPass = MQTT_PASSWORD;
const char* mqttSubTopic = MQTT_SUB_TOPIC;
const char* mqttPubTopicCallback = MQTT_PUB_CALLBACK_TOPIC;
const char* mqttPubTopicSuccess = MQTT_PUB_SUCCESS_TOPIC;
const char* mqttPubTopicFail = MQTT_PUB_FAIL_TOPIC;
const char* mqttPubMessage = "Yum yum yum!";
const char* mqttPubMessageCallback = "OK";
const int motor_pin = D2;
const int button_pin = D1;
const int ON = LOW;
const int OFF = HIGH;
const int target_debounce_count = 10;
const int max_long_value = 2147483647;
const int min_motor_revolution_duration = 1000;
const int motor_revolution_duration = 1500;
int button_state = OFF; // LOW means pressed
int motor_state = OFF; // LOW means pressed
long motor_start_time = max_long_value;
long motor_safe_stop_time = max_long_value;
int motor_target_revolutions_count = 0;

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(9600);
  setupButton();
  setupMotor();
  setupMqtt();
  setupWifi();
}

void setupMotor() {
  delay(10);
  digitalWrite(motor_pin, motor_state);
  pinMode(motor_pin, OUTPUT);
}

void setupButton() {
  pinMode(button_pin, INPUT_PULLUP);
  delay(10);
  button_state =  digitalRead(button_pin);
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
  client.setServer(mqttServer, mqttPort);
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
    motor_target_revolutions_count = amount;
    motor_safe_stop_time = millis() + ((motor_revolution_duration + 1) * amount);
    startMotor();
    Serial.println("\nMotor should stop no later than");
    Serial.println(motor_safe_stop_time);
    client.publish(mqttPubTopicCallback, mqttPubMessageCallback);
  }
}

void buttonLoop() {
  int debounce_count = 0;
  int next_button_state = digitalRead(button_pin);
  
  // Prevent accidental button state changes
  while (debounce_count < target_debounce_count && next_button_state == ON && button_state == OFF) {
    delay(1);
    next_button_state = digitalRead(button_pin);
    debounce_count++;
  }

  // Register a revolution
  if (debounce_count >= target_debounce_count) {
    motor_target_revolutions_count--;
    client.publish(mqttPubTopicSuccess, mqttPubMessage);
    Serial.println("\Feed detected... ");
    Serial.println(millis());
  }

  button_state = next_button_state;
  
  delay(1);
}

void stopMotor() {
  motor_state = OFF;
  motor_target_revolutions_count = 0;
  motor_safe_stop_time = max_long_value; // reset motor_safe_stop_time to the default value
  digitalWrite(motor_pin, motor_state);
  Serial.println("\nStopping motor... ");
  Serial.println(millis());
}

void startMotor() {
  motor_state = ON;
  motor_start_time = millis();
  digitalWrite(motor_pin, motor_state);
  Serial.println("Starting motor...");
  Serial.println(motor_start_time);
}

void motorLoop() {
  int current_time = millis();
  
  if (motor_state == ON) {
    // Turn of the motor on fail stop timer
    if (current_time > motor_safe_stop_time) {
      stopMotor();
      return;
    }
    // We've reached target rev count 
    if (motor_target_revolutions_count <= 0) {
      // Prolong run if it was running for a too little time
      if (current_time - motor_start_time < min_motor_revolution_duration) {
        motor_target_revolutions_count++;
        return;
      }
      stopMotor();
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  client.loop();
  buttonLoop();
  motorLoop();
}
