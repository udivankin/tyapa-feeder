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
#include <EEPROM.h>
#include <PubSubClient.h>
#include <TimeAlarms.h>

#include "config.h"
#include "ctype.h"

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* mqttServer = MQTT_SERVER;
const int mqttPort = MQTT_PORT;
const char* mqttClientId = MQTT_CLIENT_ID;
const char* mqttUser = MQTT_USER;
const char* mqttPass = MQTT_PASSWORD;
const char* mqttSubTopicFeed = MQTT_SUB_TOPIC_FEED;
const char* mqttSubTopicSyncTime = MQTT_SUB_TOPIC_SYNC_TIME;
const char* mqttSubTopicSetTimers = MQTT_SUB_TOPIC_SET_TIMERS;
const char* mqttSubTopicGetStatus = MQTT_SUB_TOPIC_GET_STATUS;
const char* mqttPubTopicFeedCallback = MQTT_PUB_TOPIC_FEED_CALLBACK;
const char* mqttPubTopicDebug = MQTT_PUB_TOPIC_DEBUG;
const char* mqttPubTopicGetTime = MQTT_PUB_TOPIC_GET_TIME;
const char* mqttPubMessage = "Yum yum yum!";
const int motor_pin = D2;
const int button_pin = D1;
const int ON = LOW;
const int OFF = HIGH;
const int max_long_value = 2147483647;
const int min_motor_revolution_duration = 1000;
const int motor_revolution_duration = 1500;
int button_state = OFF; // LOW means pressed
int motor_state = OFF;  // LOW means stopped
long motor_start_time = max_long_value;
long motor_safe_stop_time = max_long_value;
int motor_target_revolutions_count = 0;

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  EEPROM.begin(12);
  Serial.begin(9600);
  setupButton();
  setupMotor();
  setupMqtt();
  setupWifi();
}

void setupAlarm() {
  // Read timers from EEPROM to set the alarm
  for (int i = 0; i < 6; i++) {
    int h = EEPROM.read((i * 2));
    int m = EEPROM.read((i * 2) + 1);
    Alarm.alarmRepeat(h, m, 0, feedOnce);
    Serial.print("Alarm is set to ");
    Serial.print(h);
    Serial.print(":");
    Serial.print(m);
    Serial.print("\n");
  }
  // Set two alarms for sync time request
  Alarm.alarmRepeat(23, 55, 0, publishTimeRequest);
  Alarm.alarmRepeat(11, 55, 0, publishTimeRequest);
  // And one to reboot after certain period of time (604800 seconds = 1 week)
  Alarm.timerOnce(604800, espRestart);
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

  Serial.print(" Connected, IP address: ");
  Serial.println(WiFi.localIP());
}

void setupMqtt() {
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttHandler);
}

void subscribeMqtt() {
  client.subscribe(mqttSubTopicFeed);
  client.subscribe(mqttSubTopicSyncTime);
  client.subscribe(mqttSubTopicSetTimers);
  client.subscribe(mqttSubTopicGetStatus);
  
  // Time request is sent once after successful mqtt connection, then twice daily
  publishTimeRequest();
}

void publishTimeRequest() {
  char sendBuffer[9];
  Serial.println("Sending sync time request");
  snprintf(sendBuffer, sizeof sendBuffer, "%02d:%02d:%02d", hour(), minute(), second());
  // Send sync time request with current time in payload in debug purposes
  client.publish(mqttPubTopicGetTime, sendBuffer);
}

void publishStatus() {
  char sendBuffer[48];
  snprintf(sendBuffer, sizeof sendBuffer, "WiFi: %s, IP: %s, Time: %02d:%02d:%02d", ssid, WiFi.localIP().toString().c_str(), hour(), minute(), second());
  Serial.print("GetStatus: ");
  Serial.println(sendBuffer);
  // Send hardware status
  client.publish(mqttPubTopicDebug, sendBuffer);
}

void feed(byte* payload, unsigned int length) {
  static char* msgStart = "Starting Feed... ";
  static char* msgFail = "Feed failed, incorrect payload size";
  
  Serial.print(msgStart);
  
  if (length == 1) {
    int amount = (int)payload[0];
    motor_target_revolutions_count = amount;
    motor_safe_stop_time = millis() + (motor_revolution_duration * amount);
    startMotor();
    Serial.print("Amount is ");
    Serial.println(amount);
    Serial.print("Motor should stop no later than ");
    Serial.println(motor_safe_stop_time);
    client.publish(mqttPubTopicDebug, "Feed ok");
  } else {
    Serial.println(msgFail);
    client.publish(mqttPubTopicDebug, msgFail);
  }
}

void feedOnce() {
  Serial.print("FeedOnce... ");
  static byte payload[] = {1};
  feed(payload, 1);
}

void stopMotor() {
  motor_state = OFF;
  motor_target_revolutions_count = 0;
  motor_safe_stop_time = max_long_value; // reset motor_safe_stop_time to the default value
  digitalWrite(motor_pin, motor_state);
  Serial.print("Stopping motor... ");
  Serial.println(millis());
}

void startMotor() {
  motor_state = ON;
  motor_start_time = millis();
  digitalWrite(motor_pin, motor_state);
  Serial.print("Starting motor... ");
  Serial.println(motor_start_time);
}

void espRestart() {
  delay(500);
  ESP.restart();
}

void setTimers(byte* payload, unsigned int length) {
  static char* msgStart = "Setting timers... ";
  static char* msgSuccess = "SetTimers ok, rebooting...";
  static char* msgFail = "SetTimer failed, incorrect payload size";

  Serial.print(msgStart);
  
  // Vaildate payload, should be 6 timers, each with 2 bytes, hours and minutes
  if (length == 12) {
    for (int i = 0; i < 6; i++) {
      EEPROM.write((i * 2), payload[(i * 2)]);
      EEPROM.write((i * 2) + 1, payload[(i * 2) + 1]);
    }
    EEPROM.commit();
    Serial.println(msgSuccess);
    client.publish(mqttPubTopicDebug, msgSuccess);
    // Avoid keeping connection on server
    client.disconnect();
    // The easiest way to setup timers is to reboot after timers change
    espRestart();
  } else {
    Serial.print(msgFail);
    Serial.println(length);
    client.publish(mqttPubTopicDebug, msgFail);
  }
}

void syncTime(byte* payload, unsigned int length) {
  static char* msgStart = "Sync time start... ";
  static char* msgFail = "SyncTime failed, incorrect payload size";
    
  Serial.print(msgStart);
  
  // Validate payload, should have 6 bytes
  if (length == 6) {
    // Set current time
    setTime(payload[0], payload[1], payload[2], payload[3], payload[4], payload[5]); // hr, min, sec, day, month, year
    char sendBuffer[32];
    snprintf(sendBuffer, sizeof sendBuffer, "SyncTime ok, %02d:%02d:%02d %02d.%02d.%02d", payload[0], payload[1], payload[2], payload[3], payload[4], payload[5]);
    Serial.println(sendBuffer);
    client.publish(mqttPubTopicDebug, sendBuffer);
    // Alarm timers must be re-enabled after each setTime call
    setupAlarm();
  } else {
    Serial.print(msgFail);
    Serial.println(length);
    client.publish(mqttPubTopicDebug, msgFail);
  }
}

void mqttHandler(char* topic, byte* payload, unsigned int length) {
  static char* msgFail = "Command failed: unknown topic";
  
  // Serial print incoming message
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  if (strcmp(topic, mqttSubTopicFeed) == 0) {
    feed(payload, length);
  } else if (strcmp(topic, mqttSubTopicSetTimers) == 0) {
    setTimers(payload, length);
  } else if (strcmp(topic, mqttSubTopicSyncTime) == 0) {
    syncTime(payload, length);
  } else if (strcmp(topic, mqttSubTopicGetStatus) == 0) {
    publishStatus();
  } else {
    Serial.println(msgFail);
    client.publish(mqttPubTopicDebug, msgFail);
  }
}

void mqttConnectLoop() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection... ");
    
    // Attempt to connect
    if (client.connect(mqttClientId, mqttUser, mqttPass)) {
      Serial.println("Connected");
      // Once connected, publish an announcement + status
      publishStatus();
      // And (re)subscribe
      subscribeMqtt();
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Will try again in 5 seconds...");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void buttonLoop() {
  static int target_debounce_count = 10;
  int debounce_count = 0;
  int next_button_state = digitalRead(button_pin);
  
  // Prevent accidental button state readings
  while (debounce_count < target_debounce_count && next_button_state == OFF && button_state == ON) {
    delay(1);
    next_button_state = digitalRead(button_pin);
    debounce_count++;
  }

  // Register a revolution
  if (debounce_count >= target_debounce_count) {
    motor_target_revolutions_count--;
    client.publish(mqttPubTopicFeedCallback, mqttPubMessage);
    Serial.println("Feed detected... ");
    Serial.println(millis());
  }

  button_state = next_button_state;
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
    mqttConnectLoop();
  }

  client.loop();
  buttonLoop();
  motorLoop();

  Alarm.delay(33);
}
