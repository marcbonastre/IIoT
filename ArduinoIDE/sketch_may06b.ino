#include <ArduinoJson.h>  // https://arduinojson.org/
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient
#include <WiFi.h>
#include <Wire.h>
#include "DHT.h"

#define LED_STATUS 25
#define DHTPIN 4     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);

// Define new subscription topics here
#define COMMAND_TOPIC "comandoLED"
#define TEST_TOPIC "test"

// Replace the next variables with your Wi-Fi SSID/Password
const char *WIFI_SSID = "AulaAutomatica";
const char *WIFI_PASSWORD = "ticsFcim";
char macAddress[18];

const char *MQTT_BROKER_IP = "iiot-upc.gleeze.com";
const int MQTT_PORT = 1883;
const char *MQTT_USER = "iiot-upc";
const char *MQTT_PASSWORD = "cim2020";
const bool RETAINED = true;
const int QoS = 0; // Quality of Service for the subscriptions

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  Serial.begin(9600); // Starts the serial communication
  Serial.println("\nBooting device...");

  pinMode(LED_STATUS, OUTPUT); // Pinout as output

  mqttClient.setServer(MQTT_BROKER_IP,
                       MQTT_PORT); // Connect the configured mqtt broker
  mqttClient.setCallback(
      callback); // Prepare what to do when a message is recieved

  connectToWiFiNetwork(); // Connects to the configured network
  connectToMqttBroker();  // Connects to the configured mqtt broker
  setSubscriptions();     // Subscribe defined topics

  dht.begin();
}

void loop() {
  checkConnections(); // We check the connection every time

  // Publish every 2 seconds
  static int nowTime = millis();
  static int startTime = 0;
  static int elapsedTime = 0;
  nowTime = millis();
  elapsedTime = nowTime - startTime;
  if (elapsedTime >= 2000) {
    //publishIntNumber();   // Publishes an int number
    //publishFloatNumber(); // Publishes a float number
    //publishString();      // Publishes string
    //publishSmallJson();   // Publishes a small json
    //publishBigJson();     // Publishes a big json
    publishTemp();
    publishESP32data();
    startTime = nowTime;
  }
}

/* Additional functions */
void setSubscriptions() {
  subscribe(COMMAND_TOPIC);
  subscribe(TEST_TOPIC);
}

void subscribe(char *newTopic) {
  const String topicStr = createTopic(newTopic);
  const char *topic = topicStr.c_str();
  mqttClient.subscribe(topic, QoS);
  Serial.println("Client MQTT subscribed to topic: " + topicStr +
                 " (QoS:" + String(QoS) + ")");
}

/* Additional functions */
void publishIntNumber() {
  static int counter = 0;
  static const String topicStr = createTopic("int_number");
  static const char *topic = topicStr.c_str();

  counter++;

  mqttClient.publish(topic, String(counter).c_str(), RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(counter));
}

void publishFloatNumber() {
  static float counter = 0;
  static const String topicStr = createTopic("float_number");
  static const char *topic = topicStr.c_str();

  counter = counter + 0.1;

  mqttClient.publish(topic, String(counter).c_str(), RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(counter));
}

void publishTemp() {
  static float temp = 0;
  static const String topicStr = createTopic("Temp");
  static const char *topic = topicStr.c_str();

  temp = dht.readTemperature();

  mqttClient.publish(topic, String(temp).c_str(), RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(temp));
}

void publishESP32data() {
  static float temp = 0;
  static float hum = 0;
  static const String topicStr = createTopic("ESP32");
  static const char *topic = topicStr.c_str();

  temp = dht.readTemperature();
  hum = dht.readHumidity();

  StaticJsonDocument<128> doc; // Create JSON document of 128 bytes
  char buffer[128]; // Create the buffer where we will print the JSON document
                    // to publish through MQTT

  JsonObject values1 =
      doc.createNestedObject("values1"); // We can add another Object
  values1["t"] = temp;
  values1["h"] = hum;

  // Serialize the JSON document to a buffer in order to publish it
  serializeJson(doc, buffer);
  mqttClient.publish(topic, buffer, RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(buffer));
}

void publishString() {
  static int counter = 0;
  static const String topicStr = createTopic("string");
  static const char *topic = topicStr.c_str();

  counter++;
  String text = "this is a text with dynamic numbers " + String(counter);

  mqttClient.publish(topic, text.c_str(), RETAINED);
  Serial.println(" <= " + String(topic) + ": " + text);
}

void publishSmallJson() {
  static const String topicStr = createTopic("small_json");
  static const char *topic = topicStr.c_str();

  StaticJsonDocument<128> doc; // Create JSON document of 128 bytes
  char buffer[128]; // Create the buffer where we will print the JSON document
                    // to publish through MQTT

  doc["device"] = "ESP32"; // Add names and values to the JSON document
  doc["sensor"] = "DHT22";
  JsonObject values1 =
      doc.createNestedObject("values1"); // We can add another Object
  values1["t"] = 19.30;
  values1["h"] = 78;

  JsonArray values2 = doc.createNestedArray("values2"); // We can add an Array
  values2.add(1); // Inside the array we can add new values to "values1"
  values2.add(2); // From this number on, it will not be printed since it
                  // overpasses 128 bytes
  values2.add(3);
  values2.add(4);

  // Serialize the JSON document to a buffer in order to publish it
  serializeJson(doc, buffer);
  mqttClient.publish(topic, buffer, RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(buffer));
}

void publishBigJson() {
  static const String topicStr = createTopic("big_json");
  static const char *topic = topicStr.c_str();

  DynamicJsonDocument doc(2048); // Create JSON document
  char buffer[2048]; // Create the buffer where we will print the JSON document
                     // to publish through MQTT

  doc["device"] = "ESP32"; // Add names and values to the JSON document
  doc["sensor"] = "DHT22";
  JsonObject values1 = doc.createNestedObject("values1"); // Add another Object
  values1["t"] = 19.30;
  values1["h"] = 78;

  JsonArray values2 = doc.createNestedArray("values2"); // We can add an Array
  values2.add(1); // Inside the array we can add new values to "values"
  values2.add(2);
  values2.add(3);
  values2.add(4);
  values2.add(5);
  values2.add(6);
  values2.add(7);
  values2.add(8);
  values2.add(9);
  values2.add(10);
  values2.add(11);
  values2.add(12);

  // Serialize the JSON document to a buffer in order to publish it
  size_t n = serializeJson(doc, buffer);
  mqttClient.publish_P(topic, buffer, n); // No RETAINED option
  Serial.println(" <= " + String(topic) + ": " + String(buffer));
}

void callback(char *topic, byte *payload, unsigned int length) {
  // Register all subscription topics
  static const String cmdTopicStr = createTopic(COMMAND_TOPIC);
  static const String testTopicStr = createTopic(TEST_TOPIC);

  String msg = unwrapMessage(payload, length);
  Serial.println(" => " + String(topic) + ": " + msg);

  // What to do in each topic case?
  if (String(topic) == cmdTopicStr) {
    cmdLed(msg);
  } else if (String(topic) == testTopicStr) {
    // Do some other stuff
  } else {
    Serial.println("[WARN] - '" + String(topic) +
                   "' topic was correctly subscribed but not defined in the "
                   "callback function");
  }
}

String unwrapMessage(byte *message, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) { // Unwraps the string message
    msg += (char)message[i];
  }
  return msg;
}

String createTopic(char *topic) {
  String topicStr = String(macAddress) + " Marc /" + topic;
  return topicStr;
}

void connectToWiFiNetwork() {
  Serial.print(
      "Connecting with Wi-Fi: " +
      String(WIFI_SSID)); // Print the network which you want to connect
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".."); // Connecting effect
  }
  Serial.print("..connected!  (ip: "); // After being connected to a network,
                                       // our ESP32 should have a IP
  Serial.print(WiFi.localIP());
  Serial.println(")");
  String macAddressStr = WiFi.macAddress().c_str();
  strcpy(macAddress, macAddressStr.c_str());
}

void connectToMqttBroker() {
  Serial.print(
      "Connecting with MQTT Broker:" +
      String(MQTT_BROKER_IP));    // Print the broker which you want to connect
  mqttClient.connect(macAddress, MQTT_USER, MQTT_PASSWORD);// Using unique mac address from ESP32
  while (!mqttClient.connected()) {
    delay(500);
    Serial.print("..");             // Connecting effect
    mqttClient.connect(macAddress); // Using unique mac address from ESP32
  }
  Serial.println("..connected! (ClientID: " + String(macAddress) + ")");
}

void checkConnections() {
  if (mqttClient.connected()) {
    mqttClient.loop();
  } else { // Try to reconnect
    Serial.println("Connection has been lost with MQTT Broker");
    if (WiFi.status() != WL_CONNECTED) { // Check wifi connection
      Serial.println("Connection has been lost with Wi-Fi");
      connectToWiFiNetwork(); // Reconnect Wifi
    }
    connectToMqttBroker(); // Reconnect Server MQTT Broker
  }
}

void cmdLed(String cmd) {
  if (cmd == "On") {
    digitalWrite(LED_STATUS, HIGH);
  } else{
    digitalWrite(LED_STATUS, LOW);
  }
  }
