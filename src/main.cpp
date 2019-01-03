#include <ArduinoJson.h>
#include <AsyncMqttClient.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <FastLED.h>
#include <Ticker.h>

#define WIFI_SSID "Solomaha_2"
#define WIFI_PASSWORD "solomakha21"

#define MQTT_HOST IPAddress(192, 168, 1, 76)
#define MQTT_PORT 1883

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

Ticker rainbowTimer;

const int redPin = D1;
const int greenPin = D2;
const int bluePin = D5;

unsigned int red = 0;
unsigned int green = 0;
unsigned int blue = 0;

uint8_t rainbow = 0;

struct {
  unsigned int mode = 0;  // 0 for static color, 1 for rainbow
  unsigned int red = 0;
  unsigned int green = 0;
  unsigned int blue = 0;
} config;

void rainbowTick() {
  const CRGB& rgb = CHSV(rainbow, 255, 255);

  analogWrite(redPin, rgb.r);
  analogWrite(greenPin, rgb.g);
  analogWrite(bluePin, rgb.b);

  rainbow++;
}

void stopRainbow() {
  rainbow = 0;
  rainbowTimer.detach();
}

void startRainbow(unsigned int speed) {
  stopRainbow();
  rainbowTimer.attach_ms((20 - speed) * 10, rainbowTick);
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach();  // ensure we don't reconnect to MQTT while
                                // reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");

  // Subscribe
  mqttClient.subscribe("rgb/my-room_rgb/set", 0);

  // Send initial state
  String sendPayload = "{\"mode\":" + String(config.mode) +
                       ",\"red\":" + String(config.red) +
                       ",\"green\":" + String(config.green) +
                       ",\"blue\":" + String(config.blue) + "}";
  mqttClient.publish("rgb/my-room_rgb", 0, false, sendPayload.c_str());
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttMessage(char* topic, char* payload,
                   AsyncMqttClientMessageProperties properties, size_t len,
                   size_t index, size_t total) {
  StaticJsonBuffer<110> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(payload);

  if (root["mode"] == 0) {
    stopRainbow();

    config.mode = 0;
    config.red = root["red"];
    config.green = root["green"];
    config.blue = root["blue"];

    red = map(config.red, 0, 255, 0, 1023);
    green = map(config.green, 0, 255, 0, 1023);
    blue = map(config.blue, 0, 255, 0, 1023);

    analogWrite(redPin, red);
    analogWrite(greenPin, green);
    analogWrite(bluePin, blue);

    String sendPayload = "{\"mode\":" + String(config.mode) +
                         ",\"red\":" + String(config.red) +
                         ",\"green\":" + String(config.green) +
                         ",\"blue\":" + String(config.blue) + "}";
    mqttClient.publish("rgb/my-room_rgb", 0, false, sendPayload.c_str());
  } else if (root["mode"] == 1) {
    config.mode = 1;
    startRainbow(root["speed"]);
  }
}

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  Serial.begin(115200);
  Serial.println();
  Serial.println();

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();

  analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);
}

void loop() {}