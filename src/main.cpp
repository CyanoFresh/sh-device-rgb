#include <ArduinoJson.h>
#include <AsyncMqttClient.h>
// #include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>

#define WIFI_SSID "Solomaha"
#define WIFI_PASSWORD "solomakha21"

#define MQTT_HOST IPAddress(192, 168, 1, 230)
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

int rainbow = 0;

struct {
  unsigned int mode = 0;  // 0 for static color, 1 for rainbow
  unsigned int red = 0;
  unsigned int green = 0;
  unsigned int blue = 0;
  unsigned int speed = 0;
  unsigned int brightness = 0;
} config;

typedef struct {
  int r;
  int g;
  int b;
} rgb;

rgb hsv2rgb(int h) {
  double hh, p, q, t, ff;
  long i;
  rgb out;

  hh = h / 240.0;
  i = (long)hh;
  ff = hh - i;
  p = 0;
  q = (1.0 - (1.0 * ff));
  t = ff;

  switch (i) {
    case 0:
      out.r = 1023;
      out.g = (int)(t * 1023);
      out.b = (int)(p * 1023);
      break;
    case 1:
      out.r = (int)(q * 1023);
      out.g = 1023;
      out.b = (int)(p * 1023);
      break;
    case 2:
      out.r = (int)(p * 1023);
      out.g = 1023;
      out.b = (int)(t * 1023);
      break;

    case 3:
      out.r = (int)(p * 1023);
      out.g = (int)(q * 1023);
      out.b = 1023;
      break;
    case 4:
      out.r = (int)(t * 1023);
      out.g = (int)(p * 1023);
      out.b = 1023;
      break;
    case 5:
    default:
      out.r = 1023;
      out.g = (int)(p * 1023);
      out.b = (int)(q * 1023);
      break;
  }
  return out;
}

void rainbowTick(int brightness) {
  rgb color = hsv2rgb(rainbow);

  double ratio = brightness / 100.0;

  int red = color.r * ratio;
  int green = color.g * ratio;
  int blue = color.b * ratio;

  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);

  if (rainbow == 1439) {
    rainbow = 0;
  } else {
    rainbow++;
  }
}

void stopRainbow() {
  // rainbow = 0;
  rainbowTimer.detach();
}

void startRainbow(int speed, int brightness) {
  stopRainbow();
  rainbowTimer.attach_ms(speed, rainbowTick, brightness);
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
  mqttClient.subscribe("rgb/room1-rgb/set", 0);
  mqttClient.subscribe("devices/room1-rgb", 0);

  // Send initial state
  String sendPayload = "{\"mode\":" + String(config.mode) +
                       ",\"red\":" + String(config.red) +
                       ",\"green\":" + String(config.green) +
                       ",\"speed\":" + String(config.speed) +
                       ",\"brightness\":" + String(config.brightness) +
                       ",\"blue\":" + String(config.blue) + "}";
  mqttClient.publish("rgb/room1-rgb", 0, false, sendPayload.c_str());
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

    String sendPayload = "{\"mode\":0,\"red\":" + String(config.red) +
                         ",\"green\":" + String(config.green) +
                         ",\"blue\":" + String(config.blue) + "}";
    mqttClient.publish("rgb/room1-rgb", 0, false, sendPayload.c_str());
  } else if (root["mode"] == 1) {
    config.mode = 1;
    config.speed = root["speed"];
    config.brightness = root["brightness"];

    startRainbow(config.speed, config.brightness);

    String sendPayload = "{\"mode\":1,\"speed\":" + String(config.speed) +
                         ",\"brightness\":" + String(config.brightness) + "}";
    mqttClient.publish("rgb/room1-rgb", 0, false, sendPayload.c_str());
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
  mqttClient.setClientId("room1-rgb");
  mqttClient.setCredentials("device", "fdkhjsdfhjkhjkfsdjkldfshjklsjyghfhfgfd");

  connectToWifi();

  analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);
}

void loop() {}