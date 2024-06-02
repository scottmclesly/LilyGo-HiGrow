// WiFi credentials
#define WIFI_SSID "SafetyMeeting"
#define WIFI_PASSWORD "mowgliknowsme"
#define BLYNK_AUTH_TOKEN "w00Dhfjd54vdHtNioQoIOLpN72UjdKnk"

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <BH1750.h>
#include <Adafruit_BME280.h>
#include <BlynkSimpleEsp32.h>
#include <algorithm>
#include <iostream>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPDash.h>
#include <Button2.h>
#include <cJSON.h>
#include <OneWire.h>
#include <LoRa.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_NeoPixel.h>

// Virtual Pins
#define V_PIN_SALT        V0
#define V_PIN_BATTERY     V1
#define V_PIN_SOIL        V2
#define V_PIN_BH1750      V3
#define V_PIN_DHT_TEMP    V4
#define V_PIN_DHT_HUM     V5
#define V_PIN_PUMP        V6

// Define sensor pins
#define DHT_PIN 16
#define BAT_ADC 33
#define SOIL_PIN 32
#define LED_PIN 13
#define SALT_PIN 34

// DHT sensor type
#define DHT_TYPE DHT11

DHT dht(DHT_PIN, DHT_TYPE);
BH1750 lightMeter;
Adafruit_BME280 bmp;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;

typedef struct {
    uint32_t timestamp;     /**< time is in milliseconds */
    float temperature;      /**< temperature is in degrees centigrade (Celsius) */
    float light;            /**< light in SI lux units */
    float pressure;         /**< pressure in hectopascal (hPa) */
    float humidity;         /**< humidity in percent */
    float altitude;         /**< altitude in m */
    float voltage;          /**< voltage in volts (V) */
    uint8_t soli;           /**< percentage of soil moisture */
    uint8_t salt;           /**< percentage of salt */
} higrow_sensors_event_t;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", BLYNK_AUTH_TOKEN, "")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer("blynk-cloud.com", 1883);
  dht.begin();
  Wire.begin();
  lightMeter.begin();
  bmp.begin(0x76);  // I2C address for BME280 is 0x76
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 60000) {
    lastMsg = now;

    // Create the struct to hold sensor data
    higrow_sensors_event_t sensorData = {0};

    // Capture sensor data
    sensorData.timestamp = millis();
    sensorData.temperature = dht.readTemperature();
    sensorData.humidity = dht.readHumidity();
    sensorData.light = lightMeter.readLightLevel();
    sensorData.pressure = bmp.readPressure() / 100.0F;
    sensorData.altitude = bmp.readAltitude(1013.25);
    sensorData.soli = map(analogRead(SOIL_PIN), 0, 4095, 100, 0);
    sensorData.salt = analogRead(SALT_PIN);
    sensorData.voltage = analogRead(BAT_ADC) * (3.3 / 4095.0) * 2.0;  // Assuming a voltage divider

    // Publish sensor data to Blynk using MQTT
    client.publish(String("v1/" + String(BLYNK_AUTH_TOKEN) + "/data/" + String(V_PIN_DHT_TEMP)).c_str(), String(sensorData.temperature).c_str());
    client.publish(String("v1/" + String(BLYNK_AUTH_TOKEN) + "/data/" + String(V_PIN_DHT_HUM)).c_str(), String(sensorData.humidity).c_str());
    client.publish(String("v1/" + String(BLYNK_AUTH_TOKEN) + "/data/" + String(V_PIN_BH1750)).c_str(), String(sensorData.light).c_str());
    client.publish(String("v1/" + String(BLYNK_AUTH_TOKEN) + "/data/" + String(V_PIN_BATTERY)).c_str(), String(sensorData.voltage).c_str());
    client.publish(String("v1/" + String(BLYNK_AUTH_TOKEN) + "/data/" + String(V_PIN_SOIL)).c_str(), String(sensorData.soli).c_str());
    client.publish(String("v1/" + String(BLYNK_AUTH_TOKEN) + "/data/" + String(V_PIN_SALT)).c_str(), String(sensorData.salt).c_str());

    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);

    // Go to deep sleep for 60 seconds
    esp_sleep_enable_timer_wakeup(60000000);
    esp_deep_sleep_start();
  }
}