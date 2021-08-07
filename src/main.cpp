#include <Arduino.h>
#include <ESP8266WiFi.h>  // wifi board
#include <PubSubClient.h> // MQTT
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "Credential.h"   // id wifi + mqtt

#define PIN_DHT 0 // relay board linked on GPIO0
#define ESP_NAME "ESP_DHT22"

const int N = 15;
const int period = 1000 * 60 * N; // (1 minute = 1 s * 60) * N minutes

DHT dht(PIN_DHT, DHT22);
long lastMeasure = 0;

WiFiClient espClient;
PubSubClient clientMQTT(mqttServer, mqttPort, espClient);

void blinkLED(int nb)
{
  for (int i = 0; i < nb; i++)
  {
    digitalWrite(LED_BUILTIN, LOW);
    delay(60);
    digitalWrite(LED_BUILTIN, HIGH);

    if (i < nb - 1)
      delay(100);
  }
}

void connectWifi()
{
  // wifi
  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.begin(ssid, password);

    if (WiFi.waitForConnectResult() != WL_CONNECTED)
      ;

    // WiFi connected -> LED_BUILTIN shutdown
    blinkLED(5);
  }
  delay(100);
  // mqtt
  if (!clientMQTT.connected())
  {
    if (clientMQTT.connect(ESP_NAME, mqttUser, mqttPassword))
      ;
    else
    {
      delay(5e3);
    }
  }
  delay(100);
}

void publish()
{
  // Publishes new temperature and humidity every 15 mn (15*60*1000)
  if (millis() - lastMeasure > period)
  {
    lastMeasure = millis();
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t))
    {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

    // Computes temperature values in Celsius
    float hic = dht.computeHeatIndex(t, h, false);
    static char temperatureTemp[7];
    dtostrf(hic, 6, 2, temperatureTemp);

    static char humidityTemp[7];
    dtostrf(h, 6, 2, humidityTemp);

    // Publishes Temperature and Humidity values
    clientMQTT.publish("jardin/temperature", temperatureTemp);
    clientMQTT.publish("jardin/humidity", humidityTemp);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(100);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  dht.begin();
  delay(100);
  // init wifi
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(ESP_NAME);
  delay(100);
  // init mqtt
  clientMQTT.setServer(mqttServer, mqttPort);
  delay(100);
}

void loop()
{
  if (!clientMQTT.connected())
  {
    connectWifi();
  }
  clientMQTT.loop();
}