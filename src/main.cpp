#include "SdsDustSensor.h"
#include <WiFi.h>
#include "MQ135.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Adafruit_AHTX0.h>
#include <TFT_eSPI.h> // Graphics and font library for ST7789 driver chip
#include <SPI.h>
#include <Wire.h>

// Display settings
#define SCREEN_WIDTH 135 // TFT display width in pixels
#define SCREEN_HEIGHT 240 // TFT display height in pixels

// Pin definitions
// SDS011 (UART communication)
#define SDS011_RX_PIN int8_t 25
#define SDS011_TX_PIN int8_t 26

// MQ135 (Analog signal)
#define MQ135_PIN 33 // Analog input pin

// DHT11 (I2C communication)
#define DHTPIN 21 // SDA pin (I2C)

Adafruit_AHTX0 dht;

// WiFi and MQTT settings
const char *ssid = "yyue";     // WiFi name
const char *pass = "123456789000";       // WiFi password
#define MQTT_SERV "io.adafruit.com"
#define MQTT_PORT 1883
#define MQTT_NAME "yyue" // Adafruit IO username
#define MQTT_PASS "aio_koAP65f6z9lKWLhucp5IlCIReri5" // Adafruit IO key

// Create objects
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERV, MQTT_PORT, MQTT_NAME, MQTT_PASS);

// MQTT topics
Adafruit_MQTT_Publish AirQuality = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/feeds/cs147-team28.aqi");
Adafruit_MQTT_Publish Temperature = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/feeds/cs147-team28.temperature");
Adafruit_MQTT_Publish Humidity = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/feeds/cs147-team28.humidity");
Adafruit_MQTT_Publish PM10 = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/feeds/cs147-team28.pm10");
Adafruit_MQTT_Publish PM25 = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/feeds/cs147-team28.pm2-dot-5");
Adafruit_MQTT_Publish CO2 = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/feeds/cs147-team28.smoke");

// Sensor objects
SdsDustSensor sds(Serial1);
MQ135 gasSensor = MQ135(MQ135_PIN);

// Display object
TFT_eSPI tft = TFT_eSPI(); // Initialize TFT display

//define the CO2 initial value
float CO2_initial = 0;

void displayValues(float temperature, float humidity, float p25, float p10, float co2_ppm, float AQI)
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(TL_DATUM); // Top-left datum (0,0)

  // Set text size (adjust as needed)
  tft.setTextSize(2);

  // Display temperature
  tft.drawString("Temp: ", 0, 0);
  tft.drawFloat(temperature, 1, 80, 0);
  tft.drawString("C", 140, 0);

  // Display humidity
  tft.drawString("Humidity: ", 0, 30);
  tft.drawFloat(humidity, 1, 100, 30);
  tft.drawString("%", 160, 30);

  // Display PM2.5
  tft.drawString("PM2.5: ", 0, 60);
  tft.drawFloat(p25, 1, 80, 60);
  tft.drawString("ug/m3", 140, 60);

  // Display PM10
  tft.drawString("PM10: ", 0, 90);
  tft.drawFloat(p10, 1, 80, 90);
  tft.drawString("ug/m3", 140, 90);

  // Display CO2 concentration
  tft.drawString("CO2: ", 0, 120);
  tft.drawFloat(co2_ppm, 1, 80, 120);
  tft.drawString("ppm", 140, 120);

  // Display Air Quality Index
  tft.drawString("AQI: ", 0, 150);
  tft.drawFloat(AQI, 1, 100, 150);
}

void MQTT_connect()
{
  int8_t ret;
  if (mqtt.connected())
  {
    return;
  }
  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0)
  {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);
  }
  Serial.println("MQTT Connected!");
}

typedef struct {
    float Clow, Chigh;  // Concentration low and high
    int Ilow, Ihigh;    // AQI low and high
} Breakpoint;

Breakpoint get_pm25_breakpoint(float concentration) {
    if (concentration <= 12.0)
        return (Breakpoint){0.0, 12.0, 0, 50};
    else if (concentration <= 35.4)
        return (Breakpoint){12.1, 35.4, 51, 100};
    else if (concentration <= 55.4)
        return (Breakpoint){35.5, 55.4, 101, 150};
    else if (concentration <= 150.4)
        return (Breakpoint){55.5, 150.4, 151, 200};
    else if (concentration <= 250.4)
        return (Breakpoint){150.5, 250.4, 201, 300};
    else if (concentration <= 350.4)
        return (Breakpoint){250.5, 350.4, 301, 400};
    else if (concentration <= 500.4)
        return (Breakpoint){350.5, 500.4, 401, 500};
    else
        return (Breakpoint){500.5, 9999.9, 501, 999}; // Beyond AQI scale
}

int calculate_aqi(float concentration, Breakpoint bp) {
    return (int)(((bp.Ihigh - bp.Ilow) / (bp.Chigh - bp.Clow)) * (concentration - bp.Clow) + bp.Ilow);

}

void setup()
{
  // Initialize serial communication
  Serial.begin(9600);
  delay(10);

  // Initialize I2C bus
  Wire.begin();

  // Initialize TFT display
  tft.init();
  tft.setRotation(3); // Adjust rotation if needed (0-3)
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  Serial1.begin(9600, SERIAL_8N1, 26,27); // this line will begin Serial1 with given baud rate (9600 by default)
  sds.begin(); // this line will begin Serial1 with given baud rate (9600 by default)
  sds.setActiveReportingMode(); // ensures sensor is in 'active' reporting mode
  sds.wakeup();

  if (dht.begin()==0) {
    Serial.println("Could not find DHT11 sensor. Check wiring.");
    while (1) delay(10);
  }
  Serial.println("DHT11 sensor initialized.");

  // Connect to WiFi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // Initialize MQTT connection
  MQTT_connect();

  //wait 2 minute for the sensor to warm up
  delay(120000);

  //initialize the CO2 value
  CO2_initial = analogRead(MQ135_PIN);

}

void loop()
{
  MQTT_connect();

  // Read temperature and humidity from DHT20

  sensors_event_t humidity_event, temp_event;
  dht.getEvent(&humidity_event, &temp_event);
  float temperature = temp_event.temperature;
  float humidity = humidity_event.relative_humidity;
  if (isnan(temperature) || isnan(humidity))
  {
    Serial.println("Failed to read from DHT sensor!");
  }
  else
  {
    Serial.print("Temperature: ");
    Serial.println(temperature);
    Serial.print("Humidity: ");
    Serial.println(humidity);
  }

  // Read CO2 concentration from MQ135
  float current_co2 = analogRead(MQ135_PIN);
  if (current_co2 < CO2_initial)
  {
    CO2_initial = current_co2;
  }
  float co2_ppm = current_co2-CO2_initial;
  Serial.print("CO2 PPM: ");
  Serial.println(co2_ppm);

  // Read PM2.5 and PM10 from SDS011
  int error;
  float p10, p25;
  PmResult pm = sds.queryPm();
  if (pm.isOk())
  {
    p25 = pm.pm25;
    p10 = pm.pm10;
    Serial.print("PM2.5: ");
    Serial.println(p25);
    Serial.print("PM10: ");
    Serial.println(p10);
  }
  else
  {
    Serial.println("Failed to read from SDS011 sensor!");
  }

  // Determine Air Quality Index (AQI) based on PM2.5
  Breakpoint bp = get_pm25_breakpoint(p25);
  int AQI = calculate_aqi(p25, bp);

  // Publish data to Adafruit IO
  
  if (!Temperature.publish(temperature))
  {
    Serial.println("Failed to publish temperature");
  }
  if (!Humidity.publish(humidity))
  {
    Serial.println("Failed to publish humidity");
  }
  if (!PM25.publish(p25))
  {
    Serial.println("Failed to publish PM2.5");
  }
  if (!PM10.publish(p10))
  {
    Serial.println("Failed to publish PM10");
  }
  if (!CO2.publish(co2_ppm))
  {
    Serial.println("Failed to publish CO2");
  }
  if (!AirQuality.publish(AQI))
  {
    Serial.println("Failed to publish Air Quality Index");
  }

  // Update display with new values
  displayValues(temperature, humidity, p25, p10, co2_ppm, AQI);

  delay(12000); // Wait for 12 seconds before the next reading
}

