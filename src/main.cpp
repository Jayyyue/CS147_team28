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
const char *ssid = "";     // put WiFi name in this variable
const char *pass = "";       // put WiFi password in this variable
#define MQTT_SERV "io.adafruit.com"
#define MQTT_PORT 1883
#define MQTT_NAME "yyue" // Adafruit IO username
#define MQTT_PASS "aio_koAP65f6z9lKWLhucp5IlCIReri5" // Adafruit IO key

// Create objects
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERV, MQTT_PORT, MQTT_NAME, MQTT_PASS);

// MQTT topics
Adafruit_MQTT_Publish AirQuality = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/feeds/cs147-team28.aqi");
Adafruit_MQTT_Publish AQI_Category = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/feeds/cs147-team28.aqi-category");
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

//define the CO2 min value
int min_co2_reading;


void displayValues(float temperature, float humidity, float p25, float p10, float co2_ppm, int AQI, const char *category)
{
  // Display temperature
  tft.setCursor(50, 62);
  tft.printf("Temp: %.3f C, %.3f F", temperature, temperature * 9 / 5 + 32);

  // Display humidity
  tft.setCursor(50, 83);
  tft.printf("Humidity: %.3f%", humidity);

  // Display PM2.5
  tft.setCursor(50, 104);
  tft.printf("PM2.5: %.3f ug/m3", p25);

  // Display PM10
  tft.setCursor(50, 125);
  tft.printf("PM10: %.3f ug/m3", p10);

  // Display CO2 concentration
  tft.setCursor(50, 146);
  tft.printf("CO2: %.3f ppm", co2_ppm);

  // Display Air Quality Index
  tft.setCursor(50, 167);
  tft.printf("AQI: %d, %s", AQI, category);
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
    const char *category;
} Breakpoint;


Breakpoint get_pm25_breakpoint(float concentration) {
    if (concentration <= 9.0)
        return Breakpoint{0.0, 9.0, 0, 50, "Good"};
    else if (concentration <= 35.4)
        return Breakpoint{9.1, 35.4, 51, 100, "Moderate"};
    else if (concentration <= 55.4)
        return Breakpoint{35.5, 55.4, 101, 150, "Unhealthy For Sensitive"};
    else if (concentration <= 125.4)
        return Breakpoint{55.5, 125.4, 151, 200, "Unhealthy"};
    else if (concentration <= 225.4)
        return Breakpoint{125.5, 225.4, 201, 300, "Very Unhealthy"};
    else if (concentration <= 325.4)
        return Breakpoint{225.5, 325.4, 301, 500, "Hazardous"};
    else
        return Breakpoint{325.5, 999.9, 501, 999, "Hazardous"}; // Beyond AQI scale
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
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextSize(1);

  Serial1.begin(9600, SERIAL_8N1, 25,26); // this line will begin Serial1 with given baud rate (9600 by default)
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
  
  min_co2_reading = 1024;
  //wait 2 minute for the sensor to warm up
  delay(120000);
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
  int co2_reading = analogRead(MQ135_PIN);
  min_co2_reading = std::min(min_co2_reading, co2_reading);
  float co2_ppm = (co2_reading - min_co2_reading) / 1023.0 * 500;
  Serial.print("CO2 PPM Increase: ");
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
  if (!AQI_Category.publish(bp.category))
  {
    Serial.println("Failed to publish Air Quality Index");
  }

  // Update display with new values
  displayValues(temperature, humidity, p25, p10, co2_ppm, AQI, bp.category);

  delay(16000); // Wait for 16 seconds before the next reading
}
