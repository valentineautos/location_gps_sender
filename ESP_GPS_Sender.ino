#include <Arduino.h>
#include <cctype>
#include "secrets.h"
#include <time.h>
#include <TinyGPSPlus.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "GaugeMinimal.h"

// Define pin
#define PIN_OIL 32

// Initialise dependancies
TinyGPSPlus gps;
Preferences prefs;

struct tm api_time;
time_t local_timestamp;

// Ready flags
bool weather_data_ready = false;
bool location_data_ready = false;
bool time_data_ready = false;
bool wifi_ready = false;
bool gps_started = false;

// Loop fontrol
volatile bool pps_triggered = false;
volatile unsigned long pps_time = 0;
const int send_date_time_interval = 5000;    // Update date time every 5 seconds
const int send_oil_press_interval = 200;     // Update oil pressure every 200ms
const int location_check_interval = 30000;   // 30 second intervals for weather check
const int location_store_interval = 300000;  // 5 minute intervals for location storage

// Wifi credentials
const char *WIFI_SSID = SECRET_SSID;
const char *WIFI_PASS = SECRET_PASS;
// const char GOOGLE_API_KEY =           "YOUR_API_KEY";

uint8_t levelsAddress[] = LEVELS_MAC_ADDR;
uint8_t weatherAddress[] = WEATHER_MAC_ADDR;
uint8_t speedoAddress[] = SPEEDO_MAC_ADDR;
uint8_t canbusSnifferAddress[] = CANBUS_SNIFFER_MAC_ADDR;

uint8_t *peerAddresses[] = {
  levelsAddress,
  weatherAddress,
  speedoAddress,
  canbusSnifferAddress
};

const int num_peers = sizeof(peerAddresses) / sizeof(peerAddresses[0]);

// Open Metro weather API
const char *WEATHER_API_URL = "https://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f&current=temperature_2m,rain,weather_code,wind_speed_10m&hourly=temperature_2m,weather_code,wind_speed_10m&wind_speed_unit=mph&timezone=GMT&forecast_hours=4";

// Google Reverse GeoLocation API
const char *LOCATION_API_URL = "https://maps.googleapis.com/maps/api/geocode/json?latlng=%.4f,%.4f&result_type=sublocality_level_1|postal_town&key=%s";

// TimeAPI.io
const char *TIME_API_URL = "https://timeapi.io/api/time/current/coordinate?latitude=%.4f&longitude=%.4f";

#define API_TYPE_WEATHER 0
#define API_TYPE_LOCATION 1
#define API_TYPE_TIME 2

// Pins
#define RXD2 16
#define TXD2 17
#define PPS_PIN 4

#define GPS_BAUD 9600
#define BOARD_ID 1

// Structures
typedef struct struct_gps {
  float gps_lat;
  float gps_lng;
} struct_gps;

typedef struct struct_weather {
  uint8_t flag;
  uint8_t temp_now;
  float rain_now;
  uint8_t weather_code_now;
  char location_name[30];
} struct_weather;

typedef struct struct_time {
  uint8_t flag;
  uint8_t time_hour;
  uint8_t time_minute;
  uint8_t date_day;
  uint8_t date_month;
  uint16_t date_year;
} struct_time;

typedef struct struct_oil_press {
  uint8_t flag;
  uint8_t oil_press;
} struct_oil_press;

struct_gps GPSData;
struct_weather WeatherData;
struct_time TimeData;
struct_oil_press OilData;

esp_now_peer_info_t peerInfo;

// Hardware Serial for GPS
HardwareSerial gpsSerial(2);

// PPS IRS trigger
void IRAM_ATTR pps_isr() {
  pps_triggered = true;
  pps_time = millis();
}

struct_time create_default_time() {
  struct_time default_time = { 12, 0, 1, 1, 2025 };  // Defaults: 12:00, January 1, 2024
  return default_time;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Retreive saved data from memory
void load_last_known_location(void) {
  Serial.println("Checking for stored location");

  // Use getBytes to retrieve the entire GPS coordinate at once
  float coords[2] = { 0.0f, 0.0f };
  if (prefs.getBytes("gps_coords", coords, sizeof(coords)) == sizeof(coords)) {
    GPSData.gps_lat = coords[0];
    GPSData.gps_lng = coords[1];
  } else {
    Serial.println("No complete location data stored");
  }
}

void send_oil_press(void *parameter) {
  while (true) {
    //double adc_read = adjust_adc_voltage(PIN_OIL);
    float adc_read = analogRead(PIN_OIL);

    // Serial.print("adc_read ");
    // Serial.println(adc_read);

    // float calc_volt = (((float)adc_read / 4095) * 5) + 0.2;

    // int psi = (30 * calc_volt) - 15;

    int psi = (150 * (float)adc_read / 4095) - 9;

    if (psi < 0) {
      psi = 0;
    }

    OilData.oil_press = psi;

    Serial.print(psi);

    esp_now_send(levelsAddress, (uint8_t *)&OilData, sizeof(OilData));
    vTaskDelay(send_oil_press_interval / portTICK_PERIOD_MS);
  }
}

void send_date_time(void *parameter) {
  while (true) {
    if (time_data_ready) {
      time_t now = time(NULL);
      double elapsed_seconds = difftime(now, local_timestamp);

      time_t api_timestamp = mktime(&api_time);
      api_timestamp += (time_t)elapsed_seconds;

      struct tm *updated_time = localtime(&api_timestamp);
      api_time = *updated_time;

      local_timestamp = now;

      TimeData.time_hour = api_time.tm_hour;
      TimeData.time_minute = api_time.tm_min;
      TimeData.date_day = api_time.tm_mday;
      TimeData.date_month = api_time.tm_mon + 1;
      TimeData.date_year = api_time.tm_year + 1900;

      esp_now_send(levelsAddress, (uint8_t *)&TimeData, sizeof(TimeData));
      vTaskDelay(send_date_time_interval / portTICK_PERIOD_MS);
    }
  }
}

// Store data to memory - timed task
void store_location(void *parameter) {
  while (true) {
    if ((GPSData.gps_lat != 0.0f) && (GPSData.gps_lng != 0.0f)) {
      // Store coordinates as a single byte array
      float coords[2] = { GPSData.gps_lat, GPSData.gps_lng };
      prefs.putBytes("gps_coords", coords, sizeof(coords));

      Serial.println("Location coordinates stored");
      prefs.end();
    } else {
      Serial.println("Location invalid - cannot store");
    }

    vTaskDelay(location_store_interval / portTICK_PERIOD_MS);
  }
}

// Initialise wifi
void wifi_init() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.mode(WIFI_STA);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to Wi-Fi...");
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifi_ready = true;
    Serial.println("Connected to Wi-Fi");
  }

  // EUGH
  wifi_second_chan_t second_chan;
  uint8_t wifi_channel;
  esp_wifi_get_channel(&wifi_channel, &second_chan);
  esp_wifi_set_channel(wifi_channel, WIFI_SECOND_CHAN_NONE);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else {
    Serial.println("ESPNow init success");
  }

  esp_now_register_send_cb(OnDataSent);

  peerInfo.channel = wifi_channel;
  peerInfo.encrypt = false;

  // setup all peers and distribute the channel ID
  for (int i = 0; i < num_peers; i++) {
    // Set up the peer address
    memcpy(peerInfo.peer_addr, peerAddresses[i], 6);

    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
      Serial.printf("Peer %d added.\n", i);
    }

    // Send the channel ID to the current peer
    esp_err_t result = esp_now_send(peerAddresses[i], &wifi_channel, sizeof(wifi_channel));
    if (result == ESP_OK) {
      Serial.printf("Channel ID sent to Peer %d.\n", i);
    } else {
      Serial.printf("Failed to send to Peer %d. Error: %d\n", i, result);
    }
  }

  api_request(API_TYPE_TIME);
}

void api_request(int api_type) {
  HTTPClient http;

  char updated_url[(api_type == API_TYPE_WEATHER) ? 260 : 180];

  switch (api_type) {
    case API_TYPE_WEATHER:
      snprintf(updated_url, sizeof(updated_url), WEATHER_API_URL, GPSData.gps_lat, GPSData.gps_lng);
      break;
    case API_TYPE_LOCATION:
      snprintf(updated_url, sizeof(updated_url), LOCATION_API_URL, GPSData.gps_lat, GPSData.gps_lng, GOOGLE_API_KEY);
      break;
    case API_TYPE_TIME:
      snprintf(updated_url, sizeof(updated_url), TIME_API_URL, GPSData.gps_lat, GPSData.gps_lng);
  }

  Serial.print("Request URL: ");
  Serial.println(updated_url);

  http.begin(updated_url);

  http.setTimeout(3000);  // Set a timeout of 3 seconds

  Serial.println("Sending API request...");

  bool api_ready = false;

  // Wrap HTTP request in a try-catch to capture potential exceptions
  try {
    for (int retry = 0; retry < 3; retry++) {
      int httpResponseCode = http.GET();
      Serial.print("httpResponseCode = ");
      Serial.println(httpResponseCode);

      if (httpResponseCode > 0) {
        api_ready = true;
        break;  // Exit loop if the request succeeds
      }

      // Wait 2s and retry
      Serial.println("Retrying request...");
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    if (api_ready) {
      Serial.println("API ready");
      // Increased buffer size with safety margin
      const size_t capacity = JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(8) + 2048;

      // Use a pointer to dynamically allocate memory
      DynamicJsonDocument *doc = new DynamicJsonDocument(capacity);

      // Limit response size to prevent overflow
      String response = http.getString();
      // response = response.substring(0, 2000);  // Limit to 2000 chars

      //Serial.print(response);

      DeserializationError error = deserializeJson(*doc, response);

      if (!error) {
        Serial.print("No errors");
        switch (api_type) {
          case API_TYPE_WEATHER:
            WeatherData.temp_now = (int)(*doc)["current"]["temperature_2m"] | 0;
            WeatherData.weather_code_now = (int)(*doc)["current"]["weather_code"] | 0;
            WeatherData.rain_now = (*doc)["current"]["rain"] | 0.0;

            weather_data_ready = true;
            Serial.println("Weather data updated successfully");

            break;
          case API_TYPE_LOCATION:
            char location_result[30];
            strncpy(location_result, (*doc)["results"][1]["address_components"][0]["short_name"], sizeof(location_result) - 1);
            for (int i = 0; location_result[i] != '\0'; ++i) {
              location_result[i] = std::toupper(location_result[i]);
            }

            strncpy(WeatherData.location_name, location_result, sizeof(WeatherData.location_name) - 1);
            location_data_ready = true;

            break;
          case API_TYPE_TIME:
            api_time.tm_year = (int)(*doc)["year"] - 1900;
            api_time.tm_mon = (int)(*doc)["month"] - 1;
            api_time.tm_mday = (*doc)["day"];
            api_time.tm_hour = (*doc)["hour"];
            api_time.tm_min = (*doc)["minute"];
            api_time.tm_sec = (*doc)["seconds"];
            api_time.tm_isdst = -1;

            local_timestamp = time(NULL);

            time_data_ready = true;
        }
        // Extract JSON data

      } else {
        Serial.print("JSON parse failed: ");
        Serial.println(error.c_str());
      }

      // Properly free dynamically allocated memory
      delete doc;
    } else {
      Serial.println("Connection to weather API failed");
    }
  } catch (std::exception &e) {
    Serial.print("Exception during HTTP request: ");
    Serial.println(e.what());
  } catch (...) {
    Serial.println("Unknown exception during HTTP request");
  }

  http.end();
}


void request_details(void *parameter) {
  while (true) {
    Serial.println("Starting weather task");

    if (WiFi.status() == WL_CONNECTED) {
      // Only proceed if we have a valid location
      if ((GPSData.gps_lat != 0.0f) && (GPSData.gps_lng != 0.0f)) {

        api_request(API_TYPE_WEATHER);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        api_request(API_TYPE_LOCATION);

      } else {
        Serial.println("No valid GPS location - skipping weather request");
      }
    } else {
      Serial.println("WiFi not connected, skipping weather request");
    }

    vTaskDelay(location_check_interval / portTICK_PERIOD_MS);  // Wait before next request
  }
}

void send_location_data() {
  // Send main location data
  esp_now_send(weatherAddress, (uint8_t *)&WeatherData, sizeof(WeatherData));
}

void check_gps() {
  if (pps_triggered) {
    pps_triggered = false;
    gps_started = true;

    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }

    // Update location if valid
    if (gps.location.isValid() && gps.location.lat() != 0.0f && gps.location.lng() != 0.0f) {
      Serial.println("Latest location known");
      GPSData.gps_lat = gps.location.lat();
      GPSData.gps_lng = gps.location.lng();
    } else {
      Serial.println("Latest location not known or invalid");
    }
  }
}

void setup() {
  // Serial Monitor
  Serial.begin(115200);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  prefs.begin("gps_store", false);

  pinMode(PPS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), pps_isr, FALLING);

  wifi_init();

  load_last_known_location();

  WeatherData.flag = FLAG_GPS;
  OilData.flag = FLAG_OIL_PRESSURE;

  // Time Data defaults
  TimeData.flag = FLAG_GPS;  // define sender ID

  xTaskCreate(request_details, "RequestWeather", 8192, NULL, 1, NULL);
  xTaskCreate(send_date_time, "SendDateTime", 4096, NULL, 2, NULL);
  xTaskCreate(store_location, "StoreLocation", 4096, NULL, 2, NULL);
  xTaskCreate(send_oil_press, "SendOilPressure", 4096, NULL, 3, NULL);
}

void loop() {
  check_gps();

  if (weather_data_ready && location_data_ready) {
    // Process the weather data
    Serial.println("Sending location data");
    Serial.println(WeatherData.temp_now);
    Serial.println(WeatherData.rain_now);
    Serial.println(WeatherData.weather_code_now);
    Serial.println(WeatherData.location_name);

    send_location_data();
    weather_data_ready = false;
    location_data_ready = false;
  }
}