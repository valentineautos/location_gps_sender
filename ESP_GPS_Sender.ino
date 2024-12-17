#include <Arduino.h>
#include <cctype>
#include <TinyGPSPlus.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// Initialise dependancies
TinyGPSPlus gps;
Preferences prefs;

// Task handling
TaskHandle_t request_details_handle = NULL;
TaskHandle_t store_location_handle =  NULL;

// Ready flags
bool weather_data_ready =             false;
bool location_data_ready =            false;
bool wifi_ready =                     false;
bool gps_started =                    false;

// Loop fontrol
volatile bool pps_triggered = false;
volatile unsigned long pps_time =     0;
const int location_check_interval =   30000;   // 30 second intervals for weather check
const int location_store_interval =   300000;  // 5 minute intervals for location storage

// Wifi credentials
const char *WIFI_SSID =               "YOUR_WIFI_SSID"
const char *WIFI_PASS =               "YOUR_WIFI_PASS";
const char GOOGLE_API_KEY =           "YOUR_API_KEY";

uint8_t dataAddress[] =               {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Your screen MAC Address

// Open Metro weather API
const char *WEATHER_API_URL = "https://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f&current=temperature_2m,rain,weather_code,wind_speed_10m&hourly=temperature_2m,weather_code,wind_speed_10m&wind_speed_unit=mph&timezone=GMT&forecast_hours=4";

// Google Reverse GeoLocation API
const char *LOCATION_API_URL = "https://maps.googleapis.com/maps/api/geocode/json?latlng=%.4f,%.4f&result_type=sublocality_level_1|postal_town&key=%s";

#define API_TYPE_WEATHER              0
#define API_TYPE_LOCATION             1

// Pins
#define RXD2                          16
#define TXD2                          17
#define PPS_PIN                       4

#define GPS_BAUD                      9600
#define BOARD_ID                      1

// ESPNow Flags
int8_t FLAG_MAIN =                    0;

// Structures
typedef struct struct_gps {
  float gps_lat;
  float gps_lng;
} struct_gps;

typedef struct struct_location {
  int8_t flag;
  int8_t time_hour;
  int8_t time_minute;
  int8_t date_day;
  int8_t date_month;
  uint16_t date_year;
  int8_t satellites;
  int temperature_now;
  int8_t weather_code_now;
  float wind_now;
  float rain_now;
  char location_name[30];
} struct_location;

struct_gps myGPS;
struct_location myLocation;

// ESP-NOW peer info
esp_now_peer_info_t peerInfo;

// Hardware Serial for GPS
HardwareSerial gpsSerial(2);

// PPS IRS trigger
void IRAM_ATTR pps_isr() {
  pps_triggered = true;
  pps_time = millis();
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Retreive saved data from memory
void load_last_known_location(void) {
  Serial.println("Checking for stored location");
  
  // Use getBytes to retrieve the entire GPS coordinate at once
  float coords[2] = {0.0f, 0.0f};
  if (prefs.getBytes("gps_coords", coords, sizeof(coords)) == sizeof(coords)) {
    myGPS.gps_lat = coords[0];
    myGPS.gps_lng = coords[1];
  } else {
    Serial.println("No complete location data stored");
  }
}

// Store data to memory - timed task
void store_location(void *parameter) {
 while (true) {
    if ((myGPS.gps_lat != 0.0f) && (myGPS.gps_lng != 0.0f)) {
      // Store coordinates as a single byte array
      float coords[2] = {myGPS.gps_lat, myGPS.gps_lng};
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

  memcpy(peerInfo.peer_addr, dataAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add data gauge");
  } else {
    Serial.println("Added data peer");
  }
}

void api_request(int api_type) {
  HTTPClient http;

  char updated_url[(api_type == API_TYPE_WEATHER) ? 260 : 180];

  if (api_type == API_TYPE_WEATHER) {
    snprintf(updated_url, sizeof(updated_url), WEATHER_API_URL, myGPS.gps_lat, myGPS.gps_lng);
  } else {
    snprintf(updated_url, sizeof(updated_url), LOCATION_API_URL, myGPS.gps_lat, myGPS.gps_lng, GOOGLE_API_KEY);
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
        if (api_type == API_TYPE_WEATHER) {
          myLocation.temperature_now = (int)(*doc)["current"]["temperature_2m"] | 0;
          myLocation.weather_code_now = (int)(*doc)["current"]["weather_code"] | 0;
          myLocation.wind_now = (*doc)["current"]["wind_speed_10m"] | 0.0f;
          myLocation.rain_now = (*doc)["current"]["rain"] | 0.0f;

          weather_data_ready = true;
          Serial.println("Weather data updated successfully");
        } else {
          char location_result[30];
          strncpy(location_result, (*doc)["results"][1]["address_components"][0]["short_name"], sizeof(location_result) - 1);
          for (int i = 0; location_result[i] != '\0'; ++i) {
            location_result[i] = std::toupper(location_result[i]);
          }

          strncpy(myLocation.location_name, location_result, sizeof(myLocation.location_name) - 1);
          location_data_ready = true;
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
  } catch (std::exception& e) {
    Serial.print("Exception during HTTP request: ");
    Serial.println(e.what());
  } catch (...) {
    Serial.println("Unknown exception during HTTP request");
  }

  http.end();
}

// changed from request_weather as now includes location name
void request_details(void *parameter) {
  while (true) {
    Serial.println("Starting weather task");

    if (WiFi.status() == WL_CONNECTED) {
            // Only proceed if we have a valid location
      if ((myGPS.gps_lat != 0.0f) && (myGPS.gps_lng != 0.0f)) {

        api_request(API_TYPE_WEATHER); // get weather from API
        vTaskDelay(1000 / portTICK_PERIOD_MS); // rest 1s
        api_request(API_TYPE_LOCATION); // get location from API

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
  esp_now_send(dataAddress, (uint8_t *)&myLocation, sizeof(myLocation));
}

void check_gps() {
   if (pps_triggered) {
    pps_triggered = false; 
    gps_started = true;
    
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }

    // Update time and date information
    myLocation.time_hour = gps.time.hour();
    myLocation.time_minute = gps.time.minute();
    myLocation.date_day = gps.date.day();
    myLocation.date_month = gps.date.month();
    myLocation.date_year = gps.date.year();
    myLocation.satellites = gps.satellites.value();

    // Update location if valid
    if (gps.location.isValid() && gps.location.lat() != 0.0f && gps.location.lng() != 0.0f) {
      Serial.println("Latest location known");
      myGPS.gps_lat = gps.location.lat();
      myGPS.gps_lng = gps.location.lng();
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

  myLocation.flag =           FLAG_MAIN;

  wifi_init();
  
  load_last_known_location();

  xTaskCreate(
    request_details,
    "RequestWeather",
    8192,
    NULL,
    1,
    &request_details_handle
  );

  xTaskCreate(
    store_location,
    "StoreLocation",
    4096,
    NULL,
    2,
    &store_location_handle
  );
}

void loop() {
  check_gps();

  if (weather_data_ready && location_data_ready && gps_started) {
    // Process the weather data
    Serial.println("Sending location data");
    send_location_data();
    weather_data_ready = false;
    location_data_ready = false;
  }
}