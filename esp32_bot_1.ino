// https://thingsboard.io/docs/samples/esp32/gpio-control-pico-kit-dht22-sensor/
// https://github.com/thingsboard/thingsboard/issues/9450
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>       // search library manager for ArduinoOTA to install
#include <ThingsBoard.h>
#include <Arduino_MQTT_Client.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>      // search library manager for ArduinoJson to install
#include <Adafruit_GPS.h>

// Helper macro to calculate array size
#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

// WiFi access point
#define WIFI_AP_NAME        "IOT"
// WiFi password
#define WIFI_PASSWORD       "dreamport!"

// See https://thingsboard.io/docs/getting-started-guides/helloworld/ 
// to understand how to obtain an access token
#define TOKEN               "A1_TEST_TOKEN"
// ThingsBoard server instance.
#define THINGSBOARD_SERVER  "172.19.25.2"

// Baud rate for debug serial
#define SERIAL_DEBUG_BAUD    115200

#define GPSECHO false
// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

constexpr uint16_t MAX_MESSAGE_SIZE = 128;
constexpr uint16_t THINGSBOARD_PORT = 1883;
// Initialize the Ethernet client object
WiFiClient espClient;
// Initalize the Mqtt client instance
Arduino_MQTT_Client mqttClient(espClient);
// Initialize ThingsBoard instance
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);
// the Wifi radio's status
int status = WL_IDLE_STATUS;

// Main application loop delay
int quant = 250;

// Initial period of LED cycling.
int led_delay = 1000;
// Period of sending a temperature/humidity data.
int send_delay = 2000;

// Time passed after LED was turned ON, milliseconds.
int led_passed = 0;
// Time passed after temperature/humidity data was sent, milliseconds.
int send_passed = 0;

// Set to true if application is subscribed for the RPC messages.
bool subscribed = false;
// LED number that is currenlty ON.
int current_led = 0;

long lastMsg = 0;
char msg[50];
int value = 0;
uint32_t timer = millis();

#define LED_PIN 13
#define WARNING_LED 19
#define VBATPIN A7

// Processes function for RPC call "setValue"
// RPC_Data is a JSON variant, that can be queried using operator[]
// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
RPC_Response processDelayChange(const RPC_Data &data)
{
  Serial.println("Received the set delay RPC method");

  // Process data

  led_delay = data;

  Serial.print("Set new delay: ");
  Serial.println(led_delay);

  return RPC_Response(NULL, led_delay);
}

// Processes function for RPC call "getValue"
// RPC_Data is a JSON variant, that can be queried using operator[]
// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
RPC_Response processGetDelay(const RPC_Data &data)
{
  Serial.println("Received the get value method");

  return RPC_Response(NULL, led_delay);
}

// RPC handlers
RPC_Callback callbacks[] = {
  { "setValue",         processDelayChange },
  { "getValue",         processGetDelay },
};

void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

void reconnect() {
  // Loop until we're reconnected
  status = WiFi.status();
  if ( status != WL_CONNECTED) {
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to AP");
  }
}

// Setup an application
void setup() {
    // Initialize serial for debugging
    Serial.begin(SERIAL_DEBUG_BAUD);
    
    InitWiFi();

    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // uncomment this line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
    // the parser doesn't care about other sentences at this time
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz
       
    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);

    // Ask for firmware version
    GPSSerial.println(PMTK_Q_RELEASE);

    ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";
  
        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
      });
  
    ArduinoOTA.begin();
  
    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  
    pinMode( LED_PIN, OUTPUT ); 
    pinMode( WARNING_LED, OUTPUT );
    digitalWrite( WARNING_LED, LOW );
    
    digitalWrite( LED_PIN, HIGH );   // turn the LED on (HIGH is the voltage level)  
    delay( 500 );
    digitalWrite( LED_PIN, LOW );
    delay( 500 );
    digitalWrite( LED_PIN, HIGH );   // turn the LED on (HIGH is the voltage level)  
    delay( 500 );
    digitalWrite( LED_PIN, LOW );
    delay( 500 );
    digitalWrite( LED_PIN, HIGH );   // turn the LED on (HIGH is the voltage level)  
    delay( 500 );
    digitalWrite( LED_PIN, LOW );  
}

// Main application loop
void loop( ) {
    ArduinoOTA.handle();
    delay(quant);

    led_passed  += quant;
    send_passed += quant;

    // Reconnect to WiFi, if needed
    if (WiFi.status() != WL_CONNECTED) {
      reconnect();
      return;
    }

    if (!tb.connected()) {
        if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
          Serial.println("Failed to connect");
          return;
        }
    }

    // put your main code here, to run repeatedly:
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
    // if a sentence is received, we can check the checksum, parse it...
    if ( GPS.newNMEAreceived() ) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
          return; // we can fail to parse a sentence in which case we should just wait for another
    }

  // Check if it is a time to send DHT22 temperature and humidity
  if (send_passed > send_delay) {
    Serial.println("Sending data...");


    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
    }
    tb.sendTelemetryData("latitude", GPS.latitude);
    tb.sendTelemetryData("longitude", GPS.longitude);
    tb.sendTelemetryData("altitude", GPS.altitude);
    tb.sendTelemetryData("angle", GPS.angle);          
    send_passed = 0;
  }

  // Process messages
  tb.loop();
}