
/*
  Web client

  This sketch connects to a website (http://www.google.com)
  using the WiFi module.

  This example is written for a network using WPA encryption. For
  WEP or WPA, change the Wifi.begin() call accordingly.

  This example is written for a network using WPA encryption. For
  WEP or WPA, change the Wifi.begin() call accordingly.

  Circuit:
   Board with NINA module (Arduino MKR WiFi 1010, MKR VIDOR 4000 and UNO WiFi Rev.2)

  created 13 July 2010
  by dlf (Metodo2 srl)
  modified 31 May 2012
  by Tom Igoe
*/


#include <SPI.h>
#include <WiFiNINA.h>
//#include <WiFi101.h>
#include "ArduinoLowPower.h"
#include <ArduinoJson.h>
#include <Arduino_MKRENV.h>

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = "bawdiestNet";        // your network SSID (name)
char pass[] = "Nkensleg8";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

#define WEBSITE      "api.mikmak.cc"
#define PORT         80

#define API  "79cf6c22-dcc6-11e5-8e77-00113217113f"

float g_temperature;
float g_humidity;
float g_pressure;

int status = WL_IDLE_STATUS;


// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
WiFiClient client;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  //while (!Serial) {
  //; // wait for serial port to connect. Needed for native USB port only
  //}

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, alarmEvent0, CHANGE); 
}

void alarmEvent0() {
  int alarm_source = 0;
}

void loop() {
  Serial.println("Connecting to WIFI...");
  connectWIFI(); 
  Serial.println("Reading WeatherData...");
  readWeatherData(); 
  delay(1000);
  
  logStatus("101", "1");
  delay(2000);
  
  postWeatherData(ENV.readTemperature(), ENV.readHumidity(), ENV.readPressure() * 10);
  delay(2000);
  
  //WiFi.disconnect();
  int status = WL_IDLE_STATUS;
  delay(1000);
  WiFi.end();

  Serial.println("Going to sleep...");
  LowPower.sleep(3600 * 1000);
  //LowPower.sleep(2000);
  NVIC_SystemReset();
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}


void logStatus(char* id, char*(value)) {

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  if (client.connect(WEBSITE, PORT)) {
    Serial.println("connected to server");
    client.print(F("GET "));
    client.print("/mikmakAPI/airigare/Station/Status");
    client.print("?API=");
    client.print(API);
    client.print("&id=");
    client.print(id);
    client.print("&value=");
    client.print(value);
    client.print(F(" HTTP/1.1\r\n"));
    client.print(F("Host: ")); client.print(WEBSITE); client.print(F("\r\n"));
    client.print(F("\r\n"));
    client.println();
  }
  else {
    Serial.println(F("Connection failed"));
  }

  // Note that if you're sending a lot of data you
  // might need to tweak the delay here so the CC3000 has
  // time to finish sending all the data before shutdown.
  Serial.print(F("OK\r\nAwaiting response..."));

  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }
  //unsigned long startTime = millis();
  //while((!client.available()) && ((millis() - startTime) < responseTimeout));
  {

    // Close the connection to the server.
    // Check HTTP status
    char status[32] = {0};
    client.readBytesUntil('\r', status, sizeof(status));
    if (strcmp(status, "HTTP/1.1 200 OK") != 0) {
      Serial.print(F("Unexpected response: "));
      Serial.println(status);
      //return true;
    }

    // Skip HTTP headers
    char endOfHeaders[] = "\r\n\r\n";
    if (!client.find(endOfHeaders)) {
      Serial.println(F("Invalid response"));
      //return true;
    }


    DynamicJsonDocument doc(1024);

    // Parse JSON object
    auto error = deserializeJson(doc, client);
    if (error) {
      Serial.print(F("deserializeJson() failed with code "));
      Serial.println(error.c_str());
      return;
    }

    JsonObject root = doc.as<JsonObject>();

    // Extract values
    Serial.println(F("Response:"));
    Serial.println(root["fieldCount"].as<char*>());
    Serial.println(root["affectedRows"].as<char*>());

    Serial.println(F("Pizdets!"));
    delay(1000);
  }

  Serial.println();
  Serial.println("disconnecting from server.");
  client.stop();
  //return true;
}

bool readWeatherData() {
  if (!ENV.begin()) {
    Serial.println("Failed to initialize MKR ENV shield!");
    while (1);
  }
  // read all the sensor values
  g_temperature = ENV.readTemperature();
  g_humidity    = ENV.readHumidity();
  g_pressure    = ENV.readPressure() * 10;
  //float illuminance = ENV.readIlluminance();
  //float uva         = ENV.readUVA();
  //float uvb         = ENV.readUVB();
  //float uvIndex     = ENV.readUVIndex();

  // print each of the sensor values
  Serial.print("Temperature = ");
  Serial.print(g_temperature);
  Serial.println(" °C");

  Serial.print("Humidity    = ");
  Serial.print(g_humidity);
  Serial.println(" %");

  Serial.print("Pressure    = ");
  Serial.print(g_pressure);
  Serial.println(" Pa");

  return true;
}

bool connectWIFI() {
  Serial.println("WiFi Status: " + status);
  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWifiStatus();

  return true;
}

void postWeatherData(float temp, float humid, float press) {

  Serial.println("\nStarting connection to server /mikmakAPI/airigare/Station/postWeatherData...");
  // if you get a connection, report back via serial:
  if (client.connect(WEBSITE, PORT)) {
    Serial.println("connected to server");
    client.print(F("GET "));
    client.print("/mikmakAPI/airigare/Station/postWeatherData");
    client.print("?API=");
    client.print(API);
    client.print("&temp=");
    client.print(temp);
    client.print("&pres=");
    client.print(press);
    client.print("&humid=");
    client.print(humid);
    client.print(F(" HTTP/1.1\r\n"));
    client.print(F("Host: ")); client.print(WEBSITE); client.print(F("\r\n"));
    client.print(F("\r\n"));
    client.println();
  }
  else {
    Serial.println(F("Connection failed"));
  }
  
  delay(1000);

  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }

  Serial.println();
  Serial.println("disconnecting from server.");
  client.stop();
  //return true;
}
