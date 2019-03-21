/*************************************************** 
  This is an example for the Adafruit CC3000 Wifi Breakout & Shield
  Designed specifically to work with the Adafruit WiFi products:
  ----> https://www.adafruit.com/products/1469
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!
  Written by Limor Fried & Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

 /*
This example does a test of the TCP client capability:
  * Initialization
  * Optional: SSID scan 
  * AP connection
  * DHCP printout
  * DNS lookup
  * Optional: Ping
  * Connect to website and print out webpage contents
  * Disconnect
SmartConfig is still beta and kind of works but is not fully vetted!
It might not work on all networks!
*/
#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"
#include <avr/wdt.h>

#include <LowPower.h>

#include <ArduinoJson.h>

#define SEND_NON_BLOCKING 1 // I think this is a solution for all troubles with CC3000

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   2  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER); // you can change this clock speed



// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define RELAIS_PIN 3
#define WIFI_PIN 4

#define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data 
                                   // received before closing the connection.  If you know the server
                                   // you're accessing is quick to respond, you can reduce this value.
const unsigned long
  dhcpTimeout     = 60L * 1000L, // Max time to wait for address from DHCP
  connectTimeout  = 15L * 1000L, // Max time to wait for server connection
  responseTimeout = 15L * 1000L; // Max time to wait for data from server

// What page to grab!
#define WEBSITE      "airigare-api.scapp.io"
#define PORT         80

#define SLEEP_DURATION 60

  int MAXsleepIterations, pumpTime, sleepIterations;
  char* pumpTimeC;

#define API  "79cf6c22-dcc6-11e5-8e77-00113217113f"

/**************************************************************************/
/*!
    @brief  Sets up the HW and the CC3000 module (called automatically
            on startup)
*/
/**************************************************************************/

void(* resetFunc) (void) = 0; //declare reset function @ address 0

  char
  country[20],
  region[20],
  city[20],
  name[13],  // Temp space for name:value parsing
  value[64]; // Temp space for name:value parsing
  float
  longitude, latitude;

  uint32_t ip = 0L, t;

void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
  asm volatile ("  jmp 0");  
}

// On error, print PROGMEM string to serial monitor and stop
void hang(const __FlashStringHelper *str) {
  Serial.println(str);
  //for(;;);
  //resetFunc();
  //delay(100);
  digitalWrite(RELAIS_PIN, HIGH);
  delay(100);
  digitalWrite(RELAIS_PIN, LOW);
  

  
}



// Return true if enabled and connected, false otherwise.
boolean enableWiFi() {
  digitalWrite(WIFI_PIN, HIGH);
  delay(5000);
  delay(5000);
  delay(5000);
  
  
  Serial.print(("Initializing CC3000..."));

  for(t=millis(); !cc3000.begin() && ((millis() - t) < dhcpTimeout); delay(100));
  if(!cc3000.begin()) { hang(F("failed.")); } else { Serial.print(F("OK\r\n")); };
  delay(100);
  
  Serial.print(("Deleting old connection profiles..."));
  if(!cc3000.deleteProfiles()) { hang(F("failed.")); } else {Serial.print(F("OK\r\n"));};
delay(100);

  Serial.print(("Connecting to network..."));
  /* NOTE: Secure connections are not available in 'Tiny' mode! */
  if(!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) { hang(F("failed.")); } else {Serial.print(F("OK\r\n"));};

  Serial.print(("Requesting address from DHCP server..."));
  
  for(t=millis(); !cc3000.checkDHCP() && ((millis() - t) < dhcpTimeout); delay(100));
    if(!cc3000.checkDHCP()) { hang(F("failed.")); } else { Serial.print(F("OK\r\n")); };


  /* Display the IP address DNS, Gateway, etc. */  
  while (! displayConnectionDetails()) {
    delay(1000);
  }
  
  // Return success, the CC3000 is enabled and connected to the network.
  return true;
}

// Disconnect from wireless network and shut down the CC3000.
void shutdownWiFi() {
  // Disconnect from the AP if connected.
  // This might not be strictly necessary, but I found
  // it was sometimes difficult to quickly reconnect to
  // my AP if I shut down the CC3000 without first
  // disconnecting from the network.

  // Wait for the CC3000 to finish disconnecting before
  // continuing.

  Serial.println(("Disconnecting...")); 
  cc3000.disconnect();
  delay(5000); // Wait until disconnect
  
  // Shut down the CC3000.
  wlan_stop();
  Serial.println(F("CC3000 shut down.")); 
  delay(1000); // Wait until disconnect
  
  digitalWrite(WIFI_PIN, LOW);
  delay(500);
  digitalWrite(WIFI_PIN, INPUT);
  delay(500);
}

/**************************************************************************/
/*!
    @brief  Helper Functions
*/
/**************************************************************************/
bool displayConnectionDetails(void) {
  uint32_t addr, netmask, gateway, dhcpserv, dnsserv;

  if(!cc3000.getIPAddress(&addr, &netmask, &gateway, &dhcpserv, &dnsserv))
    return false;

  Serial.print(F("IP Addr: ")); cc3000.printIPdotsRev(addr);
  Serial.print(F("\r\nNetmask: ")); cc3000.printIPdotsRev(netmask);
  Serial.print(F("\r\nGateway: ")); cc3000.printIPdotsRev(gateway);
  Serial.print(F("\r\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
  Serial.print(F("\r\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
  Serial.println();
  return true;
}

uint32_t getIP (void) {
  ip = 0;
  // Try looking up the website's IP address
  Serial.print(WEBSITE); Serial.print(F(" -> "));
  while (ip == 0) {
    if (! cc3000.getHostByName(WEBSITE, &ip)) {
      Serial.println(F("Couldn't resolve!"));
    }
    delay(500);
  }
  return ip;
}


/**************************************************************************/
/*!
    @ WWW Functions
*/
/**************************************************************************/
/**************************************************************************/
/*!
    @ Call Mom
*/
bool callMama() {
  logStatus("101","1");
  return true;
}


bool logStatus(char* id, char*(value)) {

  uint32_t ip = getIP();

  Adafruit_CC3000_Client www = cc3000.connectTCP(ip, PORT);
  if (www.connected()) {
    www.fastrprint(F("GET "));
    www.fastrprint("/mikmakAPI/airigare/Station/Status");
    www.fastrprint("?API=");
    www.fastrprint(API);
    www.fastrprint("&id=");
    www.fastrprint(id);
    www.fastrprint("&value=");
    www.fastrprint(value);
    www.fastrprint(F(" HTTP/1.1\r\n"));
    www.fastrprint(F("Host: ")); www.fastrprint(WEBSITE); www.fastrprint(F("\r\n"));
    www.fastrprint(F("\r\n"));
    www.println();
  }
  else {
    Serial.println(F("Connection failed"));
  }



  // Note that if you're sending a lot of data you
  // might need to tweak the delay here so the CC3000 has
  // time to finish sending all the data before shutdown.
  Serial.print(F("OK\r\nAwaiting response..."));
  unsigned long startTime = millis();
  while((!www.available()) && ((millis() - startTime) < responseTimeout));
  {

  // Close the connection to the server.
  // Check HTTP status
    char status[32] = {0};
    www.readBytesUntil('\r', status, sizeof(status));
    if (strcmp(status, "HTTP/1.1 200 OK") != 0) {
      Serial.print(F("Unexpected response: "));
      Serial.println(status);
      return true;
    }

  // Skip HTTP headers
    char endOfHeaders[] = "\r\n\r\n";
    if (!www.find(endOfHeaders)) {
      Serial.println(F("Invalid response"));
      return true;
    }

  // Allocate JsonBuffer
  // Use arduinojson.org/assistant to compute the capacity.
    const size_t capacity = JSON_OBJECT_SIZE(3) + JSON_ARRAY_SIZE(2) + 60;
    DynamicJsonBuffer jsonBuffer(capacity);

  // Parse JSON object
    JsonObject& root = jsonBuffer.parseObject(www);
    if (!root.success()) {
      Serial.println(F("Parsing failed!"));
      return true;
    }

  // Extract values
    Serial.println(F("Response:"));
    Serial.println(root["fieldCount"].as<char*>());
    Serial.println(root["affectedRows"].as<char*>());

    Serial.println(F("Pizdets!"));
    delay(1000);
  }
  
  www.close();
  return true;
}

/**************************************************************************/
/*!
    @ Get Instructions
*/
bool getInstructions() {

  bool r;

  uint32_t ip = getIP();

  Adafruit_CC3000_Client www = cc3000.connectTCP(ip, PORT);
  if (www.connected()) {
    www.fastrprint(F("GET "));
    www.fastrprint("/mikmakAPI/airigare/Station/getInstructions?API=");
    www.fastrprint(API);
    www.fastrprint(F(" HTTP/1.1\r\n"));
    www.fastrprint(F("Host: ")); www.fastrprint(WEBSITE); www.fastrprint(F("\r\n"));
    www.fastrprint(F("\r\n"));
    www.println();
  }
  else {
    Serial.println(F("Connection failed"));
  }


  // Note that if you're sending a lot of data you
  // might need to tweak the delay here so the CC3000 has
  // time to finish sending all the data before shutdown.
  Serial.print(F("OK\r\nAwaiting response..."));
  unsigned long startTime = millis();
  while((!www.available()) && ((millis() - startTime) < responseTimeout));
  {

  // Close the connection to the server.
  // Check HTTP status
    char status[32] = {0};
    www.readBytesUntil('\r', status, sizeof(status));
    if (strcmp(status, "HTTP/1.1 200 OK") != 0) {
      Serial.print(F("Unexpected response: "));
      Serial.println(status);
      return false;
    }

  // Skip HTTP headers
    char endOfHeaders[] = "\r\n\r\n";
    if (!www.find(endOfHeaders)) {
      Serial.println(F("Invalid response"));
      return false;
    }

  // Allocate JsonBuffer
  // Use arduinojson.org/assistant to compute the capacity.
    const size_t capacity = JSON_OBJECT_SIZE(2) + 40;
    DynamicJsonBuffer jsonBuffer(capacity);

  // Parse JSON object
    JsonObject& root = jsonBuffer.parseObject(www);
    if (!root.success()) {
      Serial.println(F("Parsing failed!"));
      return false;
    }

  // Extract values
    Serial.println(F("Response: "));
    MAXsleepIterations = root["sleepIterations"];
    pumpTime = root["pumpTime"];
    pumpTimeC = root["pumpTime"];
    delay(100);
  }

  Serial.println(F("Pizdets!"));
  delay(1000);
  www.close();
  return true;
}

bool enablePump(int pumpTime, char* pumpTimeC) {
  Serial.print("Start Pumping for: "); Serial.println(pumpTime, DEC);
  digitalWrite(RELAIS_PIN, HIGH);
  for(int x = 0; x < pumpTime; x = x + 1){
    delay(1000);
  }
  digitalWrite(RELAIS_PIN, LOW);
  Serial.print("Stop Pumping");

logStatus("109", pumpTimeC);
  
  return true;
}

void setup(void)
{ 
  Serial.begin(115200);
  Serial.println(F("Hello, CC3000!\n")); 

  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);

  pinMode(RELAIS_PIN, OUTPUT);
  delay(100);
  pinMode( WIFI_PIN, OUTPUT);

  
  digitalWrite(RELAIS_PIN, HIGH);
  delay(1000);
  digitalWrite(RELAIS_PIN, LOW);

   /*
  digitalWrite(WIFI_PIN, HIGH);
  delay(5000);
  delay(5000);
  digitalWrite(WIFI_PIN, LOW);
  delay(5000);
  delay(5000);
*/

  MAXsleepIterations = SLEEP_DURATION / 8;
  sleepIterations = 0;

  Serial.println("Setup... OK");
  
}

void loop(void)
{
  if (sleepIterations == 0) {
    if (enableWiFi()) {

      Serial.print(F("Calling Mama... "));
      if(!callMama()) { hang(F("failed.")); } else {Serial.print(F("OK\r\n"));};
      
      Serial.print(F("Get Instructions... "));
      
      if(getInstructions()) { Serial.print(F("OK\r\n")); } else {Serial.print(F("nOK\r\n"));};
      if(pumpTime > 0) enablePump(pumpTime, pumpTimeC);
      
    }
    shutdownWiFi();
  }
  
  else if (sleepIterations > 0 && sleepIterations < MAXsleepIterations) {
          // Go to sleep!
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

  }
  else if (sleepIterations >= MAXsleepIterations) {
    //resetFunc();  //call reset
    software_Reset();
    //sleepIterations = -1;
    delay(100);
  }

  sleepIterations += 1;

}
