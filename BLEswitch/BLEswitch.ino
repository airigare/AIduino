/*********************************************************************
  This is an example for our nRF8001 Bluetooth Low Energy Breakout
  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1697
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
  Written by Kevin Townsend/KTOWN  for Adafruit Industries.
  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in any redistribution
*********************************************************************/

#include <SPI.h>


#include "Adafruit_BLE_UART.h"

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 6
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 10

#define RELAIS_PIN 3

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

#define BLE_READPACKET_TIMEOUT         50   // Timeout in ms waiting to read a response


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE_UART *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];


/**************************************************************************/
/*!
    Configure the Arduino and start advertising with the radio
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(9600);
  while (!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));


  /* Initialise the module */
  Serial.print(F("Initialising the Adafruit nRF8001 module: "));

  if ( !BTLEserial.begin())
  {
    error(F("Couldn't find nrf8001, check wiring?"));
  }
  Serial.println( F("OK!") );

  BTLEserial.setDeviceName("UART"); /* 7 characters max! */

  pinMode(RELAIS_PIN, OUTPUT);
  delay(100);
  digitalWrite(RELAIS_PIN, HIGH);
  delay(1000);
  digitalWrite(RELAIS_PIN, LOW);
}

/**************************************************************************/
/*!
    Constantly checks for new events on the nRF8001
*/
/**************************************************************************/
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop()
{
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
      Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
      Serial.println(F("* Connected!"));
      BTLEserial.print("Hallo");
    }
    if (status == ACI_EVT_DISCONNECTED) {
      Serial.println(F("* Disconnected or advertising timed out"));
      digitalWrite(RELAIS_PIN, LOW); //In Case of Contact Loss, turn Relais OFF
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status != ACI_EVT_CONNECTED) {
    return;
  }

  String bleMsg = "";
  if (BTLEserial.available()) {
    while (BTLEserial.available()) {
      char c = BTLEserial.read();
      bleMsg.concat(c);
      Serial.print(c);
    }
    // OK while we still have something to read, get a character and print it out

    BTLEserial.print(bleMsg);

    if (bleMsg == "R1") {
      digitalWrite(RELAIS_PIN, HIGH);
      BTLEserial.print("Relais is ON");
    }

    if (bleMsg == "R0") {
      BTLEserial.print("Relais is OFF");
      digitalWrite(RELAIS_PIN, LOW);
    }
  }
}
