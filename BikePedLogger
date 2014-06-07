/*
  Analog Input
Via a voltage divider, takes an analog sample, converts it
and logs it as a voltage reading to an SD card  */
//
// Wiring to the Adafruit LED Backpack is really easy
//Connect CLK to the I2C clock - on Arduino UNO thats Analog #5, on the Leonardo it's Digital #3, on the Mega it's digital #21
//Connect DAT to the I2C data - on Arduino UNO thats Analog #4, on the Leonardo it's Digital #2, on the Mega it's digital #20
//Connect GND to common ground
//Connect VCC+ to power - 5V is best but 3V also seems to work for 3V microcontrollers.

// Voltage divider
//0	0
//199 3.30v
//304 5.04v
//748 12.39v

// SDA Analog pin 4
// SCL Analog pin 5

#include <Wire.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <SD.h>
#include <TimerOne.h>
#include <Passwords.h>
#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <string.h>
#include "utility/debug.h"
#include <stdlib.h>

// A simple data logger for the Arduino analog pins
#define LOG_INTERVAL  600 // mills between entries
#define LAG 10 // slow it down this many ms so the IR sensor can recover
#define ECHO_TO_SERIAL   1 // echo data to serial port  //incompatible with grove TM1637.h
#define WAIT_TO_START    0 // Wait for serial input in setup()
#define ledPin 9
#define ledError 8  // digital pint for red error LED
#define sensorPin A0
//#define tuningFactor 16.95
#define chipSelect 4 // 4 for Ethernet shield, 10 for the data logging shield, we use digital pin 10 for the SD cs line
#define CLK 2//pins definitions for TM1637 and can be changed to other ports    default 2,3
#define DIO 3
//TM1637 tm1637(CLK,DIO);

#define WLAN_SSID       "NETWORKNAMEGOESHERE"        // cannot be longer than 32 characters!
#define WLAN_PASS       "LANPASSWORDGOESHERE"
#define WEBSITE  "api.xively.com"
#define API_key  "XIVELYAPIKEYGOESHERE"
#define feedID  "XIVELYFEEDIDGOESHERE"
// CC3000 wifi setup
// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                         SPI_CLOCK_DIVIDER); // you can change this clock speed but DI
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2
Adafruit_CC3000_Client client;
const unsigned long
connectTimeout  = 15L * 1000L, // Max time to wait for server connection
responseTimeout = 15L * 1000L; // Max time to wait for data from server
// end CC3000 wifi declarations

String dataString = "";
String datetimeString = "";

int irSensor = 2;
int sensorStateLed = 13;
int buttonPort = 3;

int counter = 0;
unsigned long startTime;

bool previousSensorState = LOW;

bool initialized = false;

void setup() {
  Serial.begin(9600);
  
  pinMode(irSensor, INPUT);
  pinMode(buttonPort, INPUT);
  pinMode(sensorStateLed, OUTPUT);

  Serial.println(F("Hello, CC3000!\n"));

  displayDriverMode();

  Serial.println(F("\nInitialising the CC3000 ..."));
  if (!cc3000.begin()) {
    Serial.println(F("Unable to initialise the CC3000! Check your wiring?"));
    for (;;);
  }

  uint16_t firmware = checkFirmwareVersion();
  if (firmware < 0x113) {
    Serial.println(F("Wrong firmware version!"));
    for (;;);
  }

  displayMACAddress();

  Serial.println(F("\nDeleting old connection profiles"));
  if (!cc3000.deleteProfiles()) {
    Serial.println(F("Failed!"));
    while (1);
  }

  Wire.begin();
  initialized = true;
  Serial.println("Ready!");
}

void loop() {
  int sensorState = digitalRead(irSensor);
  int buttonState = digitalRead(buttonPort);
  
  digitalWrite(sensorStateLed, buttonState);
  
  if (buttonState == LOW) {
    Serial.println("Sending results!");
    postIt(counter);
    delay(1000);
  }
  
  if (previousSensorState == LOW && sensorState == HIGH) {
    startTime = millis();
    delay(LAG);
  } else if (previousSensorState == HIGH && sensorState == LOW) {
    counter++;
    
    Serial.print("Counter: ");
    Serial.println(counter);
    
    Serial.print("Duration: ");
    Serial.println(millis() - startTime);
    
    // delay to avoid double counts
    delay(LOG_INTERVAL);
  }

  previousSensorState = sensorState;
}


void postIt(int count) {
  /* Attempt to connect to an access point */
  char *ssid = WLAN_SSID;             /* Max 32 chars */
  // Connect to WiFi network
  Serial.print(F("\nAttempting to connect to ")); Serial.println(ssid);

  /* NOTE: Secure connections are not available in 'Tiny' mode! */
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while (1);
  }
  Serial.println(F("Connected!"));

  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(100);
  }

  // Set the website IP
  uint32_t ip = cc3000.IP2U32(216, 52, 233, 120);
  cc3000.printIPdotsRev(ip);

//  count += 5;
  // Prepare JSON for Xively & get length
  int length = 0;

  String data = "";
  data = data + "\n" + "{\"version\":\"1.0.0\",\"datastreams\" : [ {\"id\" : \"Count\",\"current_value\" : \"" + String(count) + "\"},"
         + "{\"id\" : \"Value\",\"current_value\" : \"" + String(-1) + "\"}]}";

  length = data.length();
  Serial.print("Data length");
  Serial.println(length);
  Serial.println();

  // Print request for debug purposes
  Serial.print("PUT /v2/feeds/");
  Serial.print(feedID);
  Serial.println(".json HTTP/1.0");
  Serial.println("Host: api.xively.com");
  Serial.print("X-ApiKey: ");
  Serial.println(API_key);
  Serial.print("Content-Length: ");
  Serial.println(length, DEC);
  Serial.print("Connection: close");
  Serial.println();
  Serial.print(data);
  Serial.println();

  // Send request
  Adafruit_CC3000_Client client = cc3000.connectTCP(ip, 80);
  if (client.connected()) {
    Serial.println("Connected!");
    client.println("PUT /v2/feeds/" + String(feedID) + ".json HTTP/1.0");
    client.println("Host: api.xively.com");
    client.println("X-ApiKey: " + String(API_key));
    client.println("Content-Length: " + String(length));
    client.print("Connection: close");
    client.println();
    client.print(data);
    client.println();
  } else {
    Serial.println(F("Connection failed"));
    return;
  }

  Serial.println(F("-------------------------------------"));
  while (client.connected()) {
    while (client.available()) {
      char c = client.read();
      Serial.print(c);
    }
  }
  client.close();
  Serial.println(F("-------------------------------------"));

  Serial.println(F("\n\nDisconnecting"));
  cc3000.disconnect();
  //delay(LOG_INTERVAL); // Pause 15 seconds
}


/**************************************************************************/
/*!
    @brief  Displays the driver mode (tiny of normal), and the buffer
            size if tiny mode is not being used

    @note   The buffer size and driver mode are defined in cc3000_common.h
*/
/**************************************************************************/
void displayDriverMode(void)
{
  #ifdef CC3000_TINY_DRIVER
    Serial.println(F("CC3000 is configure in 'Tiny' mode"));
  #else
    Serial.print(F("RX Buffer : "));
    Serial.print(CC3000_RX_BUFFER_SIZE);
    Serial.println(F(" bytes"));
    Serial.print(F("TX Buffer : "));
    Serial.print(CC3000_TX_BUFFER_SIZE);
    Serial.println(F(" bytes"));
  #endif
}

/**************************************************************************/
/*!
    @brief  Tries to read the CC3000's internal firmware patch ID
*/
/**************************************************************************/
uint16_t checkFirmwareVersion(void)
{
  uint8_t major, minor;
  uint16_t version;
  
#ifndef CC3000_TINY_DRIVER  
  if(!cc3000.getFirmwareVersion(&major, &minor))
  {
    Serial.println(F("Unable to retrieve the firmware version!\r\n"));
    version = 0;
  }
  else
  {
    Serial.print(F("Firmware V. : "));
    Serial.print(major); Serial.print(F(".")); Serial.println(minor);
    version = major; version <<= 8; version |= minor;
  }
#endif
  return version;
}

/**************************************************************************/
/*!
    @brief  Tries to read the 6-byte MAC address of the CC3000 module
*/
/**************************************************************************/
void displayMACAddress(void)
{
  uint8_t macAddress[6];
  
  if(!cc3000.getMacAddress(macAddress))
  {
    Serial.println(F("Unable to retrieve MAC Address!\r\n"));
  }
  else
  {
    Serial.print(F("MAC Address : "));
    cc3000.printHex((byte*)&macAddress, 6);
  }
}


/**************************************************************************/
/*!
    @brief  Tries to read the IP address and other connection details
*/
/**************************************************************************/
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}
////  Wiring connections
//  Sparkfun RedBoard $24.95 Amazon Prime ASIN: B00BFGZZJO
//  Adafruit Assembled Data Logging shield for Arduino $19.95 + $1.95 stackable headers
//  Sanyo eneloop 2000 mAh typical, 1900 mAh minimum, 1500 cycle, 8 pack AA, Ni-MH Pre-Charged Rechargeable Batteries $20.19 Model: SEC-HR3U8BPN
//  Adafruit Diffused Green 3mm LED (25 pack) - $4.95  ID: 779
//  Adafruit 9V battery clip with 5.5mm/2.1mm plug -  ID: 80  could be better with a 90 degree barrel plug for clearance $3
//  Adafruit Diffused Red 3mm LED (25 pack) -$4.95 ID: 777
//  Adafruit 0.56" 4-Digit 7-Segment Display w/I2C Backpack - White ID:1002 $12,95 plus shipping
//  Optional: Adafruit In-line power switch for 2.1mm barrel jack - ID: 1125  Watch for faulty female connector, had to replace. $3
//  220 ohm  resistor $0.99
//  Radio Shack battery holder $2.99 RadioShack® 8 “AA” Battery Holder Model: 270-407  | Catalog #: 270-407 $3
//
//  IR Sensor Sharp GP2Y0A710K Distance Sensor (100-550cm) - DFRobot.com, got on Amazon for $34.96 out the door.
//  Black x 2 -> Ground
//  Red x 2 -> +5V
//  Blue -> A0 3.3v max so fine with Seeeduino Stalker logic
//
//  Red Error LED
//  D8 -> Anode ->  1K Resistor -> Ground
//
//  Green Count LED
//  D9 -> Anode ->  1K Resistor -> Ground
//
//  Adafruit 0.56" 4-Digit 7-Segment Display w/I2C Backpack (optional)
//  Wiring to the matrix is really easy
//  Connect CLK SCL to the I2C clock - on Arduino UNO thats Analog #5, on the Leonardo it's Digital #3, on the Mega it's digital #21
//  Connect DAT SDA to the I2C data - on Arduino UNO thats Analog #4, on the Leonardo it's Digital #2, on the Mega it's digital #20
//  Connect GND to common ground
//  Connect VCC+ to power - 5V is best but 3V also seems to work for 3V microcontrollers.  3v is dimmer which is better
