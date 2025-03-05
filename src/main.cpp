#include <Wire.h>
#include "SSD1306Wire.h"
#include <RadioLib.h>

// Display settings
#define DISPLAY_GEOMETRY GEOMETRY_128_64
#define VEXT GPIO_NUM_36
SSD1306Wire display(0x3c, SDA_OLED, SCL_OLED, DISPLAY_GEOMETRY);

// SX1262 connections
#define LORA_CS   8
#define LORA_DIO1 14
#define LORA_RST  12
#define LORA_BUSY 13

// LED pin - Heltec V3 uses GPIO35 for built-in LED
#define LED_PIN 35    

// LoRa settings for Australia
#define FREQUENCY 915.0      // MHz
#define BANDWIDTH 62.5      // kHz
#define SPREADING_FACTOR 12   // 7-12
#define CODING_RATE 5        // 5-8
#define POWER 22             // dBm
#define PREAMBLE_LENGTH 12    // 6-65535

// Communication protocol
#define TRANSMITTER_ADDRESS 0x01
#define RECEIVER_ADDRESS 0x02
#define CMD_TOGGLE_GATE 0x10
#define ACK_TOGGLE_GATE 0x11
#define STATUS_REQUEST 0x20
#define STATUS_REPORT 0x21

// Status variables
bool gateOpen = false;

// RadioLib object
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);

// Flag for received packet
volatile bool rxFlag = false;

// Forward declarations
void setFlag(void);
void processReceivedPacket(uint8_t* buffer, size_t size);
void sendStatusReport();
void toggleGate();
void updateDisplay();

// Interrupt service routine for packet reception
void setFlag(void) {
    rxFlag = true;
}

void display_power(bool on) {
  if (on) {
    pinMode(VEXT, OUTPUT);
    digitalWrite(VEXT, LOW);
    delay(5);
    pinMode(RST_OLED, OUTPUT);
    digitalWrite(RST_OLED, HIGH);
    delay(1);
    digitalWrite(RST_OLED, LOW);
    delay(20);
    digitalWrite(RST_OLED, HIGH);
  } else {
    pinMode(VEXT, INPUT);
    display.displayOff();
  }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Gate Controller - Receiver");

    // Initialize LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // ✅ Force OLED power ON
    display_power(true);
    display.init();
    display.setContrast(255);
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    display.clear();
    display.drawString(0, 0, "Gate Receiver");
    display.drawString(0, 12, "Initializing...");
    display.display();

    // Initialize SX1262
    Serial.print("Initializing SX1262... ");
    int state = radio.begin();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("Success!");
        display.drawString(0, 24, "Radio init: OK");
    } else {
        Serial.print("Failed, code: ");
        Serial.println(state);
        display.drawString(0, 24, "Radio failed! " + String(state));
        display.display();
        while (1) { delay(1000); }  // Halt if radio fails
    }

    // Set radio parameters
    radio.setFrequency(FREQUENCY);
    radio.setBandwidth(BANDWIDTH);
    radio.setSpreadingFactor(SPREADING_FACTOR);
    radio.setCodingRate(CODING_RATE);
    radio.setOutputPower(POWER);
    radio.setPreambleLength(PREAMBLE_LENGTH);
    
    // Attach interrupt for reception
    radio.setDio1Action(setFlag);
    
    // Start listening for packets
    Serial.println("Starting receiver...");
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("Receiver started!");
        display.drawString(0, 36, "Receiver started");
    } else {
        Serial.println("Failed to start receiver!");
        display.drawString(0, 36, "RX failed! " + String(state));
    }
    
    display.display();
}

// a function to process radio packets while we are delaying, instead of say a delay(1000) in loop()
void processRadioPackets(int delayTime) {
    unsigned long start = millis();
    while (millis() - start < delayTime) {
        if (rxFlag) {
            rxFlag = false;

            // Create buffer for packet
            uint8_t buffer[256];
            size_t size = sizeof(buffer);

            // Read the received data
            int state = radio.readData(buffer, size);

            // Check if packet was received correctly
            if (state == RADIOLIB_ERR_NONE) {
                processReceivedPacket(buffer, size);
            } else {
                Serial.print("Failed to read packet, code: ");
                Serial.println(state);
            }

            // Resume receiving
            radio.startReceive();
        }
    }
}



void loop() {
    // Check for received packets
    if (rxFlag) {
        rxFlag = false;

        // Create buffer for packet
        uint8_t buffer[256];
        size_t size = sizeof(buffer);

        // Read the received data
        int state = radio.readData(buffer, size);

        // Check if packet was received correctly
        if (state == RADIOLIB_ERR_NONE) {
            processReceivedPacket(buffer, size);
        } else {
            Serial.print("Failed to read packet, code: ");
            Serial.println(state);
        }

        // Resume receiving
        radio.startReceive();
    }
}

void processReceivedPacket(uint8_t* buffer, size_t size) {
  if (size == 0) return;

  char packetData[256] = {0};  
  size_t copySize = (size < sizeof(packetData) - 1) ? size : sizeof(packetData) - 1;
  memcpy(packetData, buffer, copySize);  
  packetData[copySize] = '\0';  

  String packet = String(packetData);

  Serial.print("Received Packet: [");
  Serial.print(packet);
  Serial.println("]");

  int firstDelim = packet.indexOf('|');
  int secondDelim = packet.indexOf('|', firstDelim + 1);
  int thirdDelim = packet.indexOf('|', secondDelim + 1);

  if (firstDelim == -1 || secondDelim == -1) {
      Serial.println("❌ Invalid packet format! Ignoring...");
      return;
  }

  byte destination = packet.substring(0, firstDelim).toInt();
  byte source = packet.substring(firstDelim + 1, secondDelim).toInt();
  byte command = packet.substring(secondDelim + 1, thirdDelim > 0 ? thirdDelim : packet.length()).toInt();

  Serial.printf("Parsed - Destination: %d, Source: %d, Command: %d\n", destination, source, command);

  if (destination != RECEIVER_ADDRESS) {
      //Serial.println("Packet not for us, ignoring.");
      return;
  }

  if (command == CMD_TOGGLE_GATE) {
      Serial.println("✅ Toggle Gate Command Received!");
      gateOpen = !gateOpen;
      Serial.printf("Gate is now %s\n", gateOpen ? "OPEN" : "CLOSED");

      // **Send acknowledgment immediately**
      String ackPacket = String(TRANSMITTER_ADDRESS) + "|" + String(RECEIVER_ADDRESS) + "|" + String(ACK_TOGGLE_GATE) + "|" + String(gateOpen ? "1" : "0");
      Serial.print("🚀 Sending ACK: [");
      Serial.print(ackPacket);
      Serial.println("]");
      radio.transmit(ackPacket.c_str(), ackPacket.length());

      updateDisplay();
  }
  if (command == STATUS_REQUEST) {
      Serial.println("🚀 Status Request Received!");
      sendStatusReport();
  } 
}


// Toggle the gate
void toggleGate() {
    gateOpen = !gateOpen;
    Serial.printf("Gate is now %s\n", gateOpen ? "OPEN" : "CLOSED");

    // Send acknowledgment
    String ackPacket = String(TRANSMITTER_ADDRESS) + "|" + String(RECEIVER_ADDRESS) + "|" + String(ACK_TOGGLE_GATE) + "|" + String(gateOpen ? "1" : "0");
    Serial.print("🚀 Sending ACK: [");
    Serial.print(ackPacket);
    Serial.println("]");

    // Send the acknowledgment, but after a short delay
    processRadioPackets(100);
    radio.transmit(ackPacket.c_str(), ackPacket.length());

    updateDisplay();
}

// Send a status report
void sendStatusReport() {
    String statusPacket = String(TRANSMITTER_ADDRESS) + "|" + String(RECEIVER_ADDRESS) + "|" + String(STATUS_REPORT) + "|" + String(gateOpen ? "1" : "0");
    Serial.print("🚀 Sending Status Report: [");
    Serial.print(statusPacket);
    Serial.println("]");

    radio.transmit(statusPacket.c_str(), statusPacket.length());
}

// Update OLED display
void updateDisplay() {
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_16);
    display.drawString(64, 0, "Gate Receiver");

    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 24, "Gate Status: " + String(gateOpen ? "OPEN" : "CLOSED"));

    display.display();
}
