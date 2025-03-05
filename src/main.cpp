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

// Button pin
#define PRG_BTN_PIN 0  // PRG button on Heltec board

// LED pin - Heltec V3 uses GPIO35 for built-in LED
#define LED_PIN 35    // Built-in LED

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
bool ackReceived = false;
unsigned long lastAckTime = 0;
unsigned long lastStatusRequestTime = 0;
const unsigned long ACK_TIMEOUT = 3000;       // 3 seconds timeout for acknowledgment
const unsigned long STATUS_REQUEST_INTERVAL = 30000;  // Request status every 30 seconds
float lastRssi = 0;
float lastSnr = 0;
unsigned long last_tx = 0;
unsigned long minimum_pause = 0;  // For duty cycle limits

// RadioLib object
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);

// Flag for received packet
volatile bool rxFlag = false;

// Forward declarations
void setFlag(void);
void handleButton();
void toggleGate();
void requestStatus();
void checkTimeouts();
void processReceivedPacket(uint8_t* buffer, size_t size);
void updateDisplay();

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

// Interrupt service routine for packet reception
void setFlag(void) {
  rxFlag = true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Gate Controller - Transmitter");
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize PRG button with pull-up
  pinMode(PRG_BTN_PIN, INPUT_PULLUP);
  
  // Initialize display
  display_power(true);
  display.init();
  display.setContrast(255);
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.clear();
  display.drawString(0, 0, "Gate Controller - TX");
  display.drawString(0, 12, "Initializing...");
  display.display();
  
  // Initialize SX1262
  Serial.print("Initializing SX1262... ");
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("success!");
    display.drawString(0, 24, "Radio init: OK");
  } else {
    Serial.print("failed, code: ");
    Serial.println(state);
    display.drawString(0, 24, "Radio failed! " + String(state));
    display.display();
    while(1) { delay(1000); }  // Halt if radio fails
  }
  
  // Set radio parameters
  Serial.print("Setting frequency... ");
  state = radio.setFrequency(FREQUENCY);
  
  Serial.print("Setting bandwidth... ");
  state = radio.setBandwidth(BANDWIDTH);
  
  Serial.print("Setting spreading factor... ");
  state = radio.setSpreadingFactor(SPREADING_FACTOR);
  
  Serial.print("Setting coding rate... ");
  state = radio.setCodingRate(CODING_RATE);
  
  Serial.print("Setting output power... ");
  state = radio.setOutputPower(POWER);
  
  Serial.println("Setting up interrupt... ");
  radio.setDio1Action(setFlag);

  Serial.println("Setting preamble length... ");
  radio.setPreambleLength(PREAMBLE_LENGTH);
  
  // Start listening
  Serial.println("Starting receiver... ");
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Receiver started!");
    display.drawString(0, 36, "Receiver started");
  } else {
    Serial.println("Failed to start receiver!");
    display.drawString(0, 36, "RX failed! " + String(state));
  }
  
  display.display();
  delay(1000);
  
  // Initial display update
  updateDisplay();
}

void loop() {
  // Check button
  handleButton();
  
  // Check timeouts
  checkTimeouts();
  
  // Request status periodically
  if (millis() - lastStatusRequestTime > STATUS_REQUEST_INTERVAL) {
    requestStatus();
  }
  
  // Check for received packets
  if (rxFlag) {
    rxFlag = false;
    
    // Create buffer for packet
    uint8_t buffer[256];
    size_t size = sizeof(buffer);
    
    // Read the received data
    int state = radio.readData(buffer, size);
    
    // Get signal metrics
    lastRssi = radio.getRSSI();
    lastSnr = radio.getSNR();
    
    if (state == RADIOLIB_ERR_NONE) {
      // Packet received successfully
      processReceivedPacket(buffer, size);
    } else {
      Serial.print("Failed to read packet, code: ");
      Serial.println(state);
    }
    
    
    // Resume receiving
    radio.startReceive();
  }
  
  // Update display
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 500) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }
}

void handleButton() {
  static bool lastButtonState = HIGH;
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50;

  bool buttonState = digitalRead(PRG_BTN_PIN);

  // If state changed, reset debounce timer
  if (buttonState != lastButtonState) {
      lastDebounceTime = millis();
      Serial.println("Button Press Detected! Toggling gate...");
      toggleGate();

      lastButtonState = buttonState;
  }

  // // Check if enough time has passed since the last state change
  // if ((millis() - lastDebounceTime) > debounceDelay) {
  //     // If button state is stable and is a HIGH to LOW transition
  //     if (buttonState == LOW && lastButtonState == HIGH) {
  //         Serial.println("Button Press Detected! Toggling gate...");
  //         toggleGate();
  //     }
      
  //     // Update last button state after debounce period
  //     lastButtonState = buttonState;
  // }
}

void toggleGate() {
  Serial.println("Sending gate toggle command...");

  String packet = String(RECEIVER_ADDRESS) + "|" + String(TRANSMITTER_ADDRESS) + "|" + String(CMD_TOGGLE_GATE);
  Serial.print("Sending Packet: [");
  Serial.print(packet);
  Serial.println("]");

  digitalWrite(LED_PIN, HIGH);
  int state = radio.transmit(packet.c_str(), packet.length());
  digitalWrite(LED_PIN, LOW);

  if (state == RADIOLIB_ERR_NONE) {
      Serial.println("✅ Packet Sent Successfully!");

      // **Short delay to allow receiver to process**
      delay(100);

      // **Start listening for ACK**
      radio.startReceive();
  } else {
      Serial.printf("❌ Failed to send packet, code: %d\n", state);
  }
}

void requestStatus() {
  Serial.println("Requesting gate status...");
  
  // Check if we can transmit (duty cycle compliance)
  if (millis() < last_tx + minimum_pause) {
    Serial.printf("Waiting for duty cycle (%i ms)\n", (int)(last_tx + minimum_pause - millis()));
    return;
  }
  
  // Format packet: DESTINATION|SOURCE|COMMAND
  String packet = String(RECEIVER_ADDRESS) + "|" + String(TRANSMITTER_ADDRESS) + "|" + String(STATUS_REQUEST);
  
  Serial.print("TX [" + packet + "] ");
  
  digitalWrite(LED_PIN, HIGH);
  unsigned long txTime = millis();
  
  // Send the packet
  int state = radio.transmit(packet.c_str(), packet.length());
  
  txTime = millis() - txTime;
  digitalWrite(LED_PIN, LOW);
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("OK (%i ms)\n", (int)txTime);
    
    // Set minimum pause for duty cycle compliance
    minimum_pause = txTime * 100;
    last_tx = millis();
    
    lastStatusRequestTime = millis();
  } else {
    Serial.printf("Failed, code: %i\n", state);
  }
  
  // Resume receiving
  radio.startReceive();
}

void checkTimeouts() {
  // Check for acknowledgment timeout
  if (!ackReceived && (millis() - lastAckTime < ACK_TIMEOUT) && (lastAckTime > 0)) {
    // Still waiting for ACK
  } 
  else if (!ackReceived && (millis() - lastAckTime >= ACK_TIMEOUT) && (lastAckTime > 0)) {
    // ACK timeout
    Serial.println("Acknowledgment timeout!");
    lastAckTime = 0;  // Reset timer
  }
}

void processReceivedPacket(uint8_t* buffer, size_t size) {
  if (size == 0) return;



  char packetData[256] = {0};  
  size_t copySize = (size < sizeof(packetData) - 1) ? size : sizeof(packetData) - 1;
  memcpy(packetData, buffer, copySize);  
  packetData[copySize] = '\0';  

  String packet = String(packetData);

  Serial.print("RX [");
  Serial.print(packet);
  Serial.println("]");

  int firstDelim = packet.indexOf('|');
  int secondDelim = packet.indexOf('|', firstDelim + 1);
  int thirdDelim = packet.indexOf('|', secondDelim + 1);

  if (firstDelim == -1 || secondDelim == -1) {
      //Serial.println("❌ Invalid packet format! Ignoring...");
      return;
  }

  byte destination = packet.substring(0, firstDelim).toInt();
  byte source = packet.substring(firstDelim + 1, secondDelim).toInt();
  byte command = packet.substring(secondDelim + 1, thirdDelim > 0 ? thirdDelim : packet.length()).toInt();

  // Ignore packets that we sent ourselves
  if (source == TRANSMITTER_ADDRESS) {
    Serial.println("⚠️ Ignoring self-received packet...");
    return;
  }



  Serial.printf("Parsed - Destination: %d, Source: %d, Command: %d\n", destination, source, command);

  // ✅ Ensure packet is meant for us
  if (destination != TRANSMITTER_ADDRESS) {
      //Serial.println("Packet not for us, ignoring.");
      return;
  }

  // ✅ Process acknowledgment
  if (command == ACK_TOGGLE_GATE) {
      Serial.println("✅ Acknowledgment Received!");
      ackReceived = true;  
      lastAckTime = 0;  
      
      // Update gate status
      byte status = (thirdDelim > 0) ? packet.substring(thirdDelim + 1).toInt() : 0;
      gateOpen = (status == 1);
      Serial.printf("Gate Status: %s\n", gateOpen ? "OPEN" : "CLOSED");
  }

  // ✅ Process status report
  if (command == STATUS_REPORT) {
      Serial.println("✅ Status Report Received!");
      
      // Update gate status
      byte status = (thirdDelim > 0) ? packet.substring(thirdDelim + 1).toInt() : 0;
      gateOpen = (status == 1);
      Serial.printf("Gate Status Updated: %s\n", gateOpen ? "OPEN" : "CLOSED");
  }
}

void updateDisplay() {
  display.clear();
  
  // Display header
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 0, "Gate Controller");
  
  // Draw separation line
  display.drawHorizontalLine(0, 20, 128);
  
  // Display info in smaller font
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  
  // Connection status
  display.drawString(0, 24, "Connection:");
  if (ackReceived || (lastAckTime > 0 && millis() - lastAckTime < ACK_TIMEOUT)) {
    display.drawString(70, 24, "Connected");
    display.drawString(0, 36, "RSSI: " + String(lastRssi, 1) + " dBm");
  } else {
    display.drawString(70, 24, "Waiting...");
  }
  
  // Gate status
  display.drawString(0, 48, "Gate: " + String(gateOpen ? "OPEN" : "CLOSED"));
  
  // Send to display
  display.display();
}