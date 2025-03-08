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
#define PRG_BTN_PIN 0  // PRG button on Heltec board

// LoRa settings for Australia
#define FREQUENCY 915.0      // MHz
#define BANDWIDTH 62.5      // kHz
#define SPREADING_FACTOR 9   // 7-12
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
#define HEARTBEAT_REQUEST 0x30
#define HEARTBEAT_RESPONSE 0x31

// Signal quality indicators
#define SIGNAL_EXCELLENT 1
#define SIGNAL_GOOD 2
#define SIGNAL_POOR 3
#define SIGNAL_NONE 0

// Additional LoRa Parameters
const float DUTY_CYCLE = 0.15; // 5% duty cycle (typical regulatory requirement)
const int MAX_RETRIES = 3;     // Maximum number of transmission retries
const int BASE_BACKOFF = 600;  // Base backoff time in ms

// Add these variables to track current radio parameters
uint8_t currentSpreadingFactor = SPREADING_FACTOR;
float currentBandwidth = BANDWIDTH;

// Status variables
bool gateOpen = false;

// RadioLib object
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);

unsigned long lastContactTime = 0;
String lastContactTimeStr = "--:--:--";

// Flag for received packet
volatile bool rxFlag = false;

// Forward declarations
void setFlag(void);
void processReceivedPacket(uint8_t* buffer, size_t size);
void sendStatusReport();
void sendHeartbeatResponse();
void toggleGate();
void updateReceiverDisplay();
String formatTime(unsigned long timestamp);
unsigned long calculateLoRaAirTime(uint8_t payloadSize, uint8_t sf, float bw);
int getSignalQuality(float rssi, float snr);
void drawBattery(SSD1306Wire &display, int x, int y, int percentage);
void drawSignalStrength(SSD1306Wire &display, int x, int y, int quality);
void drawBox(SSD1306Wire &display, int x, int y, int width, int height, String title);
void display_power(bool on);
void processRadioPackets(int delayTime);

// Update this in your processReceivedPacket function:
void updateLastContact() {
    Serial.println("Updating last contact time");
    lastContactTimeStr = formatTime(millis() - lastContactTime);
    lastContactTime = millis();
}

float lastRssi = 0;
float lastSnr = 0;
unsigned long lastResponseTime = 0;

// Function to calculate expected LoRa airtime (in ms)
// This is an approximation using the formula from LoRa Alliance
unsigned long calculateLoRaAirTime(uint8_t payloadSize, uint8_t sf, float bw) {
    // Convert bandwidth to Hz
    float bandwidth = bw * 1000;
    
    // Calculate symbol time
    float Tsym = pow(2, sf) / bandwidth;
    
    // Calculate number of symbols in payload
    // 8 is number of bits per byte, 4 is coding rate denominator, CR=4/5
    float payloadSymbNb = 8 + ceil((8.0 * payloadSize - 4.0 * sf + 28) / (4.0 * sf)) * 5;
    
    // Calculate time on air in ms
    float Tpkt = (PREAMBLE_LENGTH + 4.25) * Tsym + payloadSymbNb * Tsym;
    
    // Return milliseconds
    return ceil(Tpkt * 1000);
}

void handleButton() {
  static bool lastButtonState = HIGH;
  bool buttonState = digitalRead(PRG_BTN_PIN);

  // If button was released (HIGH) and is now pressed (LOW)
  if (buttonState == LOW && lastButtonState == HIGH) {
    // Debounce check
    //unsigned long currentTime = millis();
    //if (currentTime - lastButtonPressTime > BUTTON_DEBOUNCE_TIME) {
      //lastButtonPressTime = currentTime;
      
      // Simulate gate toggle
      Serial.println("Button pressed - Simulating gate toggle");
      toggleGate();
      
      // Send status update to transmitter
      sendStatusReport();
    //}
  }

  lastButtonState = buttonState;
}

// Helper function to determine signal quality from RSSI and SNR
int getSignalQuality(float rssi, float snr) {
    if (rssi == 0 && snr == 0) return SIGNAL_NONE;
    
    if (rssi > -100 && snr > 5) return SIGNAL_EXCELLENT;
    if (rssi > -110 && snr > 0) return SIGNAL_GOOD;
    return SIGNAL_POOR;
}

// Draw battery icon
void drawBattery(SSD1306Wire &display, int x, int y, int percentage) {
    // Battery outline
    display.drawRect(x, y, 12, 6);
    display.drawRect(x+12, y+1, 2, 4);
    
    // Battery level
    int width = map(percentage, 0, 100, 0, 10);
    if (width > 0) {
      display.fillRect(x+1, y+1, width, 4);
    }
}

// Format timestamp
String formatTime(unsigned long timestamp) {
    if (timestamp == 0) return "--:--:--";

    unsigned long seconds = timestamp / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    
    seconds %= 60;
    minutes %= 60;
    
    char timeStr[9];
    sprintf(timeStr, "%02lu:%02lu:%02lu", hours, minutes, seconds);
    return String(timeStr);
}

// Draw styled box
void drawBox(SSD1306Wire &display, int x, int y, int width, int height, String title) {
    display.drawRect(x, y, width, height);
    
    // Title background
    display.fillRect(x+1, y+1, width-2, 10);
    
    // Title text (inverted)
    display.setColor(BLACK);
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(x + width/2, y+1, title);
    display.setColor(WHITE);
}

// Function to display signal strength icon
void drawSignalStrength(SSD1306Wire &display, int x, int y, int quality) {
    switch (quality) {
      case SIGNAL_EXCELLENT:
        display.drawLine(x, y+6, x, y+6);
        display.drawLine(x+2, y+4, x+2, y+6);
        display.drawLine(x+4, y+2, x+4, y+6);
        display.drawLine(x+6, y, x+6, y+6);
        break;
      case SIGNAL_GOOD:
        display.drawLine(x, y+6, x, y+6);
        display.drawLine(x+2, y+4, x+2, y+6);
        display.drawLine(x+4, y+2, x+4, y+6);
        break;
      case SIGNAL_POOR:
        display.drawLine(x, y+6, x, y+6);
        display.drawLine(x+2, y+4, x+2, y+6);
        break;
      default:
        display.drawLine(x, y+6, x+6, y+6);
        display.drawLine(x+3, y+3, x+3, y+6);
        break;
    }
}

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

    pinMode(PRG_BTN_PIN, INPUT_PULLUP);
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


unsigned long lastUpdateTime = 0;
void loop() {
    handleButton();
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

    // Periodically update the screen
    if (millis() - lastUpdateTime > 500) {
        updateReceiverDisplay();
        lastUpdateTime = millis();
    }
}


void sendRadioPacket(String packet) {
  // Debug output of what we're sending byte by byte
  Serial.print("Sending bytes: ");
  for (size_t i = 0; i < packet.length(); i++) {
    Serial.printf("%02X ", packet[i]);
  }
  Serial.println();

  // Get the exact length of our packet
  size_t packetLength = packet.length();
  Serial.printf("Packet length: %d bytes\n", packetLength);

  // Set radio to idle before transmitting (helps avoid buffer contamination)
  radio.standby();
  delay(10);

  // Explicitly use the packet length
  int state = radio.transmit(packet.c_str(), packetLength);
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Packet sent successfully");
  } else {
    Serial.printf("Failed to send packet, code: %d\n", state);
  }
}

void processReceivedPacket(uint8_t* buffer, size_t size) {
  if (size == 0) return;
  
  // Get signal metrics immediately after reception
  lastRssi = radio.getRSSI();
  lastSnr = radio.getSNR();
  
  // Create a clean packet string from the buffer
  char packetData[256] = {0};
  size_t i = 0;
  size_t packetEndPos = 0;
  
  // Find the first non-printable character
  for (i = 0; i < size && i < sizeof(packetData) - 1; i++) {
    // Accept only standard ASCII characters (32-126)
    if (buffer[i] < 32 || buffer[i] > 126) {
      packetEndPos = i;
      break;
    }
    packetData[i] = (char)buffer[i];
  }
  
  // If we didn't find any terminator, use the whole buffer (up to max size)
  if (i == size || i == sizeof(packetData) - 1) {
    packetEndPos = min(size, sizeof(packetData) - 1);
  }
  
  // Ensure null-termination
  packetData[packetEndPos] = '\0';
  
  String packet = String(packetData);
  
  Serial.print("Received Packet: [");
  Serial.print(packet);
  Serial.println("]");
  Serial.printf("RSSI: %.1f dBm, SNR: %.1f dB\n", lastRssi, lastSnr);
  
  // Debug: Print raw bytes (only the valid part)
  Serial.print("Raw bytes: ");
  for (size_t i = 0; i < min(packetEndPos + 1, (size_t)16); i++) {
    Serial.printf("%02X ", buffer[i]);
  }
  Serial.println();
  
  // More robust packet parsing
  int firstDelim = packet.indexOf('|');
  if (firstDelim == -1) {
    Serial.println("Malformed packet, no delimiters found");
    return;
  }
  
  int secondDelim = packet.indexOf('|', firstDelim + 1);
  if (secondDelim == -1) {
    Serial.println("Malformed packet, only one delimiter found");
    return;
  }
  
  // Extract fields with clear validation
  String destStr = packet.substring(0, firstDelim);
  String sourceStr = packet.substring(firstDelim + 1, secondDelim);
  
  int thirdDelim = packet.indexOf('|', secondDelim + 1);
  String commandStr;
  String dataStr = "";
  
  if (thirdDelim > 0) {
    commandStr = packet.substring(secondDelim + 1, thirdDelim);
    dataStr = packet.substring(thirdDelim + 1);
    
    // Remove any non-alphanumeric characters from the data field
    for (int i = dataStr.length() - 1; i >= 0; i--) {
      char c = dataStr.charAt(i);
      if (!isDigit(c) && !isAlpha(c)) {
        dataStr.remove(i);
      }
    }
  } else {
    commandStr = packet.substring(secondDelim + 1);
  }
  
  // Convert to integers with validation, using trim() to remove whitespace
  destStr.trim();
  sourceStr.trim();
  commandStr.trim();
  byte destination = destStr.toInt();
  byte source = sourceStr.toInt();
  byte command = commandStr.toInt();
  
  Serial.printf("Parsed - Destination: %d, Source: %d, Command: %d, Data: %s\n", 
               destination, source, command, dataStr.c_str());
  
  // Debug print to confirm target check
  Serial.printf("Is dest=%d for me(%d)? From %d\n", destination, RECEIVER_ADDRESS, source);
  
  if (destination != RECEIVER_ADDRESS) {
    Serial.println("Packet not for us, ignoring");
    return;
  }
  
  // Update last contact time for any valid packet for us
  updateLastContact();

  // Process commands with random delay to avoid collisions
  if (command == CMD_TOGGLE_GATE) {
    Serial.println("✅ Toggle Gate Command Received!");
    gateOpen = !gateOpen;
    Serial.printf("Gate is now %s\n", gateOpen ? "OPEN" : "CLOSED");
  
    // Add small random delay (10-50ms) before responding
    delay(random(10, 50));
  
    // Send acknowledgment with gate status
    String ackPacket = String(TRANSMITTER_ADDRESS) + "|" + String(RECEIVER_ADDRESS) + "|" + String(ACK_TOGGLE_GATE) + "|" + String(gateOpen ? "1" : "0");
    Serial.print("🚀 Sending ACK: [");
    Serial.print(ackPacket);
    Serial.println("]");
    
    // Debug raw bytes being sent
    Serial.print("Sending ACK bytes: ");
    for (size_t i = 0; i < ackPacket.length(); i++) {
      Serial.printf("%02X ", ackPacket[i]);
    }
    Serial.println();
    
    // Set to standby before transmitting
    radio.standby();
    delay(10);
    
    // Send ACK with high priority
    int state = radio.transmit(ackPacket.c_str(), ackPacket.length());
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("First ACK sent successfully");
    } else {
      Serial.printf("Failed to send first ACK, code: %d\n", state);
    }
    
    delay(100); // Short delay between transmissions
    
    // Try a second time to be sure
    radio.standby();
    delay(10);
    state = radio.transmit(ackPacket.c_str(), ackPacket.length());
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("Second ACK sent successfully");
    } else {
      Serial.printf("Failed to send second ACK, code: %d\n", state);
    }
    
    // Resume receiving mode after sending
    radio.startReceive();
  
    // Update display
    updateReceiverDisplay();
  }
  else if (command == STATUS_REQUEST) {
    Serial.println("🚀 Status Request Received!");
    
    // Add small random delay
    delay(random(10, 50));
    
    sendStatusReport();
  }
  else if (command == HEARTBEAT_REQUEST) {
    Serial.println("💓 Heartbeat Request Received!");
    
    // Add small random delay
    delay(random(10, 50));
    
    sendHeartbeatResponse();
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

    updateReceiverDisplay();
}

void sendStatusReport() {
  String statusPacket = String(TRANSMITTER_ADDRESS) + "|" + String(RECEIVER_ADDRESS) + "|" + String(STATUS_REPORT) + "|" + String(gateOpen ? "1" : "0");
  Serial.print("🚀 Sending Status Report: [");
  Serial.print(statusPacket);
  Serial.println("]");

  // Set radio to standby before transmitting
  radio.standby();
  delay(10);

  // Send once for status report (no need for duplicates with regular heartbeats)
  int state = radio.transmit(statusPacket.c_str(), statusPacket.length());
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Status report sent successfully");
  } else {
    Serial.printf("Failed to send status report, code: %d\n", state);
  }
  
  // Resume receiving
  radio.startReceive();
}

void sendHeartbeatResponse() {
  String responsePacket = String(TRANSMITTER_ADDRESS) + "|" + String(RECEIVER_ADDRESS) + "|" + String(HEARTBEAT_RESPONSE) + "|" + String(gateOpen ? "1" : "0");
  Serial.print("🚀 Sending Heartbeat Response: [");
  Serial.print(responsePacket);
  Serial.println("]");

  // Add debug output of what we're sending byte by byte
  Serial.print("Sending bytes: ");
  for (size_t i = 0; i < responsePacket.length(); i++) {
    Serial.printf("%02X ", responsePacket[i]);
  }
  Serial.println();

  // Set radio to standby before transmitting
  radio.standby();
  delay(10);

  // Send only once for heartbeat (remove duplicate responses)
  int state = radio.transmit(responsePacket.c_str(), responsePacket.length());
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Heartbeat response sent successfully");
  } else {
    Serial.printf("Failed to send heartbeat response, code: %d\n", state);
  }
  
  // Resume receiving
  radio.startReceive();
}

void updateReceiverDisplay() {
  display.clear();
  
  // Header row with title and battery
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "Gate Receiver");
  
  // Battery indicator
  drawBattery(display, 100, 2, 75);
  
  // Horizontal line
  display.drawHorizontalLine(0, 12, 128);
  
  // Gate status with larger font
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 16, gateOpen ? "OPEN" : "CLOSED");
  
  // Line separator
  display.drawHorizontalLine(0, 35, 128);
  
  // Reset to smaller font for remaining items
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  
  // Signal quality
  int quality = getSignalQuality(lastRssi, lastSnr);
  
  // Draw signal icon
  drawSignalStrength(display, 3, 40, quality);
  
  // Signal quality text - keep it brief
  String qualityText = "";
  switch (quality) {
    case SIGNAL_EXCELLENT: qualityText = "Exc"; break;
    case SIGNAL_GOOD: qualityText = "Good"; break;
    case SIGNAL_POOR: qualityText = "Poor"; break;
    default: qualityText = "None"; break;
  }
  display.drawString(15, 38, qualityText);
  
  // Signal measurements - more compact
  display.drawString(50, 38, String(lastRssi, 0) + "dBm");
  display.drawString(90, 38, "SNR:" + String(lastSnr, 0));
  
  // Last contact time
  display.drawString(0, 52, "Last:");
  display.drawString(30, 52, formatTime(millis() - lastContactTime));
  
  // Connection status - more compact
  bool isConnected = (millis() - lastContactTime) < 60000;
  display.drawString(90, 52, isConnected ? "CONN" : "DISC");
  
  display.display();
}