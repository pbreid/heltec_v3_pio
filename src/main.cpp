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

// Signal quality indicators
#define SIGNAL_EXCELLENT 1
#define SIGNAL_GOOD 2
#define SIGNAL_POOR 3
#define SIGNAL_NONE 0

// Button pin
#define PRG_BTN_PIN 0  // PRG button on Heltec board

// LED pin - Heltec V3 uses GPIO35 for built-in LED
#define LED_PIN 35    // Built-in LED

// LoRa settings for Australia
#define FREQUENCY 915.0      // MHz
#define BANDWIDTH 62.5      // kHz
#define SPREADING_FACTOR 9   // 7-12
#define CODING_RATE 5        // 5-8
#define POWER 22             // dBm
#define PREAMBLE_LENGTH 12    // 6-65535

// Additional LoRa Parameters
const float DUTY_CYCLE = 0.15; // 5% duty cycle (typical regulatory requirement)
const int MAX_RETRIES = 2;     // Maximum number of transmission retries
const int BASE_BACKOFF = 750;  // Base backoff time in ms

// Communication protocol
#define TRANSMITTER_ADDRESS 0x01
#define RECEIVER_ADDRESS 0x02
#define CMD_TOGGLE_GATE 0x10
#define ACK_TOGGLE_GATE 0x11
#define STATUS_REQUEST 0x20
#define STATUS_REPORT 0x21
#define HEARTBEAT_REQUEST 0x30
#define HEARTBEAT_RESPONSE 0x31

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
unsigned long lastCommunicationTime = 0;
int retryCount = 0;
unsigned long lastTransmitTime = 0;
unsigned long calculatedAirTime = 0;
bool waitingForAck = false;

// Add these variables to track current radio parameters
uint8_t currentSpreadingFactor = SPREADING_FACTOR;
float currentBandwidth = BANDWIDTH;

unsigned long lastContactTime = 0;
String lastContactTimeStr = "--:--:--";

// Heartbeat variables
unsigned long lastHeartbeatTime = 0;
bool heartbeatActive = true;
#define HEARTBEAT_INTERVAL 15000  // Send heartbeat every 15 seconds

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
void updateTransmitterDisplay();
bool sendPacketWithRetry(String packet);
String formatTime(unsigned long timestamp);
void sendHeartbeat();

void updateLastContact() {
  lastContactTime = millis();
  lastContactTimeStr = formatTime(millis() - lastContactTime);
}

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
  
  // Initialize heartbeat variables
  lastHeartbeatTime = 0;
  heartbeatActive = true;
  
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
  updateTransmitterDisplay();
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
  
  // Send heartbeat periodically
  sendHeartbeat();
  
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
    updateTransmitterDisplay();
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
    lastButtonState = buttonState;
    toggleGate();
  }

  // // Check if enough time has passed since the last state change
  // if ((millis() - lastDebounceTime) > debounceDelay) {
  //   // If button state is stable and is a LOW transition (pressed)
  //   if (buttonState == LOW && lastButtonState == HIGH) {
  //     Serial.println("Button Press Detected! Toggling gate...");
  //     toggleGate();
  //   }
    
  //   // Update last button state after debounce period
  //   lastButtonState = buttonState;
  // }
}

void toggleGate() {
  Serial.println("Sending gate toggle command...");
  
  // Format packet: DESTINATION|SOURCE|COMMAND
  String packet = String(RECEIVER_ADDRESS) + "|" + String(TRANSMITTER_ADDRESS) + "|" + String(CMD_TOGGLE_GATE);
  
  // Reset retry counter
  retryCount = 0;
  ackReceived = false;
  
  // Clear any previous ACK state
  waitingForAck = false;
  
  // Set to standby before transmitting
  radio.standby();
  delay(10);
  
  // Send the packet with explicit length
  Serial.print("TX [" + packet + "] ");
  Serial.print("Sending bytes: ");
  for (size_t i = 0; i < packet.length(); i++) {
    Serial.printf("%02X ", packet[i]);
  }
  Serial.println();
  
  size_t packetLength = packet.length();
  Serial.printf("Packet length: %d bytes\n", packetLength);
  
  digitalWrite(LED_PIN, HIGH);
  int state = radio.transmit(packet.c_str(), packetLength);
  digitalWrite(LED_PIN, LOW);
  
  // Calculate air time for duty cycle
  calculatedAirTime = calculateLoRaAirTime(packetLength, currentSpreadingFactor, currentBandwidth);
  Serial.printf("Calculated airtime: %dms\n", (int)calculatedAirTime);
  
  lastTransmitTime = millis();
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("OK (%dms)\n", (int)calculatedAirTime);
    
    // Start listening for ACK
    waitingForAck = true;
    lastAckTime = millis();
    
    // Start receiving immediately
    radio.startReceive();
  } else {
    Serial.printf("Failed, code: %d\n", state);
  }
}


bool sendPacketWithRetry(String packet) {
  // Check if we're still waiting for an ACK
  if (waitingForAck) {
    return false;
  }
  
  // Check duty cycle compliance
  unsigned long currentTime = millis();
  unsigned long timeSinceLastTransmit = currentTime - lastTransmitTime;
  
  // Calculate required pause based on last air time and duty cycle
  unsigned long requiredPause = calculatedAirTime / DUTY_CYCLE;
  
  if (timeSinceLastTransmit < requiredPause) {
    Serial.printf("Waiting for duty cycle: %dms left\n", 
                 (int)(requiredPause - timeSinceLastTransmit));
    return false;
  }
  
  // Send the packet
  Serial.print("TX [" + packet + "] ");
  digitalWrite(LED_PIN, HIGH);
  
  // Debug output of what we're sending byte by byte
  Serial.print("Sending bytes: ");
  for (size_t i = 0; i < packet.length(); i++) {
    Serial.printf("%02X ", packet[i]);
  }
  Serial.println();
  
  // Set radio to idle before transmitting (helps avoid buffer contamination)
  radio.standby();
  delay(10);
  
  // Get the exact length of our packet
  size_t packetLength = packet.length();
  Serial.printf("Packet length: %d bytes\n", packetLength);
  
  unsigned long startTime = millis();
  // Explicitly use the packet length to avoid sending unnecessary data
  int state = radio.transmit(packet.c_str(), packetLength);
  unsigned long endTime = millis();
  
  digitalWrite(LED_PIN, LOW);
  
  // Calculate theoretical air time for duty cycle calculations
  // This is more accurate than measuring elapsed time, which includes processing overhead
  calculatedAirTime = calculateLoRaAirTime(packetLength, currentSpreadingFactor, currentBandwidth);
  Serial.printf("Calculated airtime: %dms\n", (int)calculatedAirTime);
  
  lastTransmitTime = endTime;
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("OK (%dms)\n", (int)calculatedAirTime);
    
    // Start listening for ACK
    waitingForAck = true;
    lastAckTime = millis();
    
    // Resume receiving
    radio.startReceive();
    return true;
  } else {
    Serial.printf("Failed, code: %d\n", state);
    return false;
  }
}

void checkTimeouts() {
  // Only check for timeout if we're waiting for an ACK
  if (waitingForAck) {
    if (ackReceived) {
      // ACK received, reset
      Serial.println("ACK received, waiting flag cleared");
      waitingForAck = false;
      retryCount = 0;
    } else if (millis() - lastAckTime >= ACK_TIMEOUT) {
      // ACK timeout
      Serial.println("ACK timeout!");
      
      // Check if we should retry
      if (retryCount < MAX_RETRIES) {
        retryCount++;
        
        // Exponential backoff
        int backoff = BASE_BACKOFF * (1 << retryCount) + random(100);
        
        Serial.printf("Retry %d/%d after %dms backoff\n", 
                     retryCount, MAX_RETRIES, backoff);
        
        // Wait for backoff period before retry
        delay(backoff);
        
        // Set radio to standby before retrying
        radio.standby();
        delay(10);
        
        // Retry sending the packet
        String packet = String(RECEIVER_ADDRESS) + "|" + 
                       String(TRANSMITTER_ADDRESS) + "|" + 
                       String(CMD_TOGGLE_GATE);
        
        Serial.print("TX [" + packet + "] ");
        Serial.print("Sending bytes: ");
        for (size_t i = 0; i < packet.length(); i++) {
          Serial.printf("%02X ", packet[i]);
        }
        Serial.println();
        
        digitalWrite(LED_PIN, HIGH);
        int state = radio.transmit(packet.c_str(), packet.length());
        digitalWrite(LED_PIN, LOW);
        
        if (state == RADIOLIB_ERR_NONE) {
          Serial.println("Retry sent OK");
          lastAckTime = millis(); // Reset ACK timeout
        } else {
          Serial.printf("Retry failed, code: %d\n", state);
        }
        
        // Resume receiving
        radio.startReceive();
      } else {
        // Max retries reached
        Serial.println("Max retries reached, giving up!");
        waitingForAck = false;
        retryCount = 0;
      }
    }
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

// Add this function to send heartbeat
void sendHeartbeat() {
  // Check if enough time has passed
  if (!heartbeatActive || millis() - lastHeartbeatTime < HEARTBEAT_INTERVAL) {
    return;
  }

  if (millis() - lastCommunicationTime < HEARTBEAT_INTERVAL) {
    // Update the heartbeat time without sending anything
    lastHeartbeatTime = millis();
    return;
  }
  
  Serial.println("Sending heartbeat...");
  
  // Format packet: DESTINATION|SOURCE|COMMAND
  String packet = String(RECEIVER_ADDRESS) + "|" + String(TRANSMITTER_ADDRESS) + "|" + String(HEARTBEAT_REQUEST);
  
  // Debug output of what we're sending byte by byte
  Serial.print("Sending bytes: ");
  for (size_t i = 0; i < packet.length(); i++) {
    Serial.printf("%02X ", packet[i]);
  }
  Serial.println();
  
  // Try to send with our existing function
  if (sendPacketWithRetry(packet)) {
    lastHeartbeatTime = millis();
    lastCommunicationTime = millis(); 
  }
}

void printBufferInfo(uint8_t* buffer, size_t size) {
  Serial.printf("Count: %d\tReceived bytes: ", size);
  for (size_t i = 0; i < min(size, (size_t)16); i++) {
    Serial.printf("%02X ", buffer[i]);
  }
  Serial.println();
}


void processReceivedPacket(uint8_t* buffer, size_t size) {
  if (size == 0) return;

  // Debug: Print buffer info
  Serial.printf("Count: %d\tReceived bytes: ", size);
  for (size_t i = 0; i < min(size, (size_t)16); i++) {
    Serial.printf("%02X ", buffer[i]);
  }
  Serial.println();

  // Get signal metrics immediately after reception
  lastRssi = radio.getRSSI();
  lastSnr = radio.getSNR();

  // We know our packet format is always "dest|source|command|data"
  // Let's explicitly look for the end of the data segment
  
  // Manually find valid pipes in the packet
  int pipePositions[3] = {-1, -1, -1};
  int pipeCount = 0;
  
  for (size_t i = 0; i < min(size, (size_t)32); i++) {
    if (buffer[i] == '|') {
      if (pipeCount < 3) {
        pipePositions[pipeCount] = i;
      }
      pipeCount++;
    }
  }
  
  // Determine the actual packet size based on pipe positions
  size_t actualPacketSize = 0;
  
  if (pipeCount >= 3 && pipePositions[2] != -1) {
    // We found at least 3 pipes, so we have a data field
    
    // Look for the end of the data segment (only digits 0-9)
    bool foundEndOfData = false;
    for (size_t i = pipePositions[2] + 1; i < min(size, (size_t)32); i++) {
      // If we find a character that's not a digit, that's the end
      if (buffer[i] < '0' || buffer[i] > '9') {
        actualPacketSize = i;
        foundEndOfData = true;
        break;
      }
    }
    
    // If we didn't find an end, use a reasonable default
    if (!foundEndOfData) {
      actualPacketSize = pipePositions[2] + 2; // Include at least one digit for data
    }
  } 
  else if (pipeCount >= 2 && pipePositions[1] != -1) {
    // We have a packet with just destination|source|command
    
    // Find the end of the command segment
    bool foundEndOfCommand = false;
    for (size_t i = pipePositions[1] + 1; i < min(size, (size_t)32); i++) {
      if (buffer[i] < '0' || buffer[i] > '9') {
        actualPacketSize = i;
        foundEndOfCommand = true;
        break;
      }
    }
    
    if (!foundEndOfCommand) {
      actualPacketSize = pipePositions[1] + 3; // Allow for command numbers up to 2 digits
    }
  }
  else {
    // Fallback - if pipe structure is completely messed up
    actualPacketSize = min(size, (size_t)16);
  }
  
  // Create a clean packet string from the buffer with proper termination
  char packetData[64] = {0};  
  size_t copySize = min(actualPacketSize, sizeof(packetData) - 1);
  memcpy(packetData, buffer, copySize);
  packetData[copySize] = '\0';

  String packet = String(packetData);

  Serial.print("RX [");
  Serial.print(packet);
  Serial.println("]");
  Serial.printf("RSSI: %.1f dBm, SNR: %.1f dB\n", lastRssi, lastSnr);
  Serial.printf("Actual packet size: %d bytes\n", actualPacketSize);

  // Skip duplicate heartbeat responses
  static String lastPacket = "";
  static unsigned long lastPacketTime = 0;
  
  // If this is a duplicate packet received within 200ms, ignore it
  if (packet == lastPacket && (millis() - lastPacketTime) < 200) {
    Serial.println("Skipping duplicate packet");
    return;
  }
  
  // Save this packet for duplicate detection
  lastPacket = packet;
  lastPacketTime = millis();

  // Robust packet parsing
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
    
    // Sanitize data string - keep only digits
    String cleanData = "";
    for (unsigned int i = 0; i < dataStr.length(); i++) {
      if (isDigit(dataStr.charAt(i))) {
        cleanData += dataStr.charAt(i);
      }
    }
    dataStr = cleanData;
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

  // Check if the packet is for us
  if (destination == TRANSMITTER_ADDRESS && source == RECEIVER_ADDRESS) {
    updateLastContact();
    
    // Process commands
    if (command == ACK_TOGGLE_GATE) {
      lastCommunicationTime = millis();
      Serial.println("✅ Acknowledgment Received!");
      ackReceived = true;
      waitingForAck = false;
      lastAckTime = 0;  
      
      // Update gate status if data is present
      if (dataStr.length() > 0) {
        gateOpen = (dataStr.toInt() == 1);
        Serial.printf("Gate Status: %s\n", gateOpen ? "OPEN" : "CLOSED");
      }
    }
    else if (command == STATUS_REPORT) {
      lastCommunicationTime = millis();
      Serial.println("✅ Status Report Received!");
      
      // Update gate status if data is present
      if (dataStr.length() > 0) {
        gateOpen = (dataStr.toInt() == 1);
        Serial.printf("Gate Status Updated: %s\n", gateOpen ? "OPEN" : "CLOSED");
      }
    }
    else if (command == HEARTBEAT_RESPONSE) {
      Serial.println("💓 Heartbeat Response Received!");
      
      // Update gate status if data is present
      if (dataStr.length() > 0) {
        gateOpen = (dataStr.toInt() == 1);
        Serial.printf("Gate Status from Heartbeat: %s\n", gateOpen ? "OPEN" : "CLOSED");
      }
    }
  } else {
    Serial.println("Packet not for us, ignoring");
  }
}

// Enhanced transmitter display
void updateTransmitterDisplay() {
  display.clear();
  
  // Header row with gate status and battery
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "Gate:");
  display.drawString(30, 0, gateOpen ? "OPEN" : "CLOSED");
  
  // Draw battery at top right
  drawBattery(display, 110, 2, 85);
  
  // Horizontal divider
  display.drawHorizontalLine(0, 12, 128);
  
  // Signal quality and metrics
  int quality = getSignalQuality(lastRssi, lastSnr);
  
  // Signal quality text and icon
  display.drawString(0, 15, "Signal:");
  
  String qualityText = "";
  switch (quality) {
    case SIGNAL_EXCELLENT: qualityText = "Excellent"; break;
    case SIGNAL_GOOD: qualityText = "Good"; break;
    case SIGNAL_POOR: qualityText = "Poor"; break;
    default: qualityText = "None"; break;
  }
  display.drawString(40, 15, qualityText);
  
  // Draw signal icon
  drawSignalStrength(display, 110, 15, quality);
  
  // Signal measurements - RSSI and SNR
  display.drawString(0, 27, "RSSI:");
  display.drawString(40, 27, String(lastRssi, 1) + "dBm");
  
  display.drawString(0, 39, "SNR:");
  display.drawString(40, 39, String(lastSnr, 1) + "dB");
  
  // Connection status - MOVED UP to use less vertical space
  bool isConnected = (millis() - lastContactTime) < (HEARTBEAT_INTERVAL * 3);
  display.drawString(75, 39, isConnected ? "CONN" : "DISC");
  
  // Last contact time - completely separated to bottom of screen
  display.drawHorizontalLine(0, 50, 128);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 52, "Last: " + formatTime(millis() - lastContactTime));
  
  display.display();
}