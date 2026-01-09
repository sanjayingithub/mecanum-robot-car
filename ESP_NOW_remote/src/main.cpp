/*
  4WD Mecanum Wheel Remote Control
  mec-robot-remote.ino
  Controls 4-wheel mecanum car base
  Uses ESP32 TTGO "T-Display" module with integrated TFT display and 2 pushbuttons 
  Uses ESP-NOW library
  Uses TFT_eSPI Library by Bodmer - https://github.com/Bodmer/TFT_eSPI
  Modify User_Setup.h file in library for TTGO-T-Display
  
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/

#include <Arduino.h>

// Include Libraries for ESP-NOW Communications
#include <esp_now.h>
#include <WiFi.h>

// Include libraries for display
#include <TFT_eSPI.h>
#include <SPI.h>

// Define Joystick Connections (some joystick models reverse X & Y)
#define X_AXIS_PIN 33
#define Y_AXIS_PIN 32
#define SWITCH_PIN 12
#define FLAP_PIN 15
#define ROTATE_PIN 2
#define SPEED_PIN 13
#define LAST_PIN 39

// Speed limiter max values (0-254 range)
const int speedLimitX = 100;
const int speedLimitY = 100;

// Battery voltage monitoring
#define BATTERY_PIN 34  // ADC pin for battery voltage divider
const float voltageConversionFactor = 2.0;  // Adjust based on voltage divider ratio
float remoteBatteryVoltage = 0.0;

// Battery voltage thresholds for color coding
const float CAR_BATTERY_THRESHOLD = 7.00;
const float REMOTE_BATTERY_THRESHOLD = 3.10;

// Define TTGO display built-in pushbuttons
#define BUTTON_DISPLAY_MODE 0    // Cycles through display modes
#define BUTTON_SCREEN_TOGGLE 35   // Turns screen on/off

// Objects for display & sprites
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);

// Display colors
#define displayColor1 0xFFFF  // White

#define displayColor2a 0x1E1E  // Cyan
#define displayColor2b 0x5566  // Green
#define displayColor2c 0xE3E3  // Orange
#define displayColor2d 0xFAFA  // Violet
#define displayColor2e 0xBEBE  // Light Blue
#define displayColor2f 0xE6E6  // Yellow

#define displayColor3 0x0000  // Black
#define displayColor4 0x2727  // Green Bar
#define displayColor5 0x1B1B  // Blue Bar
#define displayColor6 0xE8E8  // Red

// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = { 0xD4, 0xE9, 0xF4, 0xA3, 0x2F, 0xF4 };

// Define a data structure for received data
typedef struct struct_message_rcv {
  bool motorMode;
  int mecanumMode;
  int mtrRF_PWM;
  int mtrLF_PWM;
  int mtrRR_PWM;
  int mtrLR_PWM;
  float batteryVoltage;
} struct_message_rcv;

// Create a structured object for received data
struct_message_rcv rcvData;

// Create a structured object for sent data
typedef struct struct_message_xmit {
  int xAxis;
  int yAxis;
  bool pbSwitch;
  bool flap;
  bool rotate;
  bool last;
  bool speedLimitActive;
} struct_message_xmit;

// Create a structured object for sent data
struct_message_xmit xmitData;

// ESP-NOW Peer info
esp_now_peer_info_t peerInfo;

// Variable for Motor Mode
volatile byte motorModeValue = B00000000;

// Variable for Mecanum Mode
volatile int mecanumModeValue = 0;

// Variables for Motor PWM speeds
volatile int mtrRFpwmValue = 0;
volatile int mtrLFpwmValue = 0;
volatile int mtrRRpwmValue = 0;
volatile int mtrLRpwmValue = 0;

// Variable for battery voltage
volatile float batteryVoltage = 0.0;

// Variables for Joystick values
int joyXaxis = 127;
int joyYaxis = 127;

// Variable for Joystick pushbutton state
bool joySwitchState = HIGH;
bool flapState = HIGH;
bool rotate = HIGH;
bool last = HIGH;

// Speed limiter toggle state
bool speedLimitActive = false;

// Debounce delay in milliseconds
const unsigned long debounceDelay = 10;

// Button state tracking structure
struct ButtonState {
  bool lastReading;
  bool currentState;
  unsigned long lastDebounceTime;
  bool lastStableState;
};

ButtonState switchBtn = {HIGH, HIGH, 0, HIGH};
ButtonState rotateBtn = {HIGH, HIGH, 0, HIGH};
ButtonState speedBtn = {HIGH, HIGH, 0, HIGH};
ButtonState displayModeBtn = {HIGH, HIGH, 0, HIGH};
ButtonState screenToggleBtn = {HIGH, HIGH, 0, HIGH};
ButtonState lastBtn = {HIGH, HIGH, 0, HIGH};

// Variable for connection error  - HIGH is error state
volatile bool connectError = LOW;

// Variable for connection status string
String connectStatus = "NO INFO";

// Variable for display selection - 0 = Graph, 1 = Speed Display, 2 = Compact
volatile int displaySelect = 2;

// Variable for screen on/off state
volatile bool screenOn = true;

// Function declarations
int convertJoystickValues(int value, bool reverse);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void graphMotorSpeed(int opMode);
void displayMotorSpeed(int opMode);
void displayCompactMode(int opMode);
void splashScreen(String messageString, bool errStat);
void controlInfo();

// Debounced button read with edge detection
// Returns true only on button release (rising edge after debounce)
bool readButtonRelease(int pin, ButtonState &btnState) {
  bool reading = digitalRead(pin);
  bool buttonReleased = false;
  
  // If reading changed, reset debounce timer
  if (reading != btnState.lastReading) {
    btnState.lastDebounceTime = millis();
  }
  
  // Check if reading has been stable for debounce delay
  if ((millis() - btnState.lastDebounceTime) > debounceDelay) {
    // Update current state if it changed
    if (reading != btnState.currentState) {
      btnState.currentState = reading;
    }
    
    // Detect rising edge (button release, since INPUT_PULLUP)
    if (btnState.currentState == HIGH && btnState.lastStableState == LOW) {
      buttonReleased = true;
    }
    
    btnState.lastStableState = btnState.currentState;
  }
  
  btnState.lastReading = reading;
  return buttonReleased;
}

// Read remote battery voltage from built-in voltage divider
float readRemoteBatteryVoltage() {
  int adcValue = analogRead(BATTERY_PIN);
  // Convert ADC value (0-4095 for 12-bit) to voltage
  // Reference voltage is typically 3.3V
  float voltage = (adcValue / 4095.0) * 3.3 * voltageConversionFactor;
  
  // // Debug output
  // Serial.print("ADC Value: ");
  // Serial.print(adcValue);
  // Serial.print(" | Voltage: ");
  // Serial.print(voltage, 2);
  // Serial.println("V");
  
  return voltage;
}

// Interrupt Service Routines removed - using debounced polling instead

void setup() {
  // Set up Serial Monitor
  Serial.begin(115200);

  // Set joystick pin as input with Pullup
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(FLAP_PIN, INPUT_PULLUP);
  pinMode(ROTATE_PIN, INPUT_PULLUP);
  pinMode(SPEED_PIN, INPUT_PULLUP);
  pinMode(LAST_PIN, INPUT_PULLUP);

  // Set built-in pushbuttons as inputs with pullups
  pinMode(BUTTON_DISPLAY_MODE, INPUT_PULLUP);
  pinMode(BUTTON_SCREEN_TOGGLE, INPUT_PULLUP);

  // Set up backlight pin
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);  // Turn on backlight initially

  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Disable WiFi Sleep mode
  WiFi.setSleep(false);

  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    connectStatus = "ESP-NOW Error";
    connectError = HIGH;
    return;
  } else {
    connectStatus = "ESP-NOW OK";
    connectError = LOW;
  }

  // Register receive callback function
  esp_now_register_recv_cb(OnDataRecv);

  // Register the send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    connectStatus = "No peer added";
    connectError = HIGH;
    return;
  } else {
    connectStatus = "ESP-NOW Ready";
    connectError = LOW;
  }

  // Set A/D resolution to 12-bits
  analogReadResolution(12);

  // Initialize the display
  tft.init();
  tft.setRotation(1);

  // Create a sprite
  sprite.createSprite(320, 170);

  // Display a splash screen
  splashScreen(connectStatus, connectError);
  delay(2000);
  connectStatus = "Waiting for car";
  splashScreen(connectStatus, connectError);
  delay(2000);

  // Enter the Loop with connectError set HIGH to avoid intial display flicker
  connectError = HIGH;
}

void loop() {

  // Check screen toggle button with debouncing (toggle on release) - works everywhere
  if (readButtonRelease(BUTTON_SCREEN_TOGGLE, screenToggleBtn)) {
    screenOn = !screenOn;
  }
  
  // Always print battery voltage for testing (regardless of connection)
  int adcRaw = analogRead(BATTERY_PIN);
  float testVoltage = (adcRaw / 4095.0) * 3.3 * voltageConversionFactor;
  Serial.print("Remote ADC: ");
  Serial.print(adcRaw);
  Serial.print(" | Voltage: ");
  Serial.print(testVoltage, 2);
  Serial.print("V | Connected: ");
  Serial.println(connectError == LOW ? "YES" : "NO");

  // Check connection status
  if (connectError == LOW) {

    // Read remote battery voltage
    remoteBatteryVoltage = readRemoteBatteryVoltage();
    
    // Print received car battery voltage for comparison
    Serial.print("Car Battery: ");
    Serial.print(batteryVoltage, 2);
    Serial.println("V");
    Serial.println("---");

    // Check if screen is on before displaying
    if (screenOn) {
      // Turn on backlight
      digitalWrite(TFT_BL, HIGH);
      
      // Select display
      if (displaySelect == 0) {
        // Ensure landscape mode for graph
        tft.setRotation(1);
        sprite.deleteSprite();
        sprite.createSprite(320, 170);
        // Display motor speed graph
        graphMotorSpeed(mecanumModeValue);
      } else if (displaySelect == 1) {
        // Ensure landscape mode for speed animation
        tft.setRotation(1);
        sprite.deleteSprite();
        sprite.createSprite(320, 170);
        // Display motor speed "animation"
        displayMotorSpeed(mecanumModeValue);
      } else {
        // Switch to portrait mode for compact display
        tft.setRotation(0);
        sprite.deleteSprite();
        sprite.createSprite(170, 320);
        // Display compact mode
        displayCompactMode(mecanumModeValue);
      }
    } else {
      // Screen is off - clear display and turn off backlight
      digitalWrite(TFT_BL, LOW);
      sprite.fillSprite(displayColor3);
      sprite.pushSprite(0, 0);
    }

    // Get joystick values and convert them
    joyXaxis = convertJoystickValues(analogRead(X_AXIS_PIN), false);
    joyYaxis = convertJoystickValues(analogRead(Y_AXIS_PIN), false);

    // Apply speed limiter if active
    if (speedLimitActive) {
      // Cap values at configured limits (scale from 0-254 range to 0-limit range)
      joyXaxis = map(joyXaxis, 0, 254, 127-(speedLimitX/2), 127+(speedLimitX/2));
      joyYaxis = map(joyYaxis, 0, 254, 127-(speedLimitY/2), 127+(speedLimitY/2));
    }

  } else {
    // Check if screen is on before displaying splash screen
    if (screenOn) {
      digitalWrite(TFT_BL, HIGH);
      // Ensure landscape mode for splash screen
      tft.setRotation(1);
      sprite.deleteSprite();
      sprite.createSprite(320, 170);
      splashScreen(connectStatus, HIGH);
    } else {
      digitalWrite(TFT_BL, LOW);
      sprite.fillSprite(displayColor3);
      sprite.pushSprite(0, 0);
    }

    // Send "zero" values as joystick data
    joyXaxis = 127;
    joyYaxis = 127;
  }

  // Check mode switch button with debouncing (trigger on release)
  if (readButtonRelease(SWITCH_PIN, switchBtn)) {
    joySwitchState = true;
  } else {
    joySwitchState = false;
  }

  // Check flap button (direct read for continuous action)
  if (digitalRead(FLAP_PIN) == LOW) {
    flapState = true;
  } else {
    flapState = false;
  }

  // Check rotate button with debouncing (trigger on release)
  if (readButtonRelease(ROTATE_PIN, rotateBtn)) {
    rotate = true;
  } else {
    rotate = false;
  }

  // Check last button with debouncing (trigger on release)
  if (readButtonRelease(LAST_PIN, lastBtn)) {
    last = true;
  } else {
    last = false;
  }

  // Check speed limit button with debouncing (toggle on release)
  if (readButtonRelease(SPEED_PIN, speedBtn)) {
    speedLimitActive = !speedLimitActive;
  }

  // Check display mode button with debouncing (cycle on release)
  if (readButtonRelease(BUTTON_DISPLAY_MODE, displayModeBtn)) {
    displaySelect = (displaySelect + 1) % 3;
  }

  // Format structured data
  xmitData.xAxis = joyXaxis;
  xmitData.yAxis = joyYaxis;
  xmitData.pbSwitch = joySwitchState;
  xmitData.flap = flapState;
  xmitData.rotate = rotate;
  xmitData.last = last;
  xmitData.speedLimitActive = speedLimitActive;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&xmitData, sizeof(xmitData));

  // Small delay for loop timing
  if (rotate == true) {
    delay(100);
  } else {
    delay(10);
  }
}

/*
  Mecanum Wheel Remote Control - functions
  a_remote-functions.ino
  Functions for remote control
  
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/
// Map 12-bit A/D values to 8-bit PWM values
// Allow for gap in center to account for joystick inaccuracies
int convertJoystickValues(int value, bool reverse) {
  if (value >= 2200) {
    // Joystick pushed forward
    value = map(value, 2200, 4095, 127, 254);
  } else if (value <= 1800) {
    // Joystick pulled back
    value = map(value, 1800, 0, 127, 0);
  } else {
    // Joystick in center
    value = 127;
  }

  // Check direction
  if (reverse) {
    value = 254 - value;
  }
  return value;
}

/*
  Mecanum Wheel Remote Control - ESP-NOW Callbacks
  b_callbacks.ino
  Callbacks for communication with mecanum wheel robot car
  
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/
// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

  if (status == ESP_NOW_SEND_SUCCESS) {
    connectStatus = "Car found";
    connectError = LOW;
  } else {
    connectStatus = "Car not found";
    connectError = HIGH;
  }
}

// Callback function executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {

  // Get receievd data
  memcpy(&rcvData, incomingData, sizeof(rcvData));

  // Pass received values to local variables
  motorModeValue = rcvData.motorMode;
  mecanumModeValue = rcvData.mecanumMode;
  mtrRFpwmValue = rcvData.mtrRF_PWM;
  mtrLFpwmValue = rcvData.mtrLF_PWM;
  mtrRRpwmValue = rcvData.mtrRR_PWM;
  mtrLRpwmValue = rcvData.mtrLR_PWM;
  batteryVoltage = rcvData.batteryVoltage;

  //Serial.println(mecanumModeValue);
}

/*
  Mecanum Wheel Remote Control - TFT Display Graph Functions
  c_graphs.ino
  Graphs used on remote control TFT display
  
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/

void graphMotorSpeed(int opMode) {
  // Display motor speeds as bargarph
  // Background color changes with operating mode
  // Graph colors change with motor direction

  int backColor;
  String headerText;

  if (opMode == 0) {
    backColor = displayColor2a;
    headerText = "Standard Mecanum Mode";
  } else if (opMode == 1) {
    backColor = displayColor2b;
    headerText = "Rotate Mode";
  } else if (opMode == 2) {
    backColor = displayColor2c;
    headerText = "Pivot Front Mode";
  } else if (opMode == 3) {
    backColor = displayColor2d;
    headerText = "Pivot Rear Mode";
  } else if (opMode == 4) {
    backColor = displayColor2e;
    headerText = "Pivot Right Mode";
  } else if (opMode == 5) {
    backColor = displayColor2f;
    headerText = "Pivot Left Mode";
  } else {
    backColor = displayColor2f;
    headerText = "Standard Mecanum Mode";
  }


  // Fill the sprite with color & add rectangles
  sprite.fillSprite(displayColor3);
  sprite.fillRect(0, 0, 300, 160, displayColor1);
  sprite.fillRoundRect(5, 20, 230, 110, 5, backColor);
  sprite.fillRect(40, 30, 2, 80, displayColor1);
  sprite.fillRect(40, 110, 190, 2, displayColor1);

  // Text for top
  sprite.setTextDatum(4);
  sprite.setTextColor(displayColor3, displayColor1);
  sprite.drawString(headerText, 114, 10, 2);

  // Text for left sidebar (increment lables)
  sprite.setTextColor(displayColor1, backColor);
  for (int i = 2; i < 8; i = i + 2) {
    sprite.drawString(String(i * 40), 22, 105 - (i * 10));
    for (int j = 0; j < 197; j = j + 5)
      sprite.drawPixel(40 + j, 105 - (i * 10), displayColor1);
  }

  // Text for bottom
  sprite.drawString("RF", 70, 121, 2);
  sprite.drawString("LF", 115, 121, 2);
  sprite.drawString("RR", 160, 121, 2);
  sprite.drawString("LR", 205, 121, 2);

  // Display battery voltages in top right corner
  // sprite.setTextColor(displayColor1, displayColor3);
  // sprite.setTextDatum(8);  // Right aligned
  // String carBatteryText = "Car: " + String(batteryVoltage, 1) + "V";
  // String remoteBatteryText = "Rem: " + String(remoteBatteryVoltage, 1) + "V";
  // sprite.drawString(carBatteryText, 100, 50, 2);
  // sprite.drawString(remoteBatteryText, 150, 100, 2);
  // sprite.setTextDatum(4);  // Reset to center aligned

  // Bargraph values

  byte barMotorMode = motorModeValue;

  int barvalueRF = abs(mtrRFpwmValue);
  int barvalueLF = abs(mtrLFpwmValue);
  int barvalueRR = abs(mtrRRpwmValue);
  int barvalueLR = abs(mtrLRpwmValue);

  // Map results to bargraph range
  barvalueRF = map(barvalueRF, 0, 255, 0, 75);
  barvalueLF = map(barvalueLF, 0, 255, 0, 75);
  barvalueRR = map(barvalueRR, 0, 255, 0, 75);
  barvalueLR = map(barvalueLR, 0, 255, 0, 75);

  // Set bargraph colors by motor direction
  if ((bitRead(barMotorMode, 7) == 1) && (bitRead(barMotorMode, 6) == 0)) {
    sprite.fillRect(60, 110 - barvalueRF, 16, barvalueRF, displayColor5);
  } else if ((bitRead(barMotorMode, 7) == 0) && (bitRead(barMotorMode, 6) == 1)) {
    sprite.fillRect(60, 110 - barvalueRF, 16, barvalueRF, displayColor4);
  }

  if ((bitRead(barMotorMode, 5) == 1) && (bitRead(barMotorMode, 4) == 0)) {
    sprite.fillRect(105, 110 - barvalueLF, 16, barvalueLF, displayColor5);
  } else if ((bitRead(barMotorMode, 5) == 0) && (bitRead(barMotorMode, 4) == 1)) {
    sprite.fillRect(105, 110 - barvalueLF, 16, barvalueLF, displayColor4);
  }

  if ((bitRead(barMotorMode, 3) == 1) && (bitRead(barMotorMode, 2) == 0)) {
    sprite.fillRect(150, 110 - barvalueRR, 16, barvalueRR, displayColor5);
  } else if ((bitRead(barMotorMode, 3) == 0) && (bitRead(barMotorMode, 2) == 1)) {
    sprite.fillRect(150, 110 - barvalueRR, 16, barvalueRR, displayColor4);
  }

  if ((bitRead(barMotorMode, 1) == 1) && (bitRead(barMotorMode, 0) == 0)) {
    sprite.fillRect(195, 110 - barvalueLR, 16, barvalueLR, displayColor5);
  } else if ((bitRead(barMotorMode, 1) == 0) && (bitRead(barMotorMode, 0) == 1)) {
    sprite.fillRect(195, 110 - barvalueLR, 16, barvalueLR, displayColor4);
  }

  // Paint display
  sprite.pushSprite(0, 0);
}

void displayMotorSpeed(int opMode) {
  // Display of speed and directon of all motors

  int backColor;
  String headerText;

  if (opMode == 0) {
    backColor = displayColor2a;
    headerText = "Standard Mecanum Mode";
  } else if (opMode == 1) {
    backColor = displayColor2b;
    headerText = "Rotate Mode";
  } else if (opMode == 2) {
    backColor = displayColor2c;
    headerText = "Pivot Front Mode";
  } else if (opMode == 3) {
    backColor = displayColor2d;
    headerText = "Pivot Rear Mode";
  } else if (opMode == 4) {
    backColor = displayColor2e;
    headerText = "Pivot Right Mode";
  } else if (opMode == 5) {
    backColor = displayColor2f;
    headerText = "Pivot Left Mode";
  } else {
    backColor = displayColor2f;
    headerText = "Standard Mecanum Mode";
  }

  // Fill the sprite with color & add "car" rectangle
  sprite.fillSprite(backColor);
  sprite.fillRect(85, 30, 70, 90, displayColor1);

  // Text for top
  sprite.setTextDatum(4);
  sprite.setTextColor(displayColor3, backColor);
  sprite.drawString(headerText, 114, 10, 2);

  // Display battery voltages in top right corner
  sprite.setTextDatum(8);  // Right aligned
  String carBatteryText = "Car: " + String(batteryVoltage, 1) + "V";
  String remoteBatteryText = "Rem: " + String(remoteBatteryVoltage, 1) + "V";
  sprite.drawString(carBatteryText, 315, 5, 2);
  sprite.drawString(remoteBatteryText, 315, 20, 2);
  sprite.setTextDatum(4);  // Reset to center aligned

  // Speed values
  byte speedMotorMode = motorModeValue;

  String speedvalueRF = String(abs(mtrRFpwmValue));
  String speedvalueLF = String(abs(mtrLFpwmValue));
  String speedvalueRR = String(abs(mtrRRpwmValue));
  String speedvalueLR = String(abs(mtrLRpwmValue));

  // Set "wheel" colors by motor direction
  // Right Front
  if ((bitRead(speedMotorMode, 7) == 1) && (bitRead(speedMotorMode, 6) == 0)) {
    sprite.fillRoundRect(145, 35, 20, 30, 2, displayColor5);
  } else if ((bitRead(speedMotorMode, 7) == 0) && (bitRead(speedMotorMode, 6) == 1)) {
    sprite.fillRoundRect(145, 35, 20, 30, 2, displayColor4);
  } else {
    sprite.fillRoundRect(145, 35, 20, 30, 2, displayColor6);
    speedvalueRF = "OFF";
  }

  // Left Front
  if ((bitRead(speedMotorMode, 5) == 1) && (bitRead(speedMotorMode, 4) == 0)) {
    sprite.fillRoundRect(75, 35, 20, 30, 2, displayColor5);
  } else if ((bitRead(speedMotorMode, 5) == 0) && (bitRead(speedMotorMode, 4) == 1)) {
    sprite.fillRoundRect(75, 35, 20, 30, 2, displayColor4);
  } else {
    sprite.fillRoundRect(75, 35, 20, 30, 2, displayColor6);
    speedvalueLF = "OFF";
  }

  // Right Rear
  if ((bitRead(speedMotorMode, 3) == 1) && (bitRead(speedMotorMode, 2) == 0)) {
    sprite.fillRoundRect(145, 85, 20, 30, 2, displayColor5);
  } else if ((bitRead(speedMotorMode, 3) == 0) && (bitRead(speedMotorMode, 2) == 1)) {
    sprite.fillRoundRect(145, 85, 20, 30, 2, displayColor4);
  } else {
    sprite.fillRoundRect(145, 85, 20, 30, 2, displayColor6);
    speedvalueRR = "OFF";
  }

  // Left Rear
  if ((bitRead(speedMotorMode, 1) == 1) && (bitRead(speedMotorMode, 0) == 0)) {
    sprite.fillRoundRect(75, 85, 20, 30, 2, displayColor5);
  } else if ((bitRead(speedMotorMode, 1) == 0) && (bitRead(speedMotorMode, 0) == 1)) {
    sprite.fillRoundRect(75, 85, 20, 30, 2, displayColor4);
  } else {
    sprite.fillRoundRect(75, 85, 20, 30, 2, displayColor6);
    speedvalueLR = "OFF";
  }

  // Display speed values
  sprite.drawString(speedvalueRF, 195, 50, 4);
  sprite.drawString(speedvalueLF, 40, 50, 4);
  sprite.drawString(speedvalueRR, 195, 100, 4);
  sprite.drawString(speedvalueLR, 40, 100, 4);

  // Paint display
  sprite.pushSprite(0, 0);
}

void displayCompactMode(int opMode) {
  // Compact display mode in portrait orientation (170x320)
  // Shows battery voltages, motor speeds (numbers only), and status indicators

  int backColor;
  String headerText;

  if (opMode == 0) {
    backColor = displayColor2a;
    headerText = "Standard Mecanum";
  } else if (opMode == 1) {
    backColor = displayColor2b;
    headerText = "Rotate Mode";
  } else if (opMode == 2) {
    backColor = displayColor2c;
    headerText = "Pivot Front";
  } else if (opMode == 3) {
    backColor = displayColor2d;
    headerText = "Pivot Rear";
  } else if (opMode == 4) {
    backColor = displayColor2e;
    headerText = "Pivot Right";
  } else if (opMode == 5) {
    backColor = displayColor2f;
    headerText = "Pivot Left";
  } else {
    backColor = displayColor2f;
    headerText = "Standard Mecanum";
  }

  // Fill the entire sprite with background color first
  sprite.fillSprite(backColor);

  // Text for top - mode header (centered horizontally at 85px for 170px width)
  sprite.setTextDatum(4);  // Center aligned
  sprite.setTextColor(displayColor3, backColor);
  sprite.drawString(headerText, 65, 20, 2);

  // Display battery voltages below header with color coding
  sprite.setTextDatum(4);  // Center aligned
  
  // Car battery
  int carBatteryColor = (batteryVoltage >= CAR_BATTERY_THRESHOLD) ? displayColor4 : displayColor6;  // Green or Red
  sprite.setTextColor(carBatteryColor, backColor);
  String carBatteryText = "Car: " + String(batteryVoltage, 1) + "V";
  sprite.drawString(carBatteryText, 65, 55, 4);
  
  // Remote battery
  int remoteBatteryColor = (remoteBatteryVoltage >= REMOTE_BATTERY_THRESHOLD) ? displayColor4 : displayColor6;  // Green or Red
  sprite.setTextColor(remoteBatteryColor, backColor);
  String remoteBatteryText = "Rem: " + String(remoteBatteryVoltage, 1) + "V";
  sprite.drawString(remoteBatteryText, 65, 80, 4);

  // Motor speed values
  byte speedMotorMode = motorModeValue;

  String speedvalueLF = String(abs(mtrLFpwmValue));
  String speedvalueRF = String(abs(mtrRFpwmValue));
  String speedvalueLR = String(abs(mtrLRpwmValue));
  String speedvalueRR = String(abs(mtrRRpwmValue));

  int colorLF, colorRF, colorLR, colorRR;

  // Left Front color
  if ((bitRead(speedMotorMode, 5) == 1) && (bitRead(speedMotorMode, 4) == 0)) {
    colorLF = displayColor5;  // Blue - forward
  } else if ((bitRead(speedMotorMode, 5) == 0) && (bitRead(speedMotorMode, 4) == 1)) {
    colorLF = displayColor4;  // Green - reverse
  } else {
    colorLF = displayColor6;  // Red - stopped
  }

  // Right Front color
  if ((bitRead(speedMotorMode, 7) == 1) && (bitRead(speedMotorMode, 6) == 0)) {
    colorRF = displayColor5;  // Blue - forward
  } else if ((bitRead(speedMotorMode, 7) == 0) && (bitRead(speedMotorMode, 6) == 1)) {
    colorRF = displayColor4;  // Green - reverse
  } else {
    colorRF = displayColor6;  // Red - stopped
  }

  // Left Rear color
  if ((bitRead(speedMotorMode, 1) == 1) && (bitRead(speedMotorMode, 0) == 0)) {
    colorLR = displayColor5;  // Blue - forward
  } else if ((bitRead(speedMotorMode, 1) == 0) && (bitRead(speedMotorMode, 0) == 1)) {
    colorLR = displayColor4;  // Green - reverse
  } else {
    colorLR = displayColor6;  // Red - stopped
  }

  // Right Rear color
  if ((bitRead(speedMotorMode, 3) == 1) && (bitRead(speedMotorMode, 2) == 0)) {
    colorRR = displayColor5;  // Blue - forward
  } else if ((bitRead(speedMotorMode, 3) == 0) && (bitRead(speedMotorMode, 2) == 1)) {
    colorRR = displayColor4;  // Green - reverse
  } else {
    colorRR = displayColor6;  // Red - stopped
  }

  // Display motor speeds in vertical layout (portrait orientation)
  sprite.setTextDatum(4);  // Center aligned
  
  // Motor labels
  sprite.setTextColor(displayColor3, backColor);
  sprite.drawString("LF", 40, 110, 2);
  sprite.drawString("RF", 90, 110, 2);
  sprite.drawString("LR", 40, 160, 2);
  sprite.drawString("RR", 90, 160, 2);
  
  // Motor speed values in 2x2 grid
  // Top row: LF (left) | RF (right)
  sprite.setTextColor(colorLF, backColor);
  sprite.drawString(speedvalueLF, 40, 130, 4);
  sprite.setTextColor(colorRF, backColor);
  sprite.drawString(speedvalueRF, 90, 130, 4);
  
  // Bottom row: LR (left) | RR (right)
  sprite.setTextColor(colorLR, backColor);
  sprite.drawString(speedvalueLR, 40, 180, 4);
  sprite.setTextColor(colorRR, backColor);
  sprite.drawString(speedvalueRR, 90, 180, 4);

  // Bottom indicators
  sprite.setTextColor(displayColor6, backColor);
  sprite.setTextDatum(4);  // Center aligned
  
  // FLAP indicator - only show when active
  if (flapState) {
    sprite.drawString("FLAP", 90, 220, 2);
  }
  
  // SPEED indicator - only show when active
  if (speedLimitActive) {
    sprite.drawString("SPEED", 40, 220, 2);
  }

  // Paint display
  sprite.pushSprite(0, 0);
}

/*
  Mecanum Wheel Remote Control - TFT Display Screen Functions
  d_screens.ino
  Display screens used on remote control TFT display
  
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/

void splashScreen(String messageString, bool errStat) {
  // Display a splash screen with a text message box
  // Background color depends upon error status
  if (errStat == HIGH) {
    // Fill the sprite with color & add rectangles
    sprite.fillSprite(displayColor6);

    // Text for top
    sprite.setTextDatum(4);
    sprite.setTextColor(displayColor1, displayColor6);
    sprite.drawString("Error", 114, 25, 4);
    sprite.drawString("Connecting", 114, 50, 4);

  } else {

    // Fill the sprite with color & add rectangles
    sprite.fillSprite(displayColor2b);

    // Text for top
    sprite.setTextDatum(4);
    sprite.setTextColor(displayColor1, displayColor2b);
    sprite.drawString("ESP-NOW", 114, 25, 4);
    sprite.drawString("Mecanum Car", 114, 50, 4);
  }

  // Box & text for status
  sprite.fillRoundRect(5, 70, 230, 40, 5, displayColor1);
  sprite.setTextColor(displayColor3, displayColor1);
  sprite.drawString(messageString, 114, 92, 4);

  // Paint display
  sprite.pushSprite(0, 0);
}

void controlInfo() {
  // System information display
}
