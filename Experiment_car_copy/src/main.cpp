#include <Arduino.h>

/*
  4WD Mecanum Wheel Robot Car Base
  mec-robot-car.ino
  Drives 4-wheel mecanum car base
  Uses ESP32 DevKitC (other ESP32 boards will work)
  Uses 2 TB6612FNG Motor Drivers, also compatible with L298N
  Uses WiFi & ESP-NOW library
  Uses NeoPixelBus Library by Michael C. Miller - https://github.com/Makuna/NeoPixelBus
  
  
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/

// Include Libraries
#include <NeoPixelBus.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_task_wdt.h>



// Define Motor Connections
// Right Front Motor
#define MF_PWMA 19
#define MF_AI1 32
#define MF_AI2 23

// Left Front Motor
#define MF_PWMB 26
#define MF_BI1 33
#define MF_BI2 25

// Right Rear Motor
#define MR_PWMC 27
#define MR_AI1 12
#define MR_AI2 14

// Left Rear Motor
#define MR_PWMD 4
#define MR_BI1 13
#define MR_BI2 2

// Servo definitions
const int servoPin = 18;
const int servoChannel = 8;  // Use channel 8 for servo
const int servoFreq = 50;    // 50Hz for servo
const int servoResolution = 16; // 16-bit resolution for precise control
const int kickAngle = 50;

// Battery voltage monitoring
#define BATTERY_PIN 34  // ADC pin for battery voltage divider
// Voltage divider: R1=100k, R2=47k (actual measured adjustment)
// Calibrated conversion factor based on actual voltage readings
const float batteryConversionFactor = 3.36;

// Calculate duty cycles for 0° and kick angle (for 16-bit resolution)
// Servo pulse widths: 500us (0°) to 2400us (180°) at 50Hz (20ms period)
// Duty = (pulseWidth_us / 20000us) * 65536
const int servo0Deg = 1638;   // ~500us pulse
const int servo90Deg = 4915;  // ~1500us pulse

unsigned long previousMillis = 0;
const long kickDelay = 500;  // Time for single kick action
const long kickCooldown = 50;  // Delay between repeated kicks when button held

bool lastFlapState = 0;
bool kicking = false;
bool coolingDown = false;
unsigned long cooldownStart = 0;

void flapper();
float readCarBatteryVoltage();

//The x,y coordinates have been flipped
void motorControlMode0(int joyYaxis, int joyXaxis); 
void motorControlMode1(int joyYaxis);
void motorControlMode2(int joyYaxis);
void motorControlMode3(int joyYaxis);
void motorControlMode4(int joyYaxis);
void motorControlMode5(int joyYaxis);

// PWM Parameters for motor control
// PWM Frequency = 1KHz
const int mtrPWMFreq = 1000;
// PWM Resolution
const int mtrPWMResolution = 8;
// PWM Timer for motors (use Timer 0, leave Timer 2 for servo)
const int mtrPWMTimer = 0;
// Define PWM channels for each motor (channels 0-3 on Timer 0)
const int mtrRFpwmchannel = 0;
const int mtrLFpwmchannel = 1;
const int mtrRRpwmchannel = 2;
const int mtrLRpwmchannel = 3;

// Number of NeoPixels
const uint16_t PixelCount = 8;
// NeoPixel Pin
const uint8_t PixelPin = 5;

// Name each LED for easier reference
#define NEO_STATUS 0
#define NEO_STATUS1 2
#define NEO_STATUS2 4
#define NEO_STATUS3 6
#define NEO_RF 1
#define NEO_LF 7
#define NEO_RR 3
#define NEO_LR 5

// Colors for NeoPixels (black is off) - brightness reduced to 20%
RgbColor red(51, 0, 0);
RgbColor green(0, 51, 0);
RgbColor blue(0, 0, 51);
RgbColor yellow(51, 38, 0);
RgbColor violet(51, 0, 51);
RgbColor cyan(0, 51, 51);
RgbColor white(51, 51, 51);
RgbColor orange(51, 14, 0);
RgbColor lightblue(35, 43, 46);
RgbColor lightgreen(29, 48, 29);
RgbColor black(0);
RgbColor indigo(15, 0, 26);  // Flapper indicator color
RgbColor magenta(51, 0, 25);  // Speed limiter indicator color

// Create "strip" object representing NeoPixel string
// Using NeoEsp32Rmt1Ws2812xMethod explicitly specifies RMT channel 1 to avoid conflicts
NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt1Ws2812xMethod> strip(PixelCount, PixelPin);

// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = {0x08,0xD1,0xF9,0x6A,0x6E,0x94}; 

// Define a data structure for received data
typedef struct struct_message_rcv {
  int xAxis;
  int yAxis;
  bool pbSwitch;
  bool flap;
  bool rotate;
  bool last;
  bool speedLimitActive;
} struct_message_rcv;

// Create a structured object for received data
struct_message_rcv rcvData;

// Define a data structure for sent data
typedef struct struct_message_xmit {
  uint8_t motorMode;
  int mecanumMode;
  int mtrRF_PWM;
  int mtrLF_PWM;
  int mtrRR_PWM;
  int mtrLR_PWM;
  float batteryVoltage;
} struct_message_xmit;

// Create a structured object for sent data
struct_message_xmit xmitData;

// ESP-NOW Peer info
esp_now_peer_info_t peerInfo;

// Define ESP-NOW timeout value
#define SIGNAL_TIMEOUT 500
// Last Receive time
unsigned long lastRecvTime = 0;

// Define Bytes to represent Mecanum Wheel Modes
// Individual bits define TB6612FNG motor driver module input states
// B7 = MF_AI1, B6 = MF_AI2, B5 = MF_BI1, B4 = MF_BI2, B3 = MR_AI1, B2 = MR_AI2, B1 = MR_BI1, B0 = MR_BI2
const byte MEC_STRAIGHT_FORWARD = B10101010;
const byte MEC_STRAIGHT_BACKWARD = B01010101;
const byte MEC_SIDEWAYS_RIGHT = B01101001;
const byte MEC_SIDEWAYS_LEFT = B10010110;
const byte MEC_DIAGONAL_45 = B00101000;
const byte MEC_DIAGONAL_135 = B10000010;
const byte MEC_DIAGONAL_225 = B00010100;
const byte MEC_DIAGONAL_315 = B01000001;
const byte MEC_PIVOT_RIGHT_FORWARD = B00100010;
const byte MEC_PIVOT_RIGHT_BACKWARD = B00010001;
const byte MEC_PIVOT_LEFT_FORWARD = B10001000;
const byte MEC_PIVOT_LEFT_BACKWARD = B01000100;
const byte MEC_ROTATE_CLOCKWISE = B01100110;
const byte MEC_ROTATE_COUNTERCLOCKWISE = B10011001;
const byte MEC_PIVOT_SIDEWAYS_FRONT_RIGHT = B01100000;
const byte MEC_PIVOT_SIDEWAYS_FRONT_LEFT = B10010000;
const byte MEC_PIVOT_SIDEWAYS_REAR_RIGHT = B00001001;
const byte MEC_PIVOT_SIDEWAYS_REAR_LEFT = B00000110;

// Variable for Motor Mode
byte motorModeValue = B00000000;


// Variable for Mecanum Mode
// 0 = Standard, 1 = Rotate, 2 = Pivot Right, 4 = Pivot Left, 5 = Pivot Front, 6 = Pivot Rear
int mecanumModeValue = 0;

// Variables for Motor PWM speeds
int mtrRFpwmValue = 0;
int mtrLFpwmValue = 0;
int mtrRRpwmValue = 0;
int mtrLRpwmValue = 0;

// Variables for Joystick values
volatile int joyXaxis = 127;
volatile int joyYaxis = 127;

// Variable for Joystick pushbutton state
volatile bool joySwitchState = true;
volatile bool flapState = HIGH;
volatile bool flap = HIGH;
volatile bool rotate = HIGH;
volatile bool last = HIGH;

// Read car battery voltage from voltage divider
float readCarBatteryVoltage() {
  int adcValue = analogRead(BATTERY_PIN);
  // Convert ADC value (0-4095 for 12-bit) to voltage
  // Reference voltage is 3.3V
  // Account for voltage divider ratio (99k + 46k) / 46k = 3.1521
  float voltage = (adcValue / 4095.0) * 3.3 * batteryConversionFactor;
  
  // // Debug output
  // Serial.print("Car ADC: ");
  // Serial.print(adcValue);
  // Serial.print(" | Voltage: ");
  // Serial.print(voltage, 2);
  // Serial.println("V");
  
  return voltage;
}

void flapper(){

  unsigned long currentMillis = millis();

  if (!kicking && !coolingDown) {
    // Start kick - calculate duty cycle for kick angle
    int kickDuty = map(kickAngle, 0, 180, servo0Deg, servo90Deg * 2);
    ledcWrite(servoChannel, kickDuty);
    previousMillis = currentMillis;
    kicking = true;
    Serial.println("KICK START");
  } else if (kicking && currentMillis - previousMillis >= kickDelay) {
    // Finish kick - return to 0 degrees
    ledcWrite(servoChannel, servo0Deg);
    kicking = false;
    coolingDown = true;
    cooldownStart = currentMillis;
    Serial.println("KICK END - cooling down");
  } else if (coolingDown && currentMillis - cooldownStart >= kickCooldown) {
    // Cooldown complete
    coolingDown = false;
    Serial.println("Cooldown complete");
    
    // Check if signal is STILL high after cooldown
    // If yes, trigger will be detected on next loop iteration
    if (rcvData.flap) {
      Serial.println("Signal still high - ready for next kick");
    }
  }
  
  // Continuously maintain LED state during kicking/cooling
  // When idle, LED will mirror mode color via setLedStatus()
  if (kicking) {
    strip.SetPixelColor(NEO_STATUS2, indigo);
    strip.Show();
  } else if (coolingDown) {
    strip.SetPixelColor(NEO_STATUS2, black);
    strip.Show();
  }
}  


/*
  Mecanum Wheel Car - Functions
  a_car-functions.ino
  Functions for mecanum wheel robot car
  Used with mec-robot-car.ino
  
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/

void moveMotors(int speedRF, int speedLF, int speedRR, int speedLR, byte dircontrol) {

  // Serial.println("Full dircontrol");
  // Serial.println(dircontrol);
  // Moves all 4 motors
  // Directions specified in direction byte

  // Right Front Motor
  digitalWrite(MF_AI1, bitRead(dircontrol, 7));
  digitalWrite(MF_AI2, bitRead(dircontrol, 6));
  ledcWrite(mtrRFpwmchannel, abs(speedRF));
  // Serial.println("speedRF       dircontrol");
  // Serial.print(abs(speedRF));
  // Serial.print("\t");
  // Serial.print(bitRead(dircontrol, 7));
  // Serial.print("\t");
  // Serial.println(bitRead(dircontrol, 6));
  //analogWrite(mtrRFpwmchannel, abs(speedRF));

  // Left Front Motor
  digitalWrite(MF_BI1, bitRead(dircontrol, 5));
  digitalWrite(MF_BI2, bitRead(dircontrol, 4));
  ledcWrite(mtrLFpwmchannel, abs(speedLF));
  // Serial.println("speedLF       dircontrol");
  // Serial.print(abs(speedLF));
  // Serial.print("\t");
  // Serial.print(bitRead(dircontrol, 5));
  // Serial.print("\t");
  // Serial.println(bitRead(dircontrol, 4));
  //analogWrite(mtrLFpwmchannel, abs(speedLF));


  // Right Rear Motor
  digitalWrite(MR_AI1, bitRead(dircontrol, 3));
  digitalWrite(MR_AI2, bitRead(dircontrol, 2));
  ledcWrite(mtrRRpwmchannel, abs(speedRR));
  // Serial.println("speedRR       dircontrol");
  // Serial.print(abs(speedRR));
  // Serial.print("\t");
  // Serial.print(bitRead(dircontrol, 3));
  // Serial.print("\t");
  // Serial.println(bitRead(dircontrol, 2));
  //analogWrite(mtrRRpwmchannel, abs(speedRR));

  // Left Rear Motor
  digitalWrite(MR_BI1, bitRead(dircontrol, 1));
  digitalWrite(MR_BI2, bitRead(dircontrol, 0));
  ledcWrite(mtrLRpwmchannel, abs(speedLR));
  // Serial.println("speedLR       dircontrol");
  // Serial.print(abs(speedLR));
  // Serial.print("\t");
  // Serial.print(bitRead(dircontrol, 1));
  // Serial.print("\t");
  // Serial.println(bitRead(dircontrol, 0));
  //analogWrite(mtrLRpwmchannel, abs(speedLR));
}

void stopMotors() {

  // Stops all motors and motor controllers

  digitalWrite(MF_AI1, 0);
  digitalWrite(MF_AI2, 0);
  digitalWrite(MF_BI1, 0);
  digitalWrite(MF_BI2, 0);
  digitalWrite(MR_AI1, 0);
  digitalWrite(MR_AI2, 0);
  digitalWrite(MR_BI1, 0);
  digitalWrite(MR_BI2, 0);

  ledcWrite(mtrRFpwmchannel, 0);
  ledcWrite(mtrLFpwmchannel, 0);
  ledcWrite(mtrRRpwmchannel, 0);
  ledcWrite(mtrLRpwmchannel, 0);
}

void ledMotorStatus(byte dircontrol) {

  // Sets status LEDs to indicate motor direction

  // Right front
  if ((bitRead(dircontrol, 7) == 1) && (bitRead(dircontrol, 6) == 0)) {
    // Motor moves forward
    // Turn Right Front NeoPixel BLUE
    strip.SetPixelColor(NEO_RF, blue);
  } else if ((bitRead(dircontrol, 7) == 0) && (bitRead(dircontrol, 6) == 1)) {
    // Motor moves in reverse
    // Turn Right Front NeoPixel GREEN
    strip.SetPixelColor(NEO_RF, green);
  } else {
    // Motor is stopped
    // Turn Right Front NeoPixel RED
    strip.SetPixelColor(NEO_RF, red);
  }

  // Left front
  if ((bitRead(dircontrol, 5) == 1) && (bitRead(dircontrol, 4) == 0)) {
    // Motor moves forward
    // Turn Left Front NeoPixel BLUE
    strip.SetPixelColor(NEO_LF, blue);
  } else if ((bitRead(dircontrol, 5) == 0) && (bitRead(dircontrol, 4) == 1)) {
    // Motor moves in reverse
    // Turn Left Front NeoPixel GREEN
    strip.SetPixelColor(NEO_LF, green);
  } else {
    // Motor is stopped
    // Turn left Front NeoPixel RED
    strip.SetPixelColor(NEO_LF, red);
  }

  // Right rear
  if ((bitRead(dircontrol, 3) == 1) && (bitRead(dircontrol, 2) == 0)) {
    // Motor moves forward
    // Turn Right Rear NeoPixel BLUE
    strip.SetPixelColor(NEO_RR, blue);
  } else if ((bitRead(dircontrol, 3) == 0) && (bitRead(dircontrol, 2) == 1)) {
    // Motor moves in reverse
    // Turn Right Rear NeoPixel GREEN
    strip.SetPixelColor(NEO_RR, green);
  } else {
    // Motor is stopped
    // Turn Right Rear NeoPixel RED
    strip.SetPixelColor(NEO_RR, red);
  }

  // Left rear
  if ((bitRead(dircontrol, 1) == 1) && (bitRead(dircontrol, 0) == 0)) {
    // Motor moves forward
    // Turn Left Rear NeoPixel BLUE
    strip.SetPixelColor(NEO_LR, blue);
  } else if ((bitRead(dircontrol, 1) == 0) && (bitRead(dircontrol, 0) == 1)) {
    // Motor moves in reverse
    // Turn Left Rear NeoPixel GREEN
    strip.SetPixelColor(NEO_LR, green);
  } else {
    // Motor is stopped
    // Turn Left Rear NeoPixel RED
    strip.SetPixelColor(NEO_LR, red);
  }

  // Update
  strip.Show();
}

void ledMotorTurnOff() {
  // Turn off all Motor NeoPixels
  strip.SetPixelColor(NEO_RF, black);
  strip.SetPixelColor(NEO_LF, black);
  strip.SetPixelColor(NEO_RR, black);
  strip.SetPixelColor(NEO_LR, black);
  strip.Show();
}

void ledAllTurnOff() {
  // Turn off all NeoPixels (including Status LED)
  strip.SetPixelColor(NEO_STATUS, black);
  strip.SetPixelColor(NEO_STATUS1, black);
  strip.SetPixelColor(NEO_STATUS2, black);
  strip.SetPixelColor(NEO_STATUS3, black);
  strip.SetPixelColor(NEO_RF, black);
  strip.SetPixelColor(NEO_LF, black);
  strip.SetPixelColor(NEO_RR, black);
  strip.SetPixelColor(NEO_LR, black);
  strip.Show();
}

void ledAllStop() {
  // Turn all Motor NeoPixels RED
  strip.SetPixelColor(NEO_RF, red);
  strip.SetPixelColor(NEO_LF, red);
  strip.SetPixelColor(NEO_RR, red);
  strip.SetPixelColor(NEO_LR, red);
  strip.Show();
}

void ledError() {
  // Turn all LEDs RED when disconnected
  strip.SetPixelColor(NEO_RF, red);
  strip.SetPixelColor(NEO_LF, red);
  strip.SetPixelColor(NEO_RR, red);
  strip.SetPixelColor(NEO_LR, red);
  strip.SetPixelColor(NEO_STATUS, red);
  strip.SetPixelColor(NEO_STATUS1, red);
  strip.SetPixelColor(NEO_STATUS2, red);
  strip.SetPixelColor(NEO_STATUS3, red);
  strip.Show();
}

void setBatteryLed(float voltage) {
  // Set battery status LED based on voltage level
  // NEO_STATUS1 = Battery health indicator
  static unsigned long lastFlashTime = 0;
  static bool flashState = false;
  unsigned long currentMillis = millis();
  
  if (voltage > 7.2) {
    // Good - Green
    strip.SetPixelColor(NEO_STATUS1, green);
  } else if (voltage >= 7.0) {
    // Medium - Yellow
    strip.SetPixelColor(NEO_STATUS1, yellow);
  } else if (voltage >= 6.5) {
    // Low - Red
    strip.SetPixelColor(NEO_STATUS1, red);
  } else {
    // Critical - Flashing Red (500ms interval)
    if (currentMillis - lastFlashTime >= 500) {
      flashState = !flashState;
      lastFlashTime = currentMillis;
    }
    strip.SetPixelColor(NEO_STATUS1, flashState ? red : black);
  }
}

void setSpeedLimiterLed(bool speedLimited) {
  // Set speed limiter status LED
  // NEO_STATUS3 = Speed limiter indicator
  // Only override when speed limit is active, otherwise mirrors mode color
  if (speedLimited) {
    // Limited - Magenta
    strip.SetPixelColor(NEO_STATUS3, magenta);
    strip.Show();
  }
  // When not active, LED mirrors mode color via setLedStatus()
}

void setLedStatus(int statusVal) {
  // Set LED color based upon Mecanum Mode value
  // NEO_STATUS2 is reserved for flapper indicator, so we skip it when flapper is active
  // Color mapping matches remote display colors
  
  RgbColor statusColor;
  
  if (statusVal == 0) {  // Cyan - Standard Mode
    statusColor = cyan;
  } else if (statusVal == 1) {  // Green - Rotate Mode
    statusColor = green;
  } else if (statusVal == 2) {  // Orange - Front Pivot Mode
    statusColor = orange;
  } else if (statusVal == 3) {  // Violet - Rear Pivot Mode
    statusColor = violet;
  } else if (statusVal == 4) {  // Light Blue - Right Pivot Mode
    statusColor = lightblue;
  } else if (statusVal == 5) {  // Yellow - Left Pivot Mode
    statusColor = yellow;
  } else {  // Off (black) - Undefined Mode
    statusColor = black;
  }
  
  // Set status LEDs
  strip.SetPixelColor(NEO_STATUS, statusColor);
  // NEO_STATUS1 is handled by setBatteryLed()
  
  // NEO_STATUS2 replicates mode color when flapper is idle
  if (!kicking && !coolingDown) {
    strip.SetPixelColor(NEO_STATUS2, statusColor);
  }
  
  // NEO_STATUS3 replicates mode color when speed limiter is not active
  if (!rcvData.speedLimitActive) {
    strip.SetPixelColor(NEO_STATUS3, statusColor);
  }
  strip.Show();
}


/*
  Mecanum Wheel Car - Callback Functions
  b_callbacks.ino
  ESP-NOW callback functions for mecanum wheel robot car
  Used with mec-robot-car.ino
  
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/

// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  
}

void toggleMecanumMode(bool isRotateButton) {
  // Mode order: 0=Standard, 1=Rotate, 2=Front Pivot, 3=Rear Pivot, 4=Right Pivot, 5=Left Pivot
  
  int currentMecMode = mecanumModeValue;
  
  if (isRotateButton) {
    // Rotate button cycles: 0 → 1 → 2 → 0 (Standard → Rotate → Front Pivot → Standard)
    if (currentMecMode == 0) {
      mecanumModeValue = 1;  // Standard → Rotate
    } else if (currentMecMode == 1) {
      mecanumModeValue = 2;  // Rotate → Front Pivot
    } else if (currentMecMode == 2) {
      mecanumModeValue = 0;  // Front Pivot → Standard
    } else {
      // If in any other mode, go to mode 0
      mecanumModeValue = 0;
    }
  } else {
    // Joy switch cycles through all 6 modes: 0 → 1 → 2 → 3 → 4 → 5 → 0
    if (currentMecMode == 5) {
      mecanumModeValue = 0;
    } else {
      mecanumModeValue = currentMecMode + 1;
    }
  }
}


// Callback function executed when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  if (len == 0) {
    stopMotors();
    ledError();

    joyXaxis = 127;
    joyYaxis = 127;
    joySwitchState = true;
    flapState = HIGH;
    rotate = HIGH;
    return;
  }

  memcpy(&rcvData, incomingData, sizeof(rcvData));

  // Serial.print("X = ");
  // Serial.print(rcvData.xAxis);
  // Serial.print(", Y = ");
  // Serial.print(rcvData.yAxis);
  // Serial.print(", SW = ");
  // Serial.print(rcvData.pbSwitch);
  //Serial.print(", FS = ");
  //Serial.println(rcvData.flap);
  // Serial.print(", RT = ");
  // Serial.println(rcvData.rotate);

  joyXaxis = rcvData.xAxis;
  joyYaxis = rcvData.yAxis;
  joySwitchState = rcvData.pbSwitch;
  flapState = rcvData.flap;
  rotate = rcvData.rotate;
  //last = rcvData.last;  

  // Handle mode changes - rotate button and joy switch separately
  if (rotate == true) {
    toggleMecanumMode(true);  // Rotate button pressed
  } else if (joySwitchState == true) {
    toggleMecanumMode(false);  // Joy switch pressed
  }

  lastRecvTime = millis();
}


/*
  Mecanum Wheel Car - Mecanum driving functions
  c_mecanum-functions.ino
  Motor control and steering functions
  Used with mec-robot-car.ino
  
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/

//The X,Y values have been flipped
void driveMecanumMode() {

  // Pass control to proper Mecanum Mode function
  switch (mecanumModeValue) {

    case 0:
      // Standard driving mode, controlled by joystick
      motorControlMode0(joyYaxis, joyXaxis);
      break;

    case 1:
      // Rotate mode, controlled by joystick Y Axis only
      motorControlMode1(joyYaxis);
      break;

    case 2:
      // Front Pivot mode, controlled by joystick Y Axis only
      motorControlMode4(joyYaxis);
      break;

    case 3:
      // Rear Pivot mode, controlled by joystick Y Axis only
      motorControlMode5(joyYaxis);
      break;

    case 4:
      // Right Pivot mode, controlled by joystick Y Axis only
      motorControlMode2(joyYaxis);
      break;

    case 5:
      // Left Pivot mode, controlled by joystick Y Axis only
      motorControlMode3(joyYaxis);
      break;
  }
}

/*
  Mecanum Wheel Car - Mode 0 (Standard) driving functions
  d_mode-0.ino
  Allows control with joystick on remote control
  Used with mec-robot-car.ino
  
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/

// Function to control motors with joystick values - Standard Mode
void motorControlMode0(int xjoystick, int yjoystick) {

  // Variables to hold mode byte and speed values
  byte motorCtlMode = B00000000;
  int rf_speed = 0;
  int lf_speed = 0;
  int rr_speed = 0;
  int lr_speed = 0;

  // Define variables and map joystick values to ranges from -255 to +255
  int xaxis = map(xjoystick, 0, 254, -255, 255);
  int yaxis = map(yjoystick, 0, 254, -255, 255);

  // Determine Wheel Mode byte and speed values
  if ((xaxis == 0) && (yaxis > 0)) {
    //Straight Forward
    motorCtlMode = MEC_STRAIGHT_FORWARD;
    rf_speed = yaxis;
    lf_speed = yaxis;
    rr_speed = yaxis;
    lr_speed = yaxis;
  } else if ((xaxis == 0) && (yaxis < 0)) {
    //Straight Backward
    motorCtlMode = MEC_STRAIGHT_BACKWARD;
    rf_speed = yaxis;
    lf_speed = yaxis;
    rr_speed = yaxis;
    lr_speed = yaxis;
  } else if ((xaxis < 0) && (yaxis == 0)) {
    //Sideways Right
    motorCtlMode = MEC_SIDEWAYS_RIGHT;
    rf_speed = xaxis;
    lf_speed = xaxis;
    rr_speed = xaxis;
    lr_speed = xaxis;
  } else if ((xaxis > 0) && (yaxis == 0)) {
    //Sideways Left
    motorCtlMode = MEC_SIDEWAYS_LEFT;
    rf_speed = xaxis;
    lf_speed = xaxis;
    rr_speed = xaxis;
    lr_speed = xaxis;
  } else if ((xaxis < 0) && (yaxis > 0)) {
    //Diagonal 45 Degrees
    motorCtlMode = MEC_DIAGONAL_45;
    rf_speed = 0;
    lf_speed = xaxis;
    rr_speed = yaxis;
    lr_speed = 0;
  } else if ((xaxis > 0) && (yaxis > 0)) {
    //Diagonal 135 Degrees
    motorCtlMode = MEC_DIAGONAL_135;
    rf_speed = xaxis;
    lf_speed = 0;
    rr_speed = 0;
    lr_speed = yaxis;
  } else if ((xaxis > 0) && (yaxis < 0)) {
    //Diagonal 225 Degrees
    motorCtlMode = MEC_DIAGONAL_225;
    rf_speed = 0;
    lf_speed = yaxis;
    rr_speed = xaxis;
    lr_speed = 0;
  } else if ((xaxis < 0) && (yaxis < 0)) {
    //Diagonal 315 Degrees
    motorCtlMode = MEC_DIAGONAL_315;
    rf_speed = yaxis;
    lf_speed = 0;
    rr_speed = 0;
    lr_speed = xaxis;
  }

  // Drive motors & set LED colors (moveMotors will correct any negative speed values)
  moveMotors(rf_speed, lf_speed, rr_speed, lr_speed, motorCtlMode);
  ledMotorStatus(motorCtlMode);

  // Update values to send to remote
  motorModeValue = motorCtlMode;
  mtrRFpwmValue = rf_speed;
  mtrLFpwmValue = lf_speed;
  mtrRRpwmValue = rr_speed;
  mtrLRpwmValue = lr_speed;
}

/*
  Mecanum Wheel Car - Mode 1 (Rotate) driving functions
  e_mode-1.ino
  Allows control with joystick on remote control
  Used with mec-robot-car.ino
  
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/

// Function to control motors with joystick values - Rotate Mode
void motorControlMode1(int yjoystick) {

  // Variables to hold mode byte and speed values
  byte motorCtlMode = B00000000;
  int rf_speed = 0;
  int lf_speed = 0;
  int rr_speed = 0;
  int lr_speed = 0;

  // Define variables and map Y joystick values to ranges from -255 to +255
  int yaxis = map(yjoystick, 0, 254, -255, 255);

  // Determine if forward or reverse
  if (yaxis > 0) {
    // Forward
    motorCtlMode = MEC_ROTATE_CLOCKWISE;
    rf_speed = yaxis;
    lf_speed = yaxis;
    rr_speed = yaxis;
    lr_speed = yaxis;
  } else if (yaxis < 0) {
    // Reverse
    motorCtlMode = MEC_ROTATE_COUNTERCLOCKWISE;
    rf_speed = yaxis;
    lf_speed = yaxis;
    rr_speed = yaxis;
    lr_speed = yaxis;
  } else {
    // Stopped
    motorCtlMode = B00000000;
    rf_speed = 0;
    lf_speed = 0;
    rr_speed = 0;
    lr_speed = 0;
  }

  // Drive motors & set LED colors (moveMotors will correct any negative speed values)
  moveMotors(rf_speed, lf_speed, rr_speed, lr_speed, motorCtlMode);
  ledMotorStatus(motorCtlMode);

  // Update values to send to remote
  motorModeValue = motorCtlMode;
  mtrRFpwmValue = rf_speed;
  mtrLFpwmValue = lf_speed;
  mtrRRpwmValue = rr_speed;
  mtrLRpwmValue = lr_speed;
}

/*
  Mecanum Wheel Car - Modes 2 - 5 (Pivots) driving functions
  e_mode-2-5.ino
  Allows control with joystick on remote control
  Used with mec-robot-car.ino
  
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/

// Function to control motors with joystick values - Pivot Right Mode
void motorControlMode2(int yjoystick) {

  // Variables to hold mode byte and speed values
  byte motorCtlMode = B00000000;
  int rf_speed = 0;
  int lf_speed = 0;
  int rr_speed = 0;
  int lr_speed = 0;

  // Define variables and map Y joystick values to ranges from -255 to +255
  int yaxis = map(yjoystick, 0, 254, -255, 255);

  // Determine if forward or reverse
  if (yaxis > 0) {
    // Forward
    motorCtlMode = MEC_PIVOT_RIGHT_FORWARD;
    lf_speed = yaxis;
    lr_speed = yaxis;
  } else if (yaxis < 0) {
    // Reverse
    motorCtlMode = MEC_PIVOT_RIGHT_BACKWARD;
    lf_speed = yaxis;
    lr_speed = yaxis;
  } else {
    // Stopped
    motorCtlMode = B00000000;
    rf_speed = 0;
    lf_speed = 0;
    rr_speed = 0;
    lr_speed = 0;
  }

  // Drive motors & set LED colors (moveMotors will correct any negative speed values)
  moveMotors(rf_speed, lf_speed, rr_speed, lr_speed, motorCtlMode);
  ledMotorStatus(motorCtlMode);

  // Update values to send to remote
  motorModeValue = motorCtlMode;
  mtrRFpwmValue = rf_speed;
  mtrLFpwmValue = lf_speed;
  mtrRRpwmValue = rr_speed;
  mtrLRpwmValue = lr_speed;
}

// Function to control motors with joystick values - Pivot Left Mode
void motorControlMode3(int yjoystick) {

  // Variables to hold mode byte and speed values
  byte motorCtlMode = B00000000;
  int rf_speed = 0;
  int lf_speed = 0;
  int rr_speed = 0;
  int lr_speed = 0;

  // Define variables and map Y joystick values to ranges from -255 to +255
  int yaxis = map(yjoystick, 0, 254, -255, 255);

  // Determine if forward or reverse
  if (yaxis > 0) {
    // Forward
    motorCtlMode = MEC_PIVOT_LEFT_FORWARD;
    rf_speed = yaxis;
    rr_speed = yaxis;
  } else if (yaxis < 0) {
    // Reverse
    motorCtlMode = MEC_PIVOT_LEFT_BACKWARD;
    rf_speed = yaxis;
    rr_speed = yaxis;
  } else {
    // Stopped
    motorCtlMode = B00000000;
    rf_speed = 0;
    lf_speed = 0;
    rr_speed = 0;
    lr_speed = 0;
  }

  // Drive motors & set LED colors (moveMotors will correct any negative speed values)
  moveMotors(rf_speed, lf_speed, rr_speed, lr_speed, motorCtlMode);
  ledMotorStatus(motorCtlMode);

  // Update values to send to remote
  motorModeValue = motorCtlMode;
  mtrRFpwmValue = rf_speed;
  mtrLFpwmValue = lf_speed;
  mtrRRpwmValue = rr_speed;
  mtrLRpwmValue = lr_speed;
}

// Function to control motors with joystick values - Pivot Front Mode
void motorControlMode4(int yjoystick) {

  // Variables to hold mode byte and speed values
  byte motorCtlMode = B00000000;
  int rf_speed = 0;
  int lf_speed = 0;
  int rr_speed = 0;
  int lr_speed = 0;

  // Define variables and map Y joystick values to ranges from -255 to +255
  int yaxis = map(yjoystick, 0, 254, -255, 255);

  // Determine if forward or reverse
  if (yaxis > 0) {
    // Forward
    motorCtlMode = MEC_PIVOT_SIDEWAYS_FRONT_RIGHT;
    rf_speed = yaxis;
    lf_speed = yaxis;
  } else if (yaxis < 0) {
    // Reverse
    motorCtlMode = MEC_PIVOT_SIDEWAYS_FRONT_LEFT;
    rf_speed = yaxis;
    lf_speed = yaxis;
  } else {
    // Stopped
    motorCtlMode = B00000000;
    rf_speed = 0;
    lf_speed = 0;
    rr_speed = 0;
    lr_speed = 0;
  }

  // Drive motors & set LED colors (moveMotors will correct any negative speed values)
  moveMotors(rf_speed, lf_speed, rr_speed, lr_speed, motorCtlMode);
  ledMotorStatus(motorCtlMode);

  // Update values to send to remote
  motorModeValue = motorCtlMode;
  mtrRFpwmValue = rf_speed;
  mtrLFpwmValue = lf_speed;
  mtrRRpwmValue = rr_speed;
  mtrLRpwmValue = lr_speed;
}

// Function to control motors with joystick values - Pivot Rear Mode
void motorControlMode5(int yjoystick) {

  // Variables to hold mode byte and speed values
  byte motorCtlMode = B00000000;
  int rf_speed = 0;
  int lf_speed = 0;
  int rr_speed = 0;
  int lr_speed = 0;

  // Define variables and map Y joystick values to ranges from -255 to +255
  int yaxis = map(yjoystick, 0, 254, -255, 255);

  // Determine if forward or reverse
  if (yaxis > 0) {
    // Forward
    motorCtlMode = MEC_PIVOT_SIDEWAYS_REAR_RIGHT;
    rr_speed = yaxis;
    lr_speed = yaxis;
  } else if (yaxis < 0) {
    // Reverse
    motorCtlMode = MEC_PIVOT_SIDEWAYS_REAR_LEFT;
    rr_speed = yaxis;
    lr_speed = yaxis;
  } else {
    // Stopped
    motorCtlMode = B00000000;
    rf_speed = 0;
    lf_speed = 0;
    rr_speed = 0;
    lr_speed = 0;
  }

  // Drive motors & set LED colors (moveMotors will correct any negative speed values)
  moveMotors(rf_speed, lf_speed, rr_speed, lr_speed, motorCtlMode);
  ledMotorStatus(motorCtlMode);

  // Update values to send to remote
  motorModeValue = motorCtlMode;
  mtrRFpwmValue = rf_speed;
  mtrLFpwmValue = lf_speed;
  mtrRRpwmValue = rr_speed;
  mtrLRpwmValue = lr_speed;
}



void setup() {

    // Initialize the Task Watchdog with a 3-second timeout
  esp_task_wdt_init(3, true);  // 3 seconds, trigger panic (reset) if not fed

  // Add the current task (loop task) to the watchdog
  esp_task_wdt_add(NULL);  // NULL refers to the currently running task

  // Set up Serial Monitor
  Serial.begin(115200);

  // Set A/D resolution to 12-bits
  analogReadResolution(12);

  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Disable WiFi sleep mode
  WiFi.setSleep(false);

  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    return;
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
    return;
  }

  // Set all connections as outputs
  pinMode(MF_PWMA, OUTPUT);
  pinMode(MF_AI1, OUTPUT);
  pinMode(MF_AI2, OUTPUT);
  pinMode(MF_PWMB, OUTPUT);
  pinMode(MF_BI1, OUTPUT);
  pinMode(MF_BI2, OUTPUT);
  pinMode(MR_PWMC, OUTPUT);
  pinMode(MR_AI1, OUTPUT);
  pinMode(MR_AI2, OUTPUT);
  pinMode(MR_PWMD, OUTPUT);
  pinMode(MR_BI1, OUTPUT);
  pinMode(MR_BI2, OUTPUT);

  // Attach channels to PWM output pins (explicitly use Timer 0)

  ledcSetup(mtrRFpwmchannel, mtrPWMFreq, mtrPWMResolution);          
  ledcAttachPin(MF_PWMA, mtrRFpwmchannel);           
  ledcSetup(mtrLFpwmchannel, mtrPWMFreq, mtrPWMResolution);          
  ledcAttachPin(MF_PWMB, mtrLFpwmchannel);           
  ledcSetup(mtrRRpwmchannel, mtrPWMFreq, mtrPWMResolution);          
  ledcAttachPin(MR_PWMC, mtrRRpwmchannel);           
  ledcSetup(mtrLRpwmchannel, mtrPWMFreq, mtrPWMResolution);          
  ledcAttachPin(MR_PWMD, mtrLRpwmchannel);           

  //ledcAttachChannel(MF_PWMA, mtrPWMFreq, mtrPWMResolution, mtrRFpwmchannel);
  //ledcAttachChannel(MF_PWMB, mtrPWMFreq, mtrPWMResolution, mtrLFpwmchannel);
  //ledcAttachChannel(MR_PWMA, mtrPWMFreq, mtrPWMResolution, mtrRRpwmchannel);
  //ledcAttachChannel(MR_PWMB, mtrPWMFreq, mtrPWMResolution, mtrLRpwmchannel);

  // Enable watchdog timer with config struct

  
  // esp_task_wdt_config_t wdt_config = {
  //   .timeout_ms = 3000,        // 3 seconds
  //   .idle_core_mask = 0b11,    // Watch both CPU cores
  //   .trigger_panic = true      // Restart ESP32 if task hangs
  // };

  // esp_task_wdt_init(&wdt_config);   // Initialize watchdog with config
  // esp_task_wdt_add(NULL);           // Add current task to watchdog


  // Reset all the neopixels to an off state
  strip.Begin();
  strip.Show();

  delay(1000);

  // Turn all Motor LEDs red
  ledAllStop();

  // Stop all motors
  stopMotors();

  // Setup servo using native LEDC on channel 8
  ledcSetup(servoChannel, servoFreq, servoResolution);
  ledcAttachPin(servoPin, servoChannel);
  ledcWrite(servoChannel, servo0Deg);  // Initialize to 0 degrees
  
  // Initialize rcvData to prevent random triggers
  rcvData.xAxis = 127;
  rcvData.yAxis = 127;
  rcvData.pbSwitch = false;
  rcvData.flap = false;
  rcvData.rotate = false;
  rcvData.last = false;
  rcvData.speedLimitActive = false;

  delay(1000);
}

void loop() {

  //Check timer to see if signal is lost
  unsigned long now = millis();

  // Handle flapper - always update state machine if active
  if (kicking || coolingDown) {
    // Currently kicking or cooling down - update state machine
    flapper();
  } else if (rcvData.flap) {
    // Not kicking, not cooling, and signal is high - start new kick
    flapper();
  }

  if (now - lastRecvTime > SIGNAL_TIMEOUT)
  // Signal is lost
  {

    // Stop all motors
    stopMotors();

    // Put LEDs into Error mode
    ledError();

    // Set controller values to default
    joyXaxis = 127;
    joyYaxis = 127;
    joySwitchState = true;

  } else {

    // Read battery voltage
    float carBatteryVoltage = readCarBatteryVoltage();
    
    // Set battery status LED
    setBatteryLed(carBatteryVoltage);
    
    // Set speed limiter status LED
    setSpeedLimiterLed(rcvData.speedLimitActive);

    // Set LED color based upon mode
    setLedStatus(mecanumModeValue);

    // Drive car using current mode
    driveMecanumMode();

    // Format structured data to send back to controller
    xmitData.motorMode = motorModeValue;
    xmitData.mecanumMode = mecanumModeValue;
    xmitData.mtrRF_PWM = mtrRFpwmValue;
    xmitData.mtrLF_PWM = mtrLFpwmValue;
    xmitData.mtrRR_PWM = mtrRRpwmValue;
    xmitData.mtrLR_PWM = mtrLRpwmValue;
    xmitData.batteryVoltage = carBatteryVoltage;

    // Send message via ESP-NOW
    esp_err_t result2 = esp_now_send(broadcastAddress, (uint8_t *)&xmitData, sizeof(xmitData));
  }

  // Reset watchdog timer
  esp_task_wdt_reset();

  // Short delay
  delay(50);

  //   Serial.println("Received data:");
  // Serial.print("motorMode: ");
  // Serial.println(xmitData.motorMode);

  // Serial.print("mecanumMode: ");
  // Serial.println(xmitData.mecanumMode);

  // Serial.print("mtrRF_PWM: ");
  // Serial.println(xmitData.mtrRF_PWM);

  // Serial.print("mtrLF_PWM: ");
  // Serial.println(xmitData.mtrLF_PWM);

  // Serial.print("mtrRR_PWM: ");
  // Serial.println(xmitData.mtrRR_PWM);

  // Serial.print("mtrLR_PWM: ");
  // Serial.println(xmitData.mtrLR_PWM);

}