# Build Log & Documentation

## 📝 Project Overview

This is a derived and enhanced version of the Mecanum Wheel Robot Car originally designed by [DroneBot Workshop](https://dronebotworkshop.com). The original tutorial provided an excellent foundation, and this build includes several modifications and improvements based on real-world testing and usage.

**Original Tutorial:** [DroneBot Workshop - Mecanum Wheel Robot Car](https://dronebotworkshop.com/mecanum/)

> **Special Thanks:** A huge thank you to DroneBot Workshop for the comprehensive tutorial and detailed explanations that made this project possible!

---

## 🛠️ Build Process & Modifications

### Mechanical Components

#### Frame & Chassis
- **Material:** Laser-cut acrylic/MDF panels
- **Process:** 
  - Designed frame layout for mecanum wheel configuration
  - Sent design for laser cutting service
  - **Required extensive sanding** as the model i used has some problem with tolerences and the motor mounting pieces dont fit. 
  - Assembly with M3 standoffs and screws

**💡 Recommendation:** Consider purchasing a **pre-made mecanum wheel car kit** instead of sourcing individual components. It's often:
- **Cheaper** than buying parts separately
- Comes with properly fitted parts
- Saves significant time on fabrication
- Eliminates alignment issues

Popular kit options:
- 4WD Mecanum Wheel Robot Car Kit (various sellers on AliExpress, Amazon)
- Pre-assembled chassis with motor mounts
- Often includes motors and wheels

#### Custom 3D Printed Parts

##### 1. Motor-to-Wheel Coupler
- **Purpose:** Connect DC motors to mecanum wheel hubs
- **Design Considerations:**
  - The output from BO motors dont match the Maccanum wheels hole. So 3D printed a coupler.
- **Material:** PLA/PETG
- **Print Settings:**
  - Layer height: 0.2mm
  - Infill: 50-80% (needs to handle torque)
  - Supports: Required for shaft interface
  - Orient the part in a sleeping postion to improve strength and prevent breaking due to torque.
  
**Issues Faced:**
- Initial couplers were too tight - had to sand down motor shaft interface
- Wheel wobble due to loose fit - added rubber plumbers tape to compensate
- They were breaking so used adaldite to sequite them to motor.

**Files:** `3D_Models/motor_coupler.stl` *(add your file)*

##### 2. Servo Flapper Mechanism
- **Purpose:** Ball kicking mechanism mounted on servo
- **Design:**
  - Servo horn attachment point
  - Extended arm for reach
  - Reinforced impact surface
- **Material:** PETG (more flexible and impact-resistant than PLA)
- **Print Settings:**
  - Layer height: 0.2mm
  - Infill: 80% (needs strength for impacts)
  - Print orientation: Flat to ensure layer strength perpendicular to impact

**Issues Faced:**
- PLA version cracked after repeated impacts - so better switched to PETG
- Initial design too long - reduced servo strain by shortening arm
- Servo jitter under load - adjusted PWM frequency

**Files:** `3D_Models/flapper_arm.stl` *(add your file)*

---

## ⚡ Electronics Assembly

### Power Distribution
- **Main Battery:** 2S Li-ion (7.4V, 2200mAh recommended)
- **Issue:** Initial setup had voltage drop under load
  - **Solution:** Added capacitors (470µF) near motor drivers
  - Used thicker gauge wires (18 AWG) for motor power

### Motor Drivers
- **Used:** 2x TB6612FNG modules
- You could use L298N easier wiring. You dont have to solder and comes with voltage redulator but lacks in efficeintcy.

### LED Wiring
- **Critical Issue:** Added 220Ω resistor in NeoPixel power rail thinking it was needed
  - **Problem:** Caused voltage drop affecting blue channel brightness
  - **Solution:** Removed resistor - NeoPixels work best with direct 5V connection
  - **Lesson:** Always check component datasheets before adding "protective" components
  - Then here i am using a NeoPixel ring with 8 leds as it was the cheapest option availiable. 

### Servo Integration
- **Flapper Servo:** Connected to Pin 18
- **Power:** Direct from battery through buck module.
- **Issue:** Servo buzzing at idle
  - **Solution:** Adjusted duty cycle calculation for 0° position
  - Used 16-bit LEDC resolution for smoother control
  - Add caps next to the servo for smoothening the current requirement.
- The flapper wasn't working when I tried using ESPServo libary. There were conflicts assigning the 4 motors and servo to the first 0-7 channels. And the pwm frequecy was set for the motors and servo didn't work. Even after manually forcing servo into second pwn set. Finally made it to work by ditching the libary and writing the servo control manually.

---

## 🎨 Aesthetics & Finishing

### LED Brightness Tuning
- **Initial:** Full brightness (255 RGB values)
- **Problem:** Way too bright, uncomfortable to look at
- **Solution:** Reduced to 20% brightness (51 RGB values)
- **Result:** Easier on eyes while maintaining visibility

### REMOTE
- I have attached the Liligo TDisplay to a 170 whole breadboard which is attached to the 840 pin breadboard using double sided tape. 
- The battery is atttached and stuck to the bigger breadboard using double sided tape. 
- Used single stand wire for connecting the switches and power lines. 
- **The Vcc and Ground pin of joystich is flipped intentionally** As otherwise the controls won't work properly.
- Similarl to battery the joystick is attached using formy double sided tape. 

---

## 📸 Build Photos

### Assembly Stages

#### 1. Frame Assembly
*(Add photo: `photos/01_frame_assembly.jpg`)*
- Laser-cut panels before sanding
- Assembled frame with motors mounted

#### 2. Electronics Layout
*(Add photo: `photos/02_electronics_layout.jpg`)*
- Motor drivers positioned
- ESP32 mounted
- Wiring harness layout

#### 3. LED Installation
*(Add photo: `photos/03_led_installation.jpg`)*
- NeoPixel strip placement
- Wiring connections

#### 4. Flapper Mechanism
*(Add photo: `photos/04_flapper_mechanism.jpg`)*
- 3D printed flapper arm
- Servo mounting
- Range of motion demonstration

#### 5. Completed Build - Top View
*(Add photo: `photos/05_completed_top.jpg`)*
- Fully assembled robot
- All components in place

#### 6. Completed Build - Side View
*(Add photo: `photos/06_completed_side.jpg`)*
- Battery placement
- Ground clearance

#### 7. Remote Control
*(Add photo: `photos/07_remote_control.jpg`)*
- TTGO T-Display with joystick
- Button layout

#### 8. Action Shots
*(Add photo: `photos/08_robot_action.jpg`)*
- Robot in motion
- LED indicators active
- Flapper mechanism deployed

---

## 🐛 Issues Encountered & Solutions

### Hardware Issues

#### 1. Motor Alignment
**Problem:** Mecanum wheels not driving straight
- **Cause:** Uneven motor mounting, wheels not perfectly perpendicular
- **Solution:** 
  - Adjusted motor mounts with washers for leveling
  - Used a level tool during assembly
  - Software speed calibration in code

#### 2. Wheel Wobble
**Problem:** Visible wobble in wheels during rotation
- **Cause:** Loose coupler fit, bent motor shafts
- **Solution:**
  - Redesigned couplers with tighter tolerances
  - Checked motor shaft straightness before installation

### Software Issues

#### 3. LED Flickering (Flapper Indicator)
**Problem:** LED status not staying on during flapper activation
- **Cause:** Status update function overwriting LED every loop
- **Solution:** 
  - Added state protection in `setLedStatus()`
  - Continuous LED state refresh in `flapper()` function
  - Separate LED control during kick/cooldown states

#### 4. Color Synchronization
**Problem:** Car LED colors didn't match remote display
- **Cause:** Different mode-to-color mappings between devices
- **Solution:** Standardized color scheme across both programs

#### 5. Blue Channel Dimness
**Problem:** Blue LEDs appeared significantly dimmer than red/green
- **Initially thought:** LED characteristic issue
- **Actual cause:** 220Ω resistor in power rail causing voltage drop
- **Solution:** Removed resistor, blue channel returned to normal

### Communication Issues

#### 6. ESP-NOW Connection Drops
**Problem:** Intermittent disconnections during operation
- **Cause:** WiFi interference, power supply instability
- **Solution:**
  - Disabled WiFi sleep mode: `WiFi.setSleep(false)`
  - Added watchdog timer for system recovery
  - Implemented proper timeout handling (500ms)

#### 7. Joystick Drift
**Problem:** Car slowly drifting even with joystick centered
- **Cause:** Analog joystick calibration, electrical noise
- **Solution:**
  - Implemented deadzone (±200 ADC counts) in `convertJoystickValues()`
  - Added capacitors to joystick power lines
  - Software filtering for stability

---

## 💡 Recommendations & Tips

### For Future Builders

#### 1. **Buy a Kit Instead of Individual Parts**
   - **Cost:** Often 30-50% cheaper than sourcing separately
   - **Time:** Saves weeks of waiting for parts from different suppliers
   - **Quality:** Parts designed to work together
   - **Less headache:** No compatibility issues

#### 2. **3D Printing Tips**
   - Print couplers with high infill (80%+) for strength
   - Use PETG for impact parts (flapper)
   - Print orientation matters - consider layer strength
   - Keep spare printed parts for quick replacement

#### 3. **Electronics Assembly**
   - **Test components individually** before final assembly
   - Use JST connectors for easy disconnect/reconnect
   - If not available use berge male and female connectors. But keep current limit inconsiderations.
   - Label all wires (seriously, do this! saves lot of time debugging. I used nail polish to indicate diffrent motor and orientation)
   - Take photos during assembly for reference

#### 4. **Power Management**
   - Use proper capacity battery (2200mAh minimum) from good brand it does make a difference.
   - Add voltage monitoring early in development
   - Include power switch in accessible location
   - Consider adding a resettable fuse for protection. Saves money down the line.

#### 5. **LED Brightness**
   - Start with low brightness (20-30%) and increase if needed
   - Much easier to increase than decrease
   - Consider ambient lighting conditions

#### 6. **Software Development**
   - Use version control (Git) from the start
   - Comment your code thoroughly
   - Test each feature independently
   - Keep a changelog of modifications

---

## 🔮 Future Enhancements

### Planned Upgrades
- [ ] Add IMU (MPU6050) for stability control
- [ ] Implement PID speed control for accurate movements
- [ ] Add ultrasonic sensors for obstacle detection
- [ ] Create autonomous navigation mode
- [ ] Add FPV camera module
- [ ] Implement mobile app control (Bluetooth/WiFi)
- [ ] Add line-following capability
- [ ] Improve battery life optimization

### Ideas Under Consideration
- RGB underglow lighting
- Additional servo for grabber arm
- Encoder-based odometry
- Custom PCB to consolidate wiring

---

## 📚 Resources & References

### Original Tutorial
- **DroneBot Workshop:** https://dronebotworkshop.com
- YouTube tutorial series on mecanum wheel robots
- Excellent explanations of mecanum wheel mathematics

### Libraries Used
- **NeoPixelBus** by Michael C. Miller: https://github.com/Makuna/NeoPixelBus
- **TFT_eSPI** by Bodmer: https://github.com/Bodmer/TFT_eSPI
- **ESP-NOW** documentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html

### Helpful Resources
- ESP32 pinout reference
- Mecanum wheel kinematics tutorials
- PlatformIO documentation
- WS2812B datasheet

---

## 📞 Contact & Contributions

This project is open source and contributions are welcome!

**Found an issue?** Open an issue on GitHub
**Have improvements?** Submit a pull request
**Questions?** Start a discussion

---

## 🎉 Conclusion

This project has been an incredible learning experience, combining mechanical design, electronics, and embedded programming. While challenging at times (especially the sanding!), the end result is a highly capable and fun robot platform.

**Key Takeaways:**
1. **Buy a kit if possible** - saves time and money
2. **Test early and often** - catch issues before final assembly
3. **Document everything** - future you will thank present you
4. **Community resources are invaluable** - thank you DroneBot Workshop!
5. **Iterate and improve** - first version doesn't have to be perfect

**Total Project Rating:** ⭐⭐⭐⭐⭐ (5/5)
- Fun factor: ⭐⭐⭐⭐⭐
- Learning value: ⭐⭐⭐⭐⭐
- Difficulty: ⭐⭐⭐⭐ (4/5)
- Cost effectiveness: ⭐⭐⭐ (3/5 - kit would be better)

**Would I build it again?** Absolutely! But with a kit next time. 😄

---

*Last Updated: January 9, 2026*
*Build by: Sanjay Sajeev*
