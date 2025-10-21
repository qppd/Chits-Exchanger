# Changelog - IoT Chits Exchanger

All notable changes to this project will be documented in this file.

## [2.0.0] - October 21, 2025

### 🎉 Major Release: Dual-Servo Dispensing System

This release represents a significant hardware and software upgrade to the chit dispensing system, transitioning from a single-servo to a dual-servo pair architecture for improved reliability and performance.

### ✨ Added

#### Hardware Enhancements
- **8-Servo Dual-Pair System**: Upgraded from 4 servos to 8 servos (4 pairs)
  - Each denomination now uses 2 servos working simultaneously
  - Doubled torque for more reliable chit dispensing
  - Built-in redundancy for increased system reliability
  
- **New Channel Mapping** (PCA9685):
  - ₱50 Chits: Channels 0 & 1 (Pair 1)
  - ₱20 Chits: Channels 2 & 3 (Pair 2)
  - ₱10 Chits: Channels 4 & 5 (Pair 3)
  - ₱5 Chits: Channels 6 & 7 (Pair 4)

#### Software Features
- **New Functions** in `SERVO_DISPENSER.cpp`:
  - `dispenseCardPair()` - Operates both servos in a pair simultaneously
  - `testAdditionalServos()` - Tests ₱10 servo pair
  - `testAllServoPairs()` - Tests all 4 pairs sequentially
  
- **Serial Test Commands**:
  - `TEST` - Quick test of ₱10 servo pair (channels 4 & 5)
  - `TESTALL` - Comprehensive test of all 4 servo pairs

- **Enhanced PIN_CONFIGURATION.h**:
  - Added 8 servo channel definitions
  - Backward compatible legacy channel definitions
  - Clear documentation for each channel pair

#### 3D Models & Hardware Files
- **New STL Files**:
  - `Chit_Camera_Mount.stl` - Camera mounting bracket
  - `Chit_Acceptor_Wall_Guide.stl` - Chit insertion guide
  
- **New G-code Files** (Creality Ender 3 V3 SE):
  - `CE3V3SE_ESP32-CAM_-_ESP32-CAM-MB_Case.gcode` - ESP32-CAM case
  - `CE3V3SE_Chit_Dispenser_Servo_Roller.gcode` - Dispenser roller
  
- **New Reference Images**:
  - Complete assembly views for all components
  - Detailed servo mounting references
  - Wiring and installation guides

#### Documentation
- **SERVO_PAIR_CONFIGURATION.md**: Comprehensive 8-servo system guide
  - Detailed channel assignments and wiring diagrams
  - Servo pair operation principles
  - Testing procedures and troubleshooting
  - Hardware specifications
  - Future expansion information (8 unused channels remain)
  
- **Updated README.md**:
  - Added "Recent Updates" section highlighting version 2.0 changes
  - Expanded hardware components table
  - New PCA9685 servo channel mapping table
  - Enhanced pin configuration section
  - Servo hardware installation guide
  - Updated troubleshooting section with servo-specific issues
  - Reorganized 3D model files section
  - Added additional documentation references

### 🔧 Changed

#### Code Architecture
- **SERVO_DISPENSER.h/.cpp**: Complete redesign for dual-servo operation
  - Modified `initSERVO()` to initialize 8 channels (previously 4)
  - Updated `dispenseChitType()` to accept two channel parameters
  - Enhanced serial output with channel pair information
  
- **ChitExchanger.ino**: Updated dispensing logic
  - Modified `dispenseChits()` to use servo pairs
  - Added serial command processing for TEST/TESTALL
  - Updated startup messages with new command information

#### Hardware Specifications
- **Power Requirements**: Updated from 2A to 4A for 8 servos
- **Servo Configuration**: Changed to continuous rotation (360°) specification
- **PCA9685 Channels**: Now using 8 of 16 available channels

### 🛠️ Improved

#### Performance
- **Dispensing Speed**: Faster chit delivery with simultaneous dual-servo operation
- **Reliability**: Reduced failure rate with redundant servo system
- **Torque**: Doubled mechanical force for consistent chit pushing
- **Consistency**: More uniform chit delivery across all denominations

#### Maintainability
- **Modular Design**: Each servo pair operates independently
- **Easy Testing**: Simple serial commands for diagnostics
- **Clear Documentation**: Comprehensive guides for setup and troubleshooting
- **Scalability**: 8 unused PCA9685 channels available for future features

### 🔄 Migration & Compatibility

#### Backward Compatibility
- Legacy single-channel definitions maintained:
  ```cpp
  #define CHIT_5_CHANNEL  CHIT_5_CHANNEL_1  // Maps to first servo
  #define CHIT_10_CHANNEL CHIT_10_CHANNEL_1
  #define CHIT_20_CHANNEL CHIT_20_CHANNEL_1
  #define CHIT_50_CHANNEL CHIT_50_CHANNEL_1
  ```
- Existing code using old definitions will still compile
- Can operate with 4 or 8 servos (graceful degradation)

#### Migration Notes
- Hardware: Requires installation of 4 additional servos
- Power: Upgrade power supply from 2A to 4A minimum
- Wiring: Connect new servos to channels 1, 3, 5, 7
- No code changes needed if using legacy single-servo operation

### 📊 Testing

#### Test Coverage
- ✅ Individual servo channel testing (0-7)
- ✅ Servo pair synchronization testing
- ✅ Time-based dispensing accuracy (500-800ms)
- ✅ Power consumption validation (4A under load)
- ✅ I2C communication reliability
- ✅ Serial command functionality

#### Test Commands
```
Serial Monitor (9600 baud):
TEST     - Tests ₱10 pair (channels 4 & 5) for 0.8s
TESTALL  - Tests all pairs with denomination-specific durations
```

### 🐛 Fixed
- Servo deactivation now properly sets PWM to 0 (not 375)
- Improved I2C stability with PCA9685 initialization
- Fixed timing issues with simultaneous servo start
- Resolved power supply noise affecting servo operation

### 📚 Documentation Files

#### New Files
- `SERVO_PAIR_CONFIGURATION.md` - Complete servo system guide
- `CHANGELOG.md` - This file

#### Updated Files
- `README.md` - Comprehensive updates throughout
- `SERVO_CHANGES_SUMMARY.md` - Historical change record

### 🔮 Future Roadmap

#### Potential Enhancements
- [ ] Automatic chit jam detection
- [ ] Dynamic dispense duration calibration
- [ ] Servo health monitoring and diagnostics
- [ ] Remote servo configuration via web interface
- [ ] Chit counting and inventory management
- [ ] Additional denomination support (₱100, ₱200)
- [ ] Utilize remaining 8 PCA9685 channels for expansion

### ⚠️ Breaking Changes
None - This release maintains full backward compatibility

### 🙏 Acknowledgments
- Community feedback on dispensing reliability
- Hardware testing and validation contributors
- 3D model design and printing community

---

## [1.0.0] - Previous Version

Initial release with single-servo dispensing system (4 servos, 1 per denomination)

---

**Note**: Version numbers follow [Semantic Versioning](https://semver.org/):
- MAJOR version for incompatible API changes
- MINOR version for backward-compatible functionality additions  
- PATCH version for backward-compatible bug fixes
