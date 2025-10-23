# Wiring Diagram - Chits to Coin Exchanger

## Complete System Wiring

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                            RASPBERRY PI 4                                   │
│                                                                             │
│  ┌──────────┬───────────────────────────────────────────────────┐         │
│  │   PIN    │   CONNECTION                                       │         │
│  ├──────────┼───────────────────────────────────────────────────┤         │
│  │ GPIO 2   │ ──────→ LCD SDA (I2C Data)                        │         │
│  │ GPIO 3   │ ──────→ LCD SCL (I2C Clock)                       │         │
│  │ GPIO 17  │ ──────→ IR Sensor OUT                             │         │
│  │ GPIO 22  │ ──────→ Servo Signal Wire (Orange/Yellow)         │         │
│  │ 5V       │ ──────→ Power Rail (+5V)                          │         │
│  │ GND      │ ──────→ Ground Rail (GND)                         │         │
│  │ USB      │ ──────→ ESP32 (Serial Communication)              │         │
│  │ Ethernet │ ──────→ Network (for ESP32-CAM access)            │         │
│  └──────────┴───────────────────────────────────────────────────┘         │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                               ESP32                                         │
│                         (Coin Exchanger)                                    │
│                                                                             │
│  ┌──────────┬───────────────────────────────────────────────────┐         │
│  │   PIN    │   CONNECTION                                       │         │
│  ├──────────┼───────────────────────────────────────────────────┤         │
│  │ GPIO 21  │ ──────→ LCD SDA (I2C Data)                        │         │
│  │ GPIO 22  │ ──────→ LCD SCL (I2C Clock)                       │         │
│  │ GPIO 27  │ ──────→ Button (other side to GND)                │         │
│  │          │                                                    │         │
│  │ GPIO 19  │ ──────→ Hopper 1 Pulse Out (5 PHP)                │         │
│  │ GPIO 26  │ ──────→ SSR 1 Input + (Hopper 1 Power)            │         │
│  │          │                                                    │         │
│  │ GPIO 18  │ ──────→ Hopper 2 Pulse Out (10 PHP)               │         │
│  │ GPIO 25  │ ──────→ SSR 2 Input + (Hopper 2 Power)            │         │
│  │          │                                                    │         │
│  │ GPIO 4   │ ──────→ Hopper 3 Pulse Out (20 PHP)               │         │
│  │ GPIO 33  │ ──────→ SSR 3 Input + (Hopper 3 Power)            │         │
│  │          │                                                    │         │
│  │ TX/RX    │ ──────→ RPi USB (Serial)                          │         │
│  │ 5V       │ ──────→ Power Supply (+5V)                        │         │
│  │ GND      │ ──────→ All GND connections                       │         │
│  └──────────┴───────────────────────────────────────────────────┘         │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                            ESP32-CAM                                        │
│                          (IP Camera)                                        │
│                                                                             │
│  Standard ESP32-CAM Pinout                                                 │
│  - Power: 5V, GND                                                          │
│  - Camera module connected to designated pins                              │
│  - WiFi antenna connected                                                  │
│  - Connects to network via WiFi                                            │
│  - Accessed by RPi at http://<camera_ip>/stream                           │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Detailed Component Wiring

### 1. IR Sensor Module
```
┌─────────────────┐
│   IR SENSOR     │
├─────────────────┤
│ VCC ──────→ 5V  │ ← Raspberry Pi 5V
│ GND ──────→ GND │ ← Raspberry Pi GND
│ OUT ──────→ G17 │ ← Raspberry Pi GPIO 17
└─────────────────┘

Notes:
- OUT is active LOW (0V when object detected)
- Adjust sensitivity potentiometer for detection distance
- Typical detection range: 2-30cm
```

### 2. Servo Motor
```
┌───────────────────┐
│   SERVO MOTOR     │
├───────────────────┤
│ Red   ──────→ 5V  │ ← Raspberry Pi 5V (or external 5V)
│ Brown ──────→ GND │ ← Raspberry Pi GND
│ Orange──────→ G22 │ ← Raspberry Pi GPIO 22 (PWM)
└───────────────────┘

Notes:
- Use external 5V power supply if servo draws >500mA
- Signal wire expects 3.3V PWM (Pi GPIO is 3.3V compatible)
- 50Hz PWM frequency
- Pulse width: 500-2500μs for 0-180°
```

### 3. I2C LCD Display (20x4)
```
┌─────────────────────────┐
│   LCD 20x4 (I2C)        │
├─────────────────────────┤
│ VCC ─────→ 5V           │ ← Power Rail (+5V)
│ GND ─────→ GND          │ ← Ground Rail
│ SDA ─────→ GPIO 2 (Pi)  │ ← Raspberry Pi GPIO 2 (SDA)
│           GPIO 21 (ESP) │ ← ESP32 GPIO 21 (SDA)
│ SCL ─────→ GPIO 3 (Pi)  │ ← Raspberry Pi GPIO 3 (SCL)
│           GPIO 22 (ESP) │ ← ESP32 GPIO 22 (SCL)
└─────────────────────────┘

Notes:
- I2C Address: 0x27 (verify with i2cdetect -y 1)
- Contrast adjustable via potentiometer on back
- Backlight always on
- Both RPi and ESP32 share the same LCD via I2C bus
  (Connect SDA and SCL to both devices in parallel)
```

### 4. Push Button
```
┌───────────────────┐
│    BUTTON         │
├───────────────────┤
│ Pin 1 ──────→ G27 │ ← ESP32 GPIO 27
│ Pin 2 ──────→ GND │ ← ESP32 GND
└───────────────────┘

Notes:
- Internal pull-up enabled in software
- Button press = LOW signal (0V)
- Button release = HIGH signal (3.3V)
- Debounce handled in software (200ms)
```

### 5. Coin Hopper + SSR (×3 sets)

#### Hopper 1 (5 PHP coins)
```
┌──────────────────────────────────────┐
│         SOLID STATE RELAY 1          │
├──────────────────────────────────────┤
│ Input:                               │
│   (+) ────→ ESP32 GPIO 26            │
│   (-) ────→ ESP32 GND                │
│                                      │
│ Load:                                │
│   (+) ────→ 12V Power Supply (+)     │
│   (-) ────→ Hopper 1 Power (+)       │
└──────────────────────────────────────┘

┌──────────────────────────────────────┐
│        ALLAN COIN HOPPER 1           │
│           (5 PHP coins)              │
├──────────────────────────────────────┤
│ Power (+) ───→ SSR 1 Load (-)        │
│ Power (-) ───→ 12V Supply (-)        │
│ Pulse Out ───→ ESP32 GPIO 19         │
│ GND ──────────→ ESP32 GND            │
└──────────────────────────────────────┘

Pulse Signal:
- 1 pulse = 1 coin dispensed
- Pulse width: ~50-100ms
- Voltage: 3.3V compatible
```

#### Hopper 2 (10 PHP coins)
```
Same wiring as Hopper 1, but:
- SSR 2 Input (+) ──→ ESP32 GPIO 25
- Hopper 2 Pulse ───→ ESP32 GPIO 18
```

#### Hopper 3 (20 PHP coins)
```
Same wiring as Hopper 1, but:
- SSR 3 Input (+) ──→ ESP32 GPIO 33
- Hopper 3 Pulse ───→ ESP32 GPIO 4
```

### 6. Power Supply Wiring

```
┌────────────────────────────────────────┐
│         12V POWER SUPPLY               │
│         (for Coin Hoppers)             │
├────────────────────────────────────────┤
│ (+12V) ──┬──→ SSR 1 Load (+)           │
│          ├──→ SSR 2 Load (+)           │
│          └──→ SSR 3 Load (+)           │
│                                        │
│ (GND)  ──┬──→ Hopper 1 (-)             │
│          ├──→ Hopper 2 (-)             │
│          └──→ Hopper 3 (-)             │
└────────────────────────────────────────┘

┌────────────────────────────────────────┐
│         5V POWER SUPPLY                │
│         (for Logic)                    │
├────────────────────────────────────────┤
│ (+5V)  ──┬──→ Raspberry Pi             │
│          ├──→ ESP32                    │
│          ├──→ ESP32-CAM                │
│          ├──→ LCD VCC                  │
│          ├──→ IR Sensor VCC            │
│          └──→ Servo VCC (if needed)    │
│                                        │
│ (GND)  ──→ Common Ground (all devices) │
└────────────────────────────────────────┘

Notes:
- Use adequate current ratings:
  * 12V supply: 3A minimum (1A per hopper)
  * 5V supply: 5A minimum (RPi + ESP32 + peripherals)
- Fuse all power lines
- Use separate grounds, connect at power supply
```

## Connection Priority

### Phase 1: Test Setup (Breadboard)
1. Connect ESP32 to USB power
2. Connect LCD to ESP32 (I2C)
3. Connect button to ESP32
4. Upload CoinExchanger.ino
5. Test basic functionality

### Phase 2: Add Coin Hoppers
1. Connect SSR inputs to ESP32
2. Connect 12V power supply to SSRs
3. Connect hoppers to SSRs and ESP32 pulse pins
4. Test with test_relay and test_pulse commands

### Phase 3: Add Raspberry Pi Components
1. Connect IR sensor to RPi
2. Connect servo to RPi
3. Connect USB serial cable RPi ↔ ESP32
4. Test communication

### Phase 4: Add Camera
1. Setup ESP32-CAM with WiFi
2. Test video stream
3. Connect to same network as RPi

## Safety Checklist

✓ All grounds connected to common ground
✓ 12V and 5V supplies isolated
✓ SSRs rated for hopper current
✓ Fuses on all power lines
✓ No crossed connections (12V to 5V pins)
✓ Polarity correct on all components
✓ Servo powered adequately (external supply if needed)
✓ ESP32 powered before connecting peripherals
✓ I2C pull-up resistors present (usually on LCD module)

## Troubleshooting Connection Issues

### LCD not responding
1. Check I2C address: `i2cdetect -y 1` on RPi
2. Verify SDA/SCL not swapped
3. Check 5V and GND connections
4. Adjust contrast potentiometer

### IR sensor not detecting
1. Check 5V power
2. Adjust sensitivity potentiometer
3. Check OUT pin connection to GPIO 17
4. Test with ir_sensor_tester.py

### Servo not moving
1. Verify power supply (5V, adequate current)
2. Check signal wire to GPIO 22
3. Ensure pigpiod is running
4. Test pulse width with oscilloscope

### Hopper not dispensing
1. Check 12V power supply
2. Verify SSR LED lights up when activated
3. Check SSR load connections
4. Test hopper manually with external 12V
5. Check pulse wire connection

### Serial communication failed
1. Check USB cable connection
2. Verify correct port (/dev/ttyUSB0)
3. Check baud rate (115200)
4. Try different USB port
5. Check TX/RX not crossed (USB handles this)

## Pin Summary Tables

### Raspberry Pi GPIO Usage
| GPIO | Function | Direction | Connected To |
|------|----------|-----------|--------------|
| 2    | I2C SDA  | Bidirectional | LCD SDA |
| 3    | I2C SCL  | Output | LCD SCL |
| 17   | IR Sensor | Input | IR Sensor OUT |
| 22   | Servo PWM | Output | Servo Signal |

### ESP32 GPIO Usage
| GPIO | Function | Direction | Connected To |
|------|----------|-----------|--------------|
| 21   | I2C SDA  | Bidirectional | LCD SDA |
| 22   | I2C SCL  | Output | LCD SCL |
| 27   | Button   | Input (Pull-up) | Button |
| 19   | Pulse 1  | Input | Hopper 1 Pulse |
| 26   | SSR 1    | Output | SSR 1 Input |
| 18   | Pulse 2  | Input | Hopper 2 Pulse |
| 25   | SSR 2    | Output | SSR 2 Input |
| 4    | Pulse 3  | Input | Hopper 3 Pulse |
| 33   | SSR 3    | Output | SSR 3 Input |

---

**Always double-check connections before powering on!**
**Use a multimeter to verify voltages and continuity.**
**Test components individually before full integration.**
