# Serial Communication Protocol

## System Architecture

```
┌─────────────────────────────────────────┐
│   Raspberry Pi (MASTER)                 │
│   - yolo_detect.py                      │
│   - IR Sensor monitoring                │
│   - YOLO chit detection                 │
│   - Servo chit dispenser                │
│   - LCD display                         │
└──────────────┬──────────────────────────┘
               │
               │ Serial (115200 baud)
               │ /dev/ttyUSB0
               │
┌──────────────▼──────────────────────────┐
│   ESP32 (SLAVE)                         │
│   - CoinExchanger.ino                   │
│   - 3x Coin Hoppers (5, 10, 20 PHP)     │
│   - Coin dispensing logic               │
│   - LCD display                         │
└─────────────────────────────────────────┘
```

## Communication Flow

### Initialization

1. **ESP32** starts up and sends:
   - `SLAVE_READY` - Announces it's ready to receive commands

2. **RPi** starts up and begins monitoring IR sensor

### Normal Operation Flow

1. **RPi** monitors IR sensor continuously
2. When chit inserted:
   - **RPi** sends: `IR_DETECTED`
3. **RPi** runs YOLO detection
4. When chit identified:
   - **RPi** sends: `CHIT_DETECTED:50` (example: 50 peso chit)
5. **RPi** releases chit via servo
   - **RPi** sends: `CHIT_RELEASED`
6. **RPi** triggers auto-dispensing
   - **RPi** sends: `AUTO_DISPENSE:50`
7. **ESP32** calculates coin combination and dispenses
8. When complete:
   - **ESP32** sends: `DISPENSING_COMPLETE`
9. **RPi** updates LCD to "Ready" state

### Error Handling

- If detection times out:
  - **RPi** sends: `DETECTION_TIMEOUT`
  - **ESP32** resets to idle state

- If ESP32 is busy:
  - **ESP32** sends: `ERROR:SYSTEM_BUSY`
  - **RPi** waits and retries

- If invalid value received:
  - **ESP32** sends: `ERROR:INVALID_VALUE`

### Shutdown

- When shutting down:
  - **RPi** sends: `SYSTEM_SHUTDOWN`
  - **ESP32** acknowledges and enters standby mode

## Command Reference

### Commands FROM RPi TO ESP32 (Master → Slave)

| Command | Format | Description | ESP32 Action |
|---------|--------|-------------|--------------|
| `IR_DETECTED` | `IR_DETECTED` | IR sensor detected chit | Informational only |
| `CHIT_DETECTED` | `CHIT_DETECTED:50` | Chit value identified | Triggers calculation and dispensing |
| `AUTO_DISPENSE` | `AUTO_DISPENSE:50` | Auto-dispense command | Immediately dispenses coins for value |
| `CHIT_RELEASED` | `CHIT_RELEASED` | Servo released chit | Informational only |
| `DETECTION_TIMEOUT` | `DETECTION_TIMEOUT` | Detection failed | Resets ESP32 to idle |
| `SYSTEM_SHUTDOWN` | `SYSTEM_SHUTDOWN` | RPi shutting down | ESP32 enters standby |

### Responses FROM ESP32 TO RPi (Slave → Master)

| Response | Format | Description | When Sent |
|----------|--------|-------------|-----------|
| `SLAVE_READY` | `SLAVE_READY` | ESP32 is ready | At startup |
| `DISPENSING_COMPLETE` | `DISPENSING_COMPLETE` | Coins dispensed successfully | After dispensing completes |
| `ERROR:SYSTEM_BUSY` | `ERROR:SYSTEM_BUSY` | ESP32 busy, cannot accept command | When already dispensing |
| `ERROR:INVALID_VALUE` | `ERROR:INVALID_VALUE` | Invalid chit value received | When value not 5/10/20/50 |

## Valid Chit Values

- `5` - 5 peso chit
- `10` - 10 peso chit
- `20` - 20 peso chit
- `50` - 50 peso chit

## Serial Port Settings

- **Baud Rate**: 115200
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Flow Control**: None (DTR/RTS disabled to prevent auto-reset)
- **Terminator**: Newline (`\n`)

## Example Session

```
[ESP32 startup]
ESP32 → RPi: SLAVE_READY

[User inserts 50 peso chit]
RPi → ESP32: IR_DETECTED
[RPi runs YOLO detection for 3-5 seconds]
RPi → ESP32: CHIT_DETECTED:50
[RPi moves servo to release chit]
RPi → ESP32: CHIT_RELEASED
[RPi triggers auto-dispensing]
RPi → ESP32: AUTO_DISPENSE:50

[ESP32 calculates: 2x20PHP + 1x10PHP]
[ESP32 dispenses coins from hoppers]
[After ~10-15 seconds]
ESP32 → RPi: DISPENSING_COMPLETE

[System ready for next transaction]
```

## Timing Considerations

- **IR Detection**: Continuous monitoring, ~10ms polling
- **YOLO Detection**: 3-5 seconds per chit
- **Servo Release**: 2 seconds movement
- **Auto-Dispense**: 10-15 seconds depending on coin count
- **Serial Timeout**: 1 second for writes, 10ms for reads (non-blocking)

## Error Recovery

1. **Communication timeout**: RPi continues operation, ESP32 waits
2. **Invalid command**: ESP32 logs error, ignores command
3. **Busy state**: ESP32 responds with ERROR, RPi waits and retries
4. **Detection failure**: RPi sends DETECTION_TIMEOUT, both reset

## Notes

- **RPi is MASTER**: Initiates all operations based on sensor input
- **ESP32 is SLAVE**: Responds to commands, executes dispensing
- **Non-blocking**: RPi uses non-blocking serial reads for responsiveness
- **One transaction at a time**: ESP32 rejects commands if busy
- **Auto-dispensing**: Default behavior, no user selection required
