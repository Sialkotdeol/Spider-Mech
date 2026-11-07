# Servo signal wiring

The hexapod uses two PCA9685 servo boards. The **left legs go on board 1** and the **right legs go on board 2**.

This file matches `SERVO_CHANNELS` in [backend/config.py](backend/config.py). If you change one, change the other.

## Boards

| Board | I2C address | Address jumpers | Legs |
|---|---|---|---|
| Board 1 | `0x40` | none soldered | L1, L2, L3 |
| Board 2 | `0x41` | **A0** soldered | R1, R2, R3 |

## Pin map

| Leg | Joint | Board | Pin | Channel in software |
|---|---|---|---|---|
| **L1** | coxa | 1 (`0x40`) | 0 | 0 |
| L1 | femur | 1 (`0x40`) | 1 | 1 |
| L1 | tibia | 1 (`0x40`) | 2 | 2 |
| **L2** | coxa | 1 (`0x40`) | 3 | 3 |
| L2 | femur | 1 (`0x40`) | 4 | 4 |
| L2 | tibia | 1 (`0x40`) | 5 | 5 |
| **L3** | coxa | 1 (`0x40`) | 6 | 6 |
| L3 | femur | 1 (`0x40`) | 7 | 7 |
| L3 | tibia | 1 (`0x40`) | 8 | 8 |
| **R1** | coxa | 2 (`0x41`) | 0 | 16 |
| R1 | femur | 2 (`0x41`) | 1 | 17 |
| R1 | tibia | 2 (`0x41`) | 2 | 18 |
| **R2** | coxa | 2 (`0x41`) | 3 | 19 |
| R2 | femur | 2 (`0x41`) | 4 | 20 |
| R2 | tibia | 2 (`0x41`) | 5 | 21 |
| **R3** | coxa | 2 (`0x41`) | 6 | 22 |
| R3 | femur | 2 (`0x41`) | 7 | 23 |
| R3 | tibia | 2 (`0x41`) | 8 | 24 |

Pins 9–15 on both boards are free.

To work out the software channel: board 1 pin *N* is channel *N*, and board 2 pin *N* is channel *16 + N*.

```
          FRONT
   L1  ●─────────●  R1
       │         │
   L2  ●  BODY   ●  R2
       │         │
   L3  ●─────────●  R3
          BACK

 Board 1 (0x40): L1 = pins 0-2,  L2 = pins 3-5,  L3 = pins 6-8
 Board 2 (0x41): R1 = pins 0-2,  R2 = pins 3-5,  R3 = pins 6-8
 On every leg:   coxa, femur, tibia = 1st, 2nd, 3rd pin
```

## Which joint is which

| Joint | Where it is | How it moves |
|---|---|---|
| Coxa | hip, at the body | swings the leg forward and back (shaft points up/down) |
| Femur | thigh | lifts the leg up and down (shaft points sideways) |
| Tibia | knee | bends the lower leg in and out |

## Plug orientation

Each pin column on the board is **GND – V+ – PWM**, from the board edge inward.

| Servo wire | Board row |
|---|---|
| Brown / black | **GND** |
| Red | **V+** |
| Orange / yellow (signal) | **PWM** |

Plug every servo in the same way round. If a plug is reversed, that servo won't work, and it may be damaged.

## Power and the Pi connection

**Both boards** need all of these:

| Board terminal | Connect to |
|---|---|
| Green screw terminal **V+** | servo battery + |
| Green screw terminal **GND** | servo battery − |
| **VCC** pin | Pi **3.3 V** (pin 1). Never the battery. |
| **GND** pin | Pi **GND** (pin 6) |
| **SDA** | Pi **GPIO 2 / SDA** (pin 3) |
| **SCL** | Pi **GPIO 3 / SCL** (pin 5) |

- The Pi, both boards and the battery **must share a common ground**. Without it, servos twitch, stall, or go limp.
- Board 2 needs its own V+ and GND wires from the battery. Its servos won't get power from board 1.
- Each servo's power comes from V+, and its control comes from PWM. If a servo snaps to 90° at power-on and then goes limp, its **signal wire** isn't connected properly, even if V+ measures correctly.

## Checking the wiring

**Check that both boards are at 50 Hz.** On the Pi:
```bash
/usr/sbin/i2cget -y 1 0x40 0xFE    # board 1, should print 0x79
/usr/sbin/i2cget -y 1 0x41 0xFE    # board 2, should print 0x79
```
`0x1e` means the board is at about 200 Hz, and its servos won't follow commands. Restart the server and the driver will set 50 Hz again.

**Move one servo without the server.** Stop uvicorn first, then on the Pi:
```bash
cd ~/Desktop/spider_mech
.venv/bin/python servo_test.py 0x40 1    # board 1 pin 1 = L1 femur
.venv/bin/python servo_test.py 0x41 4    # board 2 pin 4 = R2 femur
```
The servo moves 60° ↔ 120° three times, then goes back to 90° and is released.

**Move one joint through the server** (from the Mac):
```bash
curl -X POST http://192.168.68.107:8000/api/leg/L1/joints -H 'Content-Type: application/json' -d '{"coxa":0,"femur":30,"tibia":109.5}'
curl -X DELETE http://192.168.68.107:8000/api/leg/L1/override    # hand the leg back
```
