# PS5 controller

The DualSense pairs with the **Pi** over Bluetooth, and the server reads it directly ([backend/gamepad.py](backend/gamepad.py)). The laptop UI keeps working at the same time; whichever one moved last is in control. A centred controller sends nothing, so it never fights the laptop.

## One-time setup on the Pi

```
sudo apt install -y python3-evdev bluetooth bluez
```

If uvicorn runs in a virtualenv, install it there instead: `pip install evdev`.

## Pairing (once per controller)

1. Turn on pairing mode: hold **Create** + **PS** until the light bar flashes quickly.
2. On the Pi:
   ```
   bluetoothctl
   power on
   agent on
   default-agent
   scan on
   ```
3. Wait for `DualSense Wireless Controller` and note its address (`XX:XX:...`), then:
   ```
   scan off
   pair XX:XX:XX:XX:XX:XX
   trust XX:XX:XX:XX:XX:XX
   connect XX:XX:XX:XX:XX:XX
   exit
   ```

After that, pressing **PS** reconnects it. It can join before or after uvicorn starts; the uvicorn log prints `controller connected`.

## Controls

| Input | Does |
|---|---|
| Left stick | walk forward/back, strafe |
| Right stick left/right | turn |
| Hold **L1** + right stick | move the body (current axis mode) |
| **R1** | next axis mode: X-Y → Z-W → U-V → XY-UV |
| D-pad left/right | previous / next gait |
| D-pad up/down | speed ±10 % |
| Cross / Circle | stand / sit |
| Triangle / Square | tall / crouch |
| **R2** / **L2** | jump / bounce |
| L3 / R3 (press sticks) | flex / dive |
| **PS button** | **E-STOP** |
| Options | clear E-stop |
| Create | beep |

If the controller disconnects (out of range or battery dead), the robot stops walking.

## Troubleshooting

- Check the Pi sees it: `ls /dev/input/` should list new `event*` files when it connects. `python3 -m evdev.evtest` shows live events.
- `Permission denied` on `/dev/input/event*`: add the user to the input group: `sudo usermod -aG input pi`, then log out and back in.
- Sticks drift: raise `GAMEPAD_DEADZONE` in [backend/config.py](backend/config.py).
- To turn it off: `GAMEPAD_ENABLED = False` in config.py.
