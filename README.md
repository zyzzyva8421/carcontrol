# Raspberry Car SSH Control Tool

A minimal command-line tool to control a Raspberry Pi car by sending commands over SSH.

## Requirements

- Linux or macOS terminal
- Python 3.10+
- SSH access to the car (`pi@192.168.2.21`)
- A command on the Raspberry Pi that accepts actions like `forward`, `left`, `stop`, etc.

## Quick Start

From this project folder:

```bash
python3 car_control.py --help
```

### 1) Test SSH command generation (safe)

```bash
python3 car_control.py \
  --host 192.168.2.21 \
  --user pi \
  --remote-template 'python /home/pi/car_action.py {action}' \
  --dry-run \
  action forward
```

### 2) Send one command

```bash
python3 car_control.py \
  --host 192.168.2.21 \
  --user pi \
  --remote-template 'python /home/pi/car_action.py {action}' \
  action forward
```

Then stop:

```bash
python3 car_control.py \
  --host 192.168.2.21 \
  --user pi \
  --remote-template 'python /home/pi/car_action.py {action}' \
  action stop
```

### 3) Keyboard mode

```bash
python3 car_control.py \
  --host 192.168.2.21 \
  --user pi \
  --remote-template 'python /home/pi/car_action.py {action}' \
  --control-style hold \
  keyboard
```

Keyboard controls:

- `w`: hold to move forward
- `s`: hold to move backward
- `a`: hold to turn left
- `d`: hold to turn right
- `x`: stop
- `+`: speed up
- `-`: speed down
- `h`: horn
- `l`: lights
- `q`: quit (sends final stop)

Default style is `hold` (continuous move while key is pressed). Optional step mode:

```bash
python3 car_control.py keyboard --control-style step --step-seconds 0.25
```

### 4) Automatic obstacle avoidance mode

Copy the auto-avoid script to the Pi once:

```bash
scp auto_avoid.py pi@192.168.2.21:/home/pi/auto_avoid.py
ssh pi@192.168.2.21 'chmod +x /home/pi/auto_avoid.py'
```

Start auto-avoid mode:

```bash
python3 car_control.py auto --threshold-cm 30 --speed 45
```

Tune behavior:

- `--threshold-cm`: obstacle trigger distance (higher = more cautious)
- `--speed`: motor speed (20-100)
- `--loop-seconds`: control loop interval

Stop auto mode with `Ctrl+C`, then run:

```bash
python3 car_control.py action stop
```

### 5) Simple controller GUI

Start GUI:

```bash
python3 car_controller_gui.py
```

Controls in GUI:

- `Test Connection` checks SSH connectivity and reports status.
- `Control style`: choose `Hold` for continuous movement while pressed, or `Step` for short moves.
- In `Step` style, adjust step duration in the numeric box (seconds).
- In `Hold` style, the app sends one move command on press and `stop` on release for smoother motion.
- Default GUI template is `python /home/pi/car_action.py {action}`.
- Press and hold `Forward/Left/Right/Backward` buttons to move.
- Release movement button to auto-stop.
- `Stop`, `Horn`, `Lights`, `Speed +`, `Speed -` are one-click actions.
- `Start Auto` launches obstacle avoidance mode from the GUI.
- `Stop Auto` stops obstacle avoidance and sends a stop command.
- Keyboard shortcuts: `W/A/S/D` or arrow keys. `Space` = stop.
- Live `Car Status` section shows current mode, direction, and speed target.
- `Infrared Obstacle Sensors` panel shows Left/Right IR status as `Close`/`Clear`.
- `Refresh IR` reads sensors on demand; `Auto Refresh` updates the IR status continuously.
- `Raw L/Raw R` shows direct GPIO values (`0`/`1`) for diagnosis.
- `Swap L/R` swaps display mapping in GUI (useful if wiring sides are reversed).
- `Ultrasonic Sensor` panel shows obstacle distance in centimeters and `Close/Clear` status.
- `Refresh Distance` reads ultrasonic on demand; `Auto Refresh` keeps distance/status updated continuously.
- `Camera Control` panel supports servo pan control (`Cam Left`, `Center`, `Cam Right`, and `Set Angle`).
- Camera control uses `/home/pi/camera_servo.py` on the Pi (already deployed by this tool).
- `Camera View` section supports in-panel live preview from stream URL (snapshot endpoint).
- Use `Start Stream` to launch `mjpg-streamer` on Pi, then enable `Preview` to show camera frames in GUI.

## Notes

- If SSH asks for a password, enter it in the terminal.
- If you use SSH keys, pass `--identity-file ~/.ssh/id_rsa` (or your key path).
- Update `--remote-template` to match your actual script on the Pi.
- By default, strict host key checking is `accept-new` for first-time connection convenience.
