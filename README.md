# MasterController

An Arduino-based RC vehicle controller with GPS tracking, ultrasonic obstacle avoidance, Bluetooth remote control, and autonomous driving modes.

---

## Features

- **Bluetooth Control** — Drive and steer remotely via a Bluetooth serial connection
- **GPS Tracking** — Reads NMEA data, converts coordinates to UTM, and streams position over Bluetooth every 2 seconds
- **Obstacle Avoidance** — Four ultrasonic sensors detect obstacles and trigger proportional braking or a full stop
- **Launch Control** — Bursts to a high pulse on launch then smoothly ramps down to a cruise speed
- **Autonomous Rectangle Mode** — Drives a repeating rectangular pattern autonomously using a finite state machine (FSM)
- **Telemetry** — Broadcasts current PWM speed to the Bluetooth client at 200 ms intervals
- **Serial Monitor Control** — Manually set speed (PWM) and steering angle via USB serial for debugging
