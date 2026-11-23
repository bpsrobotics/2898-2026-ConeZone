# 2898-2026-ConeZone

2898's robot code for the 2026 Bunnybots season, "Cone Zone".

## Hardware

- Drivebase: Swerve w/ REV Spark MAX driving NEO v1.1 motors, CTRE CANcoders, NavX 2 MXP
- Vision: Orange Pi 5 w/ PhotonVision, 2x Arducam OV9281, 1x Arducam AR0234

## Diagnosing roboRIO dropouts

From a laptop on the robot network, run `python3 tools/monitor_rio.py --host 10.28.98.2`.
Use `--output filename.csv` to choose the output file. Press Ctrl-C to stop.
The CSV records UTC receive time, RIO uptime, total and available RAM, and the Java
process PID and resident memory once per second. It also records SSH disconnects.
Compare timestamps with the Driver Station event log: reset uptime suggests a RIO
reboot; a changing or missing Java PID suggests robot code restarted or exited.
