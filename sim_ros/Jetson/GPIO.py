"""Simulated Jetson.GPIO for the drone's winch DC motor. counterclockwise() = lower the parcel,
clockwise() = reel in. We translate motor direction into the per-drone winch flag files that the
Gazebo winch manager (parcel_fleet.py) already understands, so the real DCmotor code visibly lowers
the parcel onto the drop pad in simulation. On real hardware the genuine Jetson.GPIO drives the pins."""
import os
BCM = "BCM"; BOARD = "BOARD"; OUT = "OUT"; IN = "IN"; HIGH = 1; LOW = 0
_pins = {}
def setmode(_): pass
def setwarnings(_): pass
def setup(pin, mode, initial=LOW): _pins[pin] = initial
def cleanup(*a): _pins.clear()
def _inst(): return os.environ.get("DRONE_INSTANCE", "0")
def output(pin, value):
    _pins[pin] = value
    # MOTOR_B_A1=5 high + B1=6 low = clockwise (reel in); A1 low + B1 high = counterclockwise (lower)
    a1, b1 = _pins.get(5, LOW), _pins.get(6, LOW)
    i = _inst()
    if b1 and not a1:            # counterclockwise -> lower the parcel
        try:
            open(f"/tmp/airpost_winch_ground_{i}", "w").write("0.25")
            open(f"/tmp/airpost_winch_go_{i}", "w").write("go")
        except OSError: pass
