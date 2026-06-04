"""Simulated sensor_msgs for drone SITL (only the fields the station code reads)."""
class NavSatFix:
    def __init__(self): self.latitude = 0.0; self.longitude = 0.0; self.altitude = 0.0
class BatteryState:
    def __init__(self): self.percentage = 1.0; self.voltage = 16.8
