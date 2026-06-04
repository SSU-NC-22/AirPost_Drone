"""Simulated mavros_msgs.msg for drone SITL — Waypoint + the small types drone_controller uses."""
class Waypoint:
    def __init__(self):
        self.frame = 0; self.command = 16; self.is_current = False; self.autocontinue = True
        self.param1 = self.param2 = self.param3 = self.param4 = 0.0
        self.x_lat = 0.0; self.y_long = 0.0; self.z_alt = 0.0
class OverrideRCIn:
    def __init__(self): self.channels = [0]*8
class RCIn:
    def __init__(self): self.channels = []
class WaypointReached:
    def __init__(self, wp_seq=0): self.wp_seq = wp_seq
