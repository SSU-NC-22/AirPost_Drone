"""Simulated geometry_msgs for drone SITL."""
class _V:  # generic vector
    def __init__(self): self.x = 0.0; self.y = 0.0; self.z = 0.0
class _Twist:
    def __init__(self): self.linear = _V(); self.angular = _V()
class TwistStamped:
    def __init__(self): self.twist = _Twist()
class Pose: pass
class PoseStamped: pass
class Twist: pass
class Quaternion: pass
