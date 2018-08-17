import numpy as np
from enum import Enum

from udacidrone import Drone
import time
visdom_available= True
try:
    import visdom
except:
    visdom_available = False

class PlaneMode(Enum):
    """
    Constant which isn't defined in Mavlink but useful when dealing with
    the airplane simulation
    """
    SUB_MODE_MANUAL = 1
    SUB_MODE_LONGITUDE = 2
    SUB_MODE_LATERAL = 3
    SUB_MODE_STABILIZED = 4
    SUB_MODE_VTOL_ATTITUDE = 9
    SUB_MODE_VTOL_POSITION = 10

class Udaciplane(Drone):
    """
    Udaciplane class for use with the Unity Fixed Wing/Flying Car simulation
    """
    
    def __init__(self, connection, tlog_name="TLog.txt"):
        
        super().__init__(connection, tlog_name)
        

    def cmd_stabilized(self, roll, altitude, sideslip, airspeed):
        """Command the stabilized mode of the drone
        
        Args:
            roll: in radians
            altitude: in meters (positive up)
            sideslip: in radians (positive nose left)
            airspeed: in meters/sec
        
        """
        self.connection.set_sub_mode(PlaneMode.SUB_MODE_STABILIZED.value)
        self.connection.cmd_moment(roll, altitude, sideslip, airspeed)

    def cmd_longitude_mode(self, elevator, throttle, roll = 0, sideslip = 0,
                           t=0):
        """Command the longitude mode while lateral is stabilized
        
        Args:
            elevator: in percentage of maximum elevator (-1:1)
            throttle: in percentage of maximum throttle RPM (0:1)
            roll: in radians
            sideslip: in radians (positive nose left)
        """
        self.connection.set_sub_mode(PlaneMode.SUB_MODE_LONGITUDE.value)
        self.connection.cmd_moment(roll, elevator, sideslip, throttle, t)

        
    def cmd_lateral_mode(self, aileron, rudder, altitude, airspeed):
        """Command the lateral mode while longitudinal mode is stabilized
        
        Args:
            aileron: in percentage of maximum aileron (-1:1)
            rudder: in percentage of maximum rudder (-1:1)
            altitude: in meters (positive up)
            airspeed: in meters/sec
        """
        self.connection.set_sub_mode(PlaneMode.SUB_MODE_LATERAL.value)
        self.connection.cmd_moment(aileron, altitude, rudder, airspeed)
        
    
    def cmd_controls(self, aileron, elevator, rudder, throttle):
        """Command the manual aircraft controls
        
        Args:
            aileron: in percentage of maximum aileron (-1:1)
            rudder: in percentage of maximum rudder (-1:1)
            elevator: in percentage of maximum elevator (-1:1)
            throttle: in percentage of maximum throttle RPM (0:1)
        """
        self.connection.set_sub_mode(PlaneMode.SUB_MODE_MANUAL.value)
        controls = [aileron, elevator, rudder, throttle]
        self.connection.cmd_controls(controls)
    
    def cmd_hybrid(self, aileron, elevator, rudder, throttle, roll_moment, pitch_moment, yaw_moment, thrust):
        """Command the manual aircraft controls, the VTOL moments and total thrust force
        
        Args:
            aileron: in percentage of maximum aileron (-1:1)
            rudder: in percentage of maximum rudder (-1:1)
            elevator: in percentage of maximum elevator (-1:1)
            throttle: in percentage of maximum throttle RPM (0:1)
            roll_moment: in percentage of maximum roll moment (-1:1)
            pitch_moment: in percentage of maximum pitch moment (-1:1)
            yaw_moment: in percentage of maximum yaw_moment (-1:1)
            thrust: in percentage of maximum thrust (0:1)
        """
        self.connection.set_sub_mode(PlaneMode.SUB_MODE_MANUAL.value)
        controls = [aileron, elevator, rudder, throttle, roll_moment, pitch_moment, yaw_moment , thrust]
        self.connection.cmd_controls(controls)
    
    def cmd_moment(self, roll_moment, pitch_moment, yaw_moment, thrust):
        """Command the VTOL moments and total thrust force
        
        Args:
            roll_moment: in percentage of maximum roll moment (-1:1)
            pitch_moment: in percentage of maximum pitch moment (-1:1)
            yaw_moment: in percentage of maximum yaw_moment (-1:1)
            thrust: in percentage of maximum thrust (0:1)
        """
        self.connection.set_sub_mode(PlaneMode.SUB_MODE_MANUAL.value)
        controls = [0.0, 0.0, 0.0, 0.0, roll_moment, pitch_moment, yaw_moment, thrust]
        self.connection.cmd_controls(controls)    
        
    def cmd_vtol_position(self, north, east, altitude, heading):
        """Command the local position and drone heading.

        Args:
            north: local north in meters
            east: local east in meters
            altitude: altitude above ground in meters
            heading: drone yaw in radians
        """
        self.connection.set_sub_mode(PlaneMode.SUB_MODE_VTOL_POSITION.value)
        self.cmd_position(north, east, altitude, heading)
        
    def cmd_vtol_attitude(self,roll, pitch, yaw_rate, vert_vel):
        """Command the drone through attitude command

        Args:
            roll: in radians
            pitch: in randians
            yaw_rate: in radians/second
            vert_vel: upward velocity in meters/second
        """
        self.connection.set_sub_mode(PlaneMode.SUB_MODE_VTOL_ATTITUDE.value)
        self.cmd_attitude(roll, pitch, yaw_rate, vert_vel)

