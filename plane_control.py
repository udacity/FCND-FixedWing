# -*- coding: utf-8 -*-
import numpy as np
PI = 3.14159
class PlaneControl(object):
    def __init__(self):
        self.max_throttle_rpm = 2500
        self.max_elevator = 30.0*PI/180.0
        
        self.speed_int = 0.0
        self.alt_int = 0.0
        self.climb_speed_int = 0.0
        
        return
    
    
    """Used to calculate the throttle command required command the target 
    airspeed
        
        Args:
            airspeed: in meters/sec
            airspeed_cmd: in meters/sec
        
        Returns:
            throttle_command: in percent throttle [0,1]
    """
    def airspeed_loop(self, airspeed, airspeed_cmd, 
                      dt = 0.0):        
        throttle_cmd = 0.0
        # STUDENT CODE HERE
        
        # START SOLUTION
        throttle_ff = 0.67
        gain_p_speed = 0.2
        gain_i_speed = 0.1
        max_speed_int = 0.25
        speed_error = (airspeed_cmd-airspeed)
        self.speed_int = self.speed_int + speed_error*dt
        if(gain_i_speed*abs(self.speed_int) > max_speed_int):
            self.speed_int = np.sign(self.speed_int)*max_speed_int/gain_i_speed
        

        throttle_cmd = gain_p_speed*speed_error + gain_i_speed * self.speed_int
        throttle_cmd = throttle_cmd + throttle_ff
        # END SOLUTION
        return throttle_cmd
    
    """Used to calculate the elevator command required to acheive the target
    pitch
    
        Args:
            pitch: in radians
            pitch_rate: in radians/sec
            pitch_cmd: in radians
        
        Returns:
            elevator_cmd: in percentage elevator [-1,1]
    """
    def pitch_loop(self, pitch, pitch_rate, pitch_cmd):
        elevator_cmd = 0.0
        # STUDENT CODE HERE
        
        # START SOLUTION
        gain_p_pitch = 20.0
        gain_p_q = 10.0
        elevator_cmd = gain_p_pitch*(pitch_cmd - pitch) - gain_p_q*pitch_rate
        # END SOLUTION
        return elevator_cmd
    
    """Used to calculate the pitch command required to maintain the commanded
    altitude
    
        Args:
            altitude: in meters (positive up)
            altitude_cmd: in meters (positive up)
        
        Returns:
            pitch_cmd: in radians
    """
    def altitude_loop(self, altitude, altitude_cmd, dt = 0.0):
        pitch_cmd = 0.0
        # STUDENT CODE HERE
        
        # START SOLUTION
        gain_p_alt = 0.03
        gain_i_alt = 0.005
        max_alt_int = 0.1
        max_pitch_cmd = 30.0*np.pi/180.0
        
        altitude_error = altitude_cmd-altitude
        self.alt_int = self.alt_int + altitude_error*dt
        if(abs(self.alt_int) > max_alt_int):
            self.alt_int = np.sign(self.alt_int)*max_alt_int
            
        pitch_cmd = gain_p_alt*altitude_error + gain_i_alt*self.alt_int
        
        if(abs(pitch_cmd)>max_pitch_cmd):
            pitch_cmd = np.sign(pitch_cmd)*max_pitch_cmd
        # END SOLUTION
        
        return pitch_cmd
    
    """Used to calculate the pitch command required to maintain the commanded
    airspeed
    
        Args:
            airspeed: in meters/sec
            airspeed_cmd: in meters/sec
        
        Returns:
            pitch_cmd: in radians
    """
    def airspeed_pitch_loop(self, airspeed, airspeed_cmd,
                            dt = 0.0, pitch_ff = 0.0):
        pitch_cmd = 0.0
        # STUDENT CODE HERE
        
        # START SOLUTION
        gain_p_airspeed = 0.2
        gain_i_airspeed = 0.02
        max_airspeed_int = 50.0
        
        airspeed_error = airspeed_cmd-airspeed
        self.climb_speed_int = self.climb_speed_int + airspeed_error*dt
        if(abs(self.climb_speed_int) > max_airspeed_int):
            self.climb_speed_int = np.sign(self.climb_speed_int)*max_airspeed_int
        pitch_cmd = -1.0*(gain_p_airspeed*airspeed_error +
                          gain_i_airspeed*self.climb_speed_int)
        pitch_cmd = pitch_cmd + pitch_ff
        #print(gain_i_airspeed*self.climb_speed_int)

        # END SOLUTION
        return pitch_cmd
    
    """Used to calculate the pitch command and throttle command based on the
    aicraft altitude error
    
        Args:
            airspeed: in meter/sec
            altitude: in meters (positive up)
            airspeed_cmd: in meters/sec
            altitude_cmd: in meters/sec (positive up)
            
        Returns:
            pitch_cmd: in radians
            throttle_cmd: in in percent throttle [0,1]
    """
    def longitudinal_loop(self, airspeed, altitude, airspeed_cmd, altitude_cmd,
                          dt = 0.0):
        pitch_cmd = 0.0
        throttle_cmd = 0.0
        # STUDENT CODE HERE
        
        # START SOLUTION
        max_altitude_diff = 25.0
        if(altitude_cmd-altitude > max_altitude_diff):
            throttle_cmd = 1.0
            pitch_cmd = self.airspeed_pitch_loop(airspeed, airspeed_cmd, dt)
        elif(altitude - altitude_cmd > max_altitude_diff):
            throttle_cmd = 0.1
            pitch_cmd = self.airspeed_pitch_loop(airspeed, airspeed_cmd, dt)
        else:
            throttle_cmd = self.airspeed_loop(airspeed, airspeed_cmd, dt)
            pitch_cmd = self.altitude_loop(altitude, altitude_cmd, dt)
        # END SOLUTION
        return[pitch_cmd, throttle_cmd]