# -*- coding: utf-8 -*-
import numpy as np
PI = 3.14159
class LongitudinalAutoPilot(object):
    def __init__(self):
        self.max_throttle_rpm = 2500
        self.max_elevator = 30.0*PI/180.0
        
        self.min_throttle = 0.0
        self.max_throttle = 1.0
        self.max_pitch_cmd = 30.0*np.pi/180.0
        self.max_pitch_cmd2 = 45.0*np.pi/180.0
        
        self.speed_int = 0.0
        self.alt_int = 0.0
        self.climb_speed_int = 0.0
        
        
        
        return
    
    
    
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
    def altitude_loop(self, altitude, altitude_cmd, dt):
        pitch_cmd = 0.0
        # STUDENT CODE HERE
        
        # START SOLUTION
        gain_p_alt = 0.03
        gain_i_alt = 0.005
       
        altitude_error = altitude_cmd-altitude
        self.alt_int = self.alt_int + altitude_error*dt

            
        pitch_cmd_unsat = gain_p_alt*altitude_error + gain_i_alt*self.alt_int
        
        if(abs(pitch_cmd_unsat)>self.max_pitch_cmd):
            pitch_cmd = np.sign(pitch_cmd_unsat)*self.max_pitch_cmd
        else:
            pitch_cmd = pitch_cmd_unsat
        
        # Integrator anti-windup
        if(gain_i_alt != 0):
            self.alt_int = self.alt_int + dt/gain_i_alt*(pitch_cmd-pitch_cmd_unsat)
        # END SOLUTION
        
        return pitch_cmd
    

    """Used to calculate the throttle command required command the target 
    airspeed
        
        Args:
            airspeed: in meters/sec
            airspeed_cmd: in meters/sec
        
        Returns:
            throttle_command: in percent throttle [0,1]
    """
    def airspeed_loop(self, airspeed, airspeed_cmd, dt):        
        throttle_cmd = 0.0
        # STUDENT CODE HERE
        
        # START SOLUTION
        throttle_ff = 0.67
        gain_p_speed = 0.2
        gain_i_speed = 0.1
        speed_error = (airspeed_cmd-airspeed)
        self.speed_int = self.speed_int + speed_error*dt
        
        throttle_cmd_unsat = gain_p_speed*speed_error + gain_i_speed * self.speed_int + throttle_ff
        
        #Anti windup        
        if(throttle_cmd_unsat > self.max_throttle):
            throttle_cmd = self.max_throttle
        elif(throttle_cmd_unsat < self.min_throttle):
            throttle_cmd = self.min_throttle
        else:
            throttle_cmd = throttle_cmd_unsat
        
        if(gain_i_speed != 0):
            self.speed_int = self.speed_int + dt/gain_i_speed*(throttle_cmd-throttle_cmd_unsat)                
        # END SOLUTION
        return throttle_cmd
    """Used to calculate the pitch command required to maintain the commanded
    airspeed
    
        Args:
            airspeed: in meters/sec
            airspeed_cmd: in meters/sec
        
        Returns:
            pitch_cmd: in radians
    """
    def airspeed_pitch_loop(self, airspeed, airspeed_cmd, dt):
        pitch_cmd = 0.0
        # STUDENT CODE HERE
        
        # START SOLUTION
        gain_p_airspeed = -0.2
        gain_i_airspeed = -0.1
        
        airspeed_error = airspeed_cmd-airspeed
        self.climb_speed_int = self.climb_speed_int + airspeed_error*dt
       
        pitch_cmd_unsat = gain_p_airspeed*airspeed_error + gain_i_airspeed*self.climb_speed_int

        if(np.abs(pitch_cmd_unsat) > self.max_pitch_cmd2):
            pitch_cmd = np.sign(pitch_cmd_unsat)*self.max_pitch_cmd2
        else:
            pitch_cmd = pitch_cmd_unsat
            
        # Anti wind-up
        if(gain_i_airspeed != 0):
            self.climb_speed_int = self.climb_speed_int + dt/gain_i_airspeed*(pitch_cmd - pitch_cmd_unsat)

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


 
class LateralAutoPilot:
    
    def __init__(self):
        self.g = 9.81
        self.integrator_yaw = 0.0 
        self.integrator_beta = 0.0
        self.gate = 1



    """Used to calculate the commanded aileron based on the roll error
    
        Args:
            airspeed: in meter/sec
            altitude: in meters (positive up)
            airspeed_cmd: in meters/sec
            altitude_cmd: in meters/sec (positive up)
            
        Returns:
            pitch_cmd: in radians
            throttle_cmd: in in percent throttle [0,1]
    """
    def roll_attitude_hold_loop(self,
                                phi_c,  # commanded roll
                                phi,    # actual roll 
                                roll_rate, 
                                T_s = 0.0,
                                phi_ff = 0.0):
        aileron = 0
        # STUDENT CODE HERE
        
        
        # START SOLUION
        gain_p_phi = 40.0
        gain_d_phi = 1.0
        aileron = gain_p_phi*(phi_c-phi) - gain_d_phi*roll_rate
        # END SOLUTION
        return aileron


    def yaw_hold_loop(self,
                         yaw_cmd,  # desired heading
                         yaw,     # actual heading 
                         T_s
                         ):
        roll_cmd = 0
        
        # STUDENT CODE HERE
        
        # START SOLUTION
        gain_p_yaw = 2.0
        gain_i_yaw = 0.01
        yaw_error = yaw_cmd-yaw
        while(yaw_error < np.pi):
            yaw_error = yaw_error + 2*np.pi
        
        while(yaw_error >= np.pi):
            yaw_error = yaw_error - 2*np.pi
        self.integrator_yaw = self.integrator_yaw + yaw_error*T_s
        
        if(self.integrator_yaw > 100):
            self.integrator_yaw = 100
        elif(self.integrator_yaw < -100):
            self.integrator_yaw = -100
        roll_cmd = gain_p_yaw*yaw_error+gain_i_yaw*self.integrator_yaw
        # END SOLUTION
        return roll_cmd



    def sideslip_hold_loop(self,
                           beta, # sideslip angle 
                           T_s):
        rudder = 0
        # STUDENT CODE HERE
        
        # START SOLUTION
        gain_p_beta = 1.0
        gain_i_beta = 1.0
        self.integrator_beta = self.integrator_beta+(0.0-beta)*T_s
        rudder = -1.0*(gain_p_beta*(0.0-beta)+gain_i_beta*self.integrator_beta)
        #END SOLUTION
        return rudder
    
    def straight_line_guidance(self, line_origin, line_course, 
                               local_position):
        course_cmd = 0
        # STUDENT CODE HERE
        
        # START SOLUTION
        gain_p_xtrack = 0.003
        xtrack_error = np.cos(line_course)*(local_position[1]-line_origin[1])+\
            -np.sin(line_course) * (local_position[0]-line_origin[0])
        course_cmd = -np.pi/2*np.arctan(gain_p_xtrack * xtrack_error) +\
            line_course;
        # END SOLUTION
        return course_cmd
    
    def orbit_guidance(self, orbit_center, orbit_radius, local_position, yaw,
                       clockwise = True):
        course_cmd = 0
        # STUDENT CODE HERE
        
        # START SOLUTION
        gain_orbit = 1.5
        radius = np.linalg.norm(orbit_center[0:2]-local_position[0:2])
        course_cmd = np.pi / 2 + np.arctan(
                gain_orbit * (radius - orbit_radius) / orbit_radius);
        if (clockwise == False):
            course_cmd = -course_cmd;

        addon = np.arctan2(local_position[1] - orbit_center[1],
                           local_position[0] - orbit_center[0]);
        if (addon-yaw < -np.pi):
            while (addon-yaw < -np.pi):
                addon = addon + np.pi * 2;
        elif (addon - yaw > np.pi):
            while (addon - yaw > np.pi):
                addon = addon - np.pi * 2;
        course_cmd = course_cmd + addon;        
        # END SOLUTION
        return course_cmd

    # turn_rate positive is cw, negative is ccw
    def coordinated_turn_ff(self, speed, radius, cw):
        
        roll_ff = 0
        # STUDENT CODE HERE
        
        # START SOLUTION
        if(cw):
            roll_ff = np.arctan(speed**2/(self.g*radius))
        else:
            roll_ff = -np.arctan(speed**2/(self.g*radius))
        # END SOLUTION
        return roll_ff

    def path_manager(self, local_position, yaw, airspeed_cmd):
        
        roll_ff = 0
        yaw_cmd = 0
        # STUDENT CODE HERE
        
        # START SOLUTION
        if(self.gate == 1):
            if(local_position[0] > 500):
                self.gate = self.gate+1
                print('Gate 1 Complete')
                print('Yaw Int = ',self.integrator_yaw)
                self.integrator_yaw = 0.0
            else:
                roll_ff = 0.0
                line_origin = np.array([0.0, 50.0, -450.0])
                line_course = 0.0
                yaw_cmd = self.straight_line_guidance(line_origin, line_course,
                                             local_position)
        if(self.gate == 2):
            if(local_position[1] < -350):
                self.gate = self.gate+1
                print('Gate 2 Complete')
                print('Yaw Int = ',self.integrator_yaw)
                self.integrator_yaw = 0.0
            else:
                radius = 400
                cw = False
                orbit_center = np.array([500.0, -350.0, -450.0])
                roll_ff = self.coordinated_turn_ff(airspeed_cmd, radius, cw)
                yaw_cmd = self.orbit_guidance(orbit_center, radius, 
                                              local_position, yaw, cw)
        if(self.gate == 3):
            if(local_position[0] < 600):
                self.gate = self.gate+1
                print('Gate 3 Complete')
                print('Yaw Int = ',self.integrator_yaw)
                self.integrator_yaw = 0.0
            else:
                radius = 300
                cw = False
                orbit_center = np.array([600.0, -350.0, -450.0])
                roll_ff = self.coordinated_turn_ff(airspeed_cmd, radius, cw)
                yaw_cmd = self.orbit_guidance(orbit_center, radius, 
                                              local_position, yaw, cw)
        if(self.gate==4):
            if(local_position[0] < -500):
                print('Lateral Challenge Finished')
                print('Yaw Int = ',self.integrator_yaw)
                self.integrator_yaw = 0.0
            else:
                roll_ff = 0.0
                line_origin = np.array([600.0, -650.0, -450.0])
                line_course = np.pi
                yaw_cmd = self.straight_line_guidance(line_origin, line_course,
                                             local_position)
                #print('Yaw Cmd = ', yaw_cmd)
        if(self.gate > 4):
            roll_ff = 0.0
            yaw_cmd = 0.0
            print('Invalid gate')
            
        # END SOLUTION
        return(roll_ff,yaw_cmd)



def euler2RM(roll,pitch,yaw):
    R = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
    cr = np.cos(roll)
    sr = np.sin(roll)
    
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    
    R[0,0] = cp*cy
    R[1,0] = -cr*sy+sr*sp*cy
    R[2,0] = sr*sy+cr*sp*cy
    
    R[0,1] = cp*sy
    R[1,1] = cr*cy+sr*sp*sy
    R[2,1] = -sr*cy+cr*sp*sy
    
    R[0,2] = -sp
    R[1,2] = sr*cp
    R[2,2] = cr*cp
    
    return R.transpose()