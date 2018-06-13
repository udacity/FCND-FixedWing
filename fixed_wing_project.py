# -*- coding: utf-8 -*-

from udacidrone.messaging import MsgID
from enum import Enum
from udacidrone.connection import MavlinkConnection
import numpy as np
from plane_drone import Udaciplane
from plane_control import PlaneControl
from plane_control import LateralAutoPilot
from plane_control import euler2RM
import time

class Scenario(Enum):
    SANDBOX = 0
    TRIM = 1
    AIRSPEED = 2
    ALTITUDE = 3
    CLIMB = 4
    LONGITUDINAL = 5
    ROLL = 6
    TURN = 7
    YAW = 8
    LINE = 9
    ORBIT = 10
    LATERAL = 11

class FixedWingProject(Udaciplane):
        
    def __init__(self, connection, tlog_name="TLog.txt"):
        super().__init__(connection, tlog_name)
                
        self.controller = PlaneControl()
        self.lateral_autopilot = LateralAutoPilot()
        #defined as [along_track_distance (meters), altitude (meters)]
        self.longitudinal_gates = [np.array([200.0, 200.0]),
                                   np.array([1100.0, 300.0]),
                                   np.array([1400.0, 280.0]),
                                   np.array([2200.0, 200.0])]
        self.airspeed_cmd = 41.0
        self.altitude_cmd = 450.0
        self.throttle_cmd = 0.0
        self.elevator_cmd = 0.0        
        self.pitch_cmd = 0.0
        
        self.aileron_cmd = 0.0
        self.rudder_cmd = 0.0
        self.roll_cmd = 0.0
        self.sideslip_cmd = 0.0
        self.yaw_cmd = 0.0
        self.course_cmd = 0.0
        self.line_origin = np.array([0.0, 0.0, 0.0])
        
        self.scenario = Scenario.SANDBOX
        
        self.time_cmd = 0.0
        self.cmd_freq = 100.0
        
        self.last_airspeed_time = None
        self.last_position_time = None
        self.last_attitude_time = None
        
        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION,
                               self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.airspeed_callback)
        self.register_callback(MsgID.STATE, self.state_callback)        
        self.register_callback(MsgID.ATTITUDE, self.attitude_callback)
        #self.register_callback(MsgID.RAW_GYROSCOPE, self.gyro_callback)
        #self.register_callback(MsgID.AIRSPEED, self.airspeed_callback)
        self._scenario_started = False
        
    def state_callback(self):
        if(self.scenario != Scenario.SANDBOX):
            if(self._scenario_started == False):
                self.take_control()
                self.arm()
                self._scenario_started = True
                print('Start Scenario...')
            elif(self.guided != True):
                self.stop()

    def airspeed_callback(self):
        #Assuming no wind, for now...
        self.airspeed = np.linalg.norm(self.local_velocity)
        if(self.airspeed != 0.0):
            rot_mat = euler2RM(self.attitude[0], self.attitude[1], 
                               self.attitude[2])
            side_velocity = rot_mat.transpose()[1,0]*self.local_velocity[0] +\
            rot_mat.transpose()[1,1]*self.local_velocity[1] +\
            rot_mat.transpose()[1,2]*self.local_velocity[2]
            self.sideslip = np.arcsin(side_velocity/self.airspeed)
        else:
            self.sideslip = 0.0
        dt = 0.0
        if(self.last_airspeed_time != None):
            dt = self.local_velocity_time - self.last_airspeed_time
            if(dt <= 0.0):
                return
            
        self.last_airspeed_time = self.local_velocity_time

        if(self._scenario_started == False):
            return
        
        if(self.scenario == Scenario.AIRSPEED):                
            self.throttle_cmd = self.controller.airspeed_loop(self.airspeed,
                                                     self.airspeed_cmd, dt)
            self.cmd_longitude_mode(self.elevator_cmd, self.throttle_cmd,
                                    0,0,self.last_airspeed_time)
            
        if(self.scenario == Scenario.ALTITUDE):
            self.throttle_cmd = self.controller.airspeed_loop(self.airspeed,
                                                     self.airspeed_cmd, dt)
            
        if(self.scenario == Scenario.CLIMB):
            self.pitch_cmd = self.controller.airspeed_pitch_loop(
                    self.airspeed, self.airspeed_cmd, dt)
            
        if(self.scenario == Scenario.LONGITUDINAL):            
            altitude = -self.local_position[2]
            [self.pitch_cmd, self.throttle_cmd] = \
                self.controller.longitudinal_loop(self.airspeed, altitude, \
                                                  self.airspeed_cmd, \
                                                  self.altitude_cmd, dt)
        
        if(self.scenario == Scenario.TURN):
            self.rudder_cmd = self.lateral_autopilot.sideslip_hold_loop(
                    self.sideslip, dt)
    
    def attitude_callback(self):
        dt = 0.0
        if(self.last_attitude_time != None):
            dt = self.attitude_time-self.last_attitude_time
        self.last_attitude_time = self.attitude_time
        
        if(self._scenario_started == False):
            return
        
        if((self.scenario == Scenario.ALTITUDE) |
                (self.scenario == Scenario.CLIMB) |
                (self.scenario == Scenario.LONGITUDINAL)):
            self.elevator_cmd = self.controller.pitch_loop(self.attitude[1],
                                                           self.gyro_raw[1],
                                                           self.pitch_cmd)
            self.cmd_longitude_mode(self.elevator_cmd, self.throttle_cmd)
        
        if((self.scenario == Scenario.YAW) |
                (self.scenario == Scenario.LINE) |
                (self.scenario == Scenario.ORBIT)):
            self.roll_cmd = self.lateral_autopilot.yaw_hold_loop(
                    self.yaw_cmd, self.attitude[2], dt)
        
        if((self.scenario == Scenario.ROLL) |
                (self.scenario == Scenario.TURN) |
                (self.scenario == Scenario.YAW) |
                (self.scenario == Scenario.LINE) |
                (self.scenario == Scenario.ORBIT)):
            self.aileron_cmd = self.lateral_autopilot.roll_attitude_hold_loop(
                    self.roll_cmd, self.attitude[0], self.gyro_raw[0])
            self.cmd_lateral_mode(self.aileron_cmd, self.rudder_cmd,
                                  self.altitude_cmd, self.airspeed_cmd)
    
    def local_position_callback(self):
        dt = 0.0
        if(self.last_position_time != None):
            dt = self.local_position_time - self.last_position_time
            
        self.last_position_time = self.local_position_time
        if(dt <= 0.0):
            return
        
        if(self._scenario_started == False):
            return
        
        if(self.scenario == Scenario.ALTITUDE):            
            altitude = -self.local_position[2]
            self.pitch_cmd = self.controller.altitude_loop(altitude,
                                                           self.altitude_cmd,
                                                           dt)
        if(self.scenario == Scenario.LONGITUDINAL):
            along_track = np.linalg.norm(self.local_position[0:2])
            if(along_track > self.gate_target[0]):
                if(len(self.longitudinal_gates)==0):
                    self.stop()
                else:
                    self.gate_target = self.longitudinal_gates.pop(0)
                    print('Gate Target: ', self.gate_target)
                    self.altitude_cmd = self.gate_target[1]
        
        if(self.scenario == Scenario.LINE):
            self.yaw_cmd = self.lateral_autopilot.straight_line_guidance(
                    self.line_origin, self.line_course, self.local_position)
        
        if(self.scenario == Scenario.ORBIT):
            self.yaw_cmd = self.lateral_autopilot.orbit_guidance(
                    self.orbit_center, self.orbit_radius, self.local_position,
                    self.attitude[2], self.orbit_cw)
            
    
    def run_scenario(self,scenario):
        self.scenario = scenario
        
        if(scenario == Scenario.AIRSPEED):
            self.elevator_cmd = 0.0
            self.airspeed_cmd = 41.0
        elif(scenario == Scenario.ALTITUDE):
            self.airspeed_cmd = 41.0
            self.altitude_cmd = 450.0
        elif(scenario == Scenario.CLIMB):
            self.airspeed_cmd = 41.0
            self.throttle_cmd = 1.0
        elif(scenario == Scenario.LONGITUDINAL):
            self.airspeed_cmd = 41.0
            self.gate_target = self.longitudinal_gates.pop(0)
            self.altitude_cmd = self.gate_target[1]
        elif(scenario == Scenario.ROLL):
            self.airspeed_cmd = 41.0
            self.altitude_cmd = 450.0
            self.roll_cmd = 0.0  
            self.rudder_cmd = 0.0
        elif(scenario == Scenario.TURN):
            self.airpseed_cmd = 41.0
            self.altitude_cmd = 450.0
            self.roll_cmd = 45.0*np.pi/180.0
            self.sideslip_cmd = 0.0
        elif(scenario == Scenario.YAW):
            self.airspeed_cmd = 41.0
            self.altitude_cmd = 450.0
            self.yaw_cmd = 0.0;
            self.sideslip_cmd = 0.0
        elif(scenario == Scenario.LINE):
            self.airspeed_cmd = 41.0
            self.altitude_cmd = 450.0
            self.line_course = 0.0
            self.line_origin = np.array([0.0, 20.0, 450.0])
        elif(scenario == Scenario.ORBIT):
            self.airspeed_cmd = 41.0
            self.altitude_cmd = 450.0
            self.orbit_radius = 500.0
            self.orbit_center = np.array([0.0, 500.0, 450.0])
            self.orbit_cw = True
        else:
            print('Invalid Scenario')
            return
        
        
        #self.take_control()
        
        #self.arm()
        
        self.start()
            
  
if __name__ == "__main__":
    conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://127.0.0.1:5760')
    drone = FixedWingProject(conn)
    time.sleep(2)
    drone.run_scenario(Scenario.LONGITUDINAL)