# -*- coding: utf-8 -*-

from udacidrone.messaging import MsgID
from enum import Enum
from udacidrone.connection import MavlinkConnection
import numpy as np
from plane_drone import Udaciplane
from plane_control import PlaneControl
import time

class Scenario(Enum):
    SANDBOX = 0
    TRIM = 1
    AIRSPEED = 2
    ALTITUDE = 3
    CLIMB = 4
    LONGITUDINAL = 5

class FixedWingProject(Udaciplane):
        
    def __init__(self, connection, tlog_name="TLog.txt"):
        super().__init__(connection, tlog_name)
                
        self.controller = PlaneControl()
        
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
        
        self.scenario = Scenario.SANDBOX
        
        self.time_cmd = 0.0
        self.cmd_freq = 100.0
        
        self.last_airspeed_time = None
        self.last_position_time = None
        
        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION,
                               self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.airspeed_callback)
        self.register_callback(MsgID.STATE, self.state_callback)        
        self.register_callback(MsgID.ATTITUDE, self.attitude_callback)
        #self.register_callback(MsgID.RAW_GYROSCOPE, self.gyro_callback)
        #self.register_callback(MsgID.AIRSPEED, self.airspeed_callback)
        
    def state_callback(self):
        if(self.scenario != Scenario.SANDBOX):
            if(self.guided != True):
                self.stop()

    def airspeed_callback(self):
        #Assuming no wind, for now...
        self.airspeed = np.linalg.norm(self.local_velocity)
        dt = 0.0
        if(self.last_airspeed_time != None):
            dt = self.local_velocity_time - self.last_airspeed_time
            if(dt <= 0.0):
                return
            
        self.last_airspeed_time = self.local_velocity_time
                    
        if(self.scenario == Scenario.AIRSPEED):                
            self.throttle_cmd = self.controller.airspeed_loop(self.airspeed,
                                                     self.airspeed_cmd, dt,
                                                     0.67)
            self.cmd_longitude_mode(self.elevator_cmd, self.throttle_cmd,
                                    0,0,self.last_airspeed_time)
            
        if(self.scenario == Scenario.ALTITUDE):
            self.throttle_cmd = self.controller.airspeed_loop(self.airspeed,
                                                     self.airspeed_cmd, dt,
                                                     0.67)
            
        if(self.scenario == Scenario.CLIMB):
            self.pitch_cmd = self.controller.airspeed_pitch_loop(
                    self.airspeed, self.airspeed_cmd, dt)
            
        if(self.scenario == Scenario.LONGITUDINAL):            
            altitude = -self.local_position[2]
            [self.pitch_cmd, self.throttle_cmd] = \
                self.controller.longitudinal_loop(self.airspeed, altitude, \
                                                  self.airspeed_cmd, \
                                                  self.altitude_cmd, dt)
    
    def attitude_callback(self):
        if((self.scenario == Scenario.ALTITUDE) |
                (self.scenario == Scenario.CLIMB) |
                (self.scenario == Scenario.LONGITUDINAL)):
            self.elevator_cmd = self.controller.pitch_loop(self.attitude[1],
                                                           self.gyro_raw[1],
                                                           self.pitch_cmd)
            self.cmd_longitude_mode(self.elevator_cmd, self.throttle_cmd)
    
    def local_position_callback(self):
        dt = 0.0
        if(self.last_position_time != None):
            dt = self.local_position_time - self.last_position_time
            
        self.last_position_time = self.local_position_time
        if(dt <= 0.0):
            return
        
        if(self.scenario == Scenario.ALTITUDE):            
            altitude = -self.local_position[2]
            self.pitch_cmd = self.controller.altitude_loop(altitude,
                                                           self.altitude_cmd,
                                                           dt)
        if(self.scenario == Scenario.LONGITUDINAL):
            along_track = np.linalg.norm(self.local_position[0:2])
            print(along_track)
            if(along_track > self.gate_target[0]):
                if(len(self.longitudinal_gates)==0):
                    self.stop()
                else:
                    self.gate_target = self.longitudinal_gates.pop(0)
                    print('Gate Target: ', self.gate_target)
                    self.altitude_cmd = self.gate_target[1]     
    
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
        else:
            print('Invalid Scenario')
            return
        
        self.take_control()
        self.arm();
        self.start()
            
    
if __name__ == "__main__":
    conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://127.0.0.1:5760')
    drone = FixedWingProject(conn)
    time.sleep(2)
    drone.run_scenario(Scenario.LONGITUDINAL)
                
            
                
            
            
            
            
