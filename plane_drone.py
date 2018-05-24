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
    SUB_MODE_ASCENDDESCEND = 5

class Udaciplane(Drone):
    """
    UnityDrone class adds additional low-level capabilities to control the
    Unity simulation version of the drone
    """
    
    def __init__(self, connection, tlog_name="TLog.txt"):
        
        super().__init__(connection, tlog_name)
        
        self._target_north = 0.0
        self._target_east = 0.0
        self._target_down = 0.0
        self._target_position_time = 0.0
        
        self._target_velocity_north = 0.0
        self._target_velocity_east = 0.0
        self._target_velocity_down = 0.0
        self._target_velocity_time = 0.0
        
        self._target_acceleration_north = 0.0
        self._target_acceleration_east = 0.0
        self._target_acceleration_down = 0.0
        self._target_acceleration_time = 0.0
        
        self._target_roll = 0.0
        self._target_pitch = 0.0
        self._target_yaw = 0.0
        self._target_attitude_time = 0.0
        
        self._target_roll_rate = 0.0
        self._target_pitch_rate = 0.0
        self._target_yaw_rate = 0.0
        self._target_body_rate_time = 0.0
        
        #Used for the autograder
        self.all_horizontal_errors = np.empty((0),float)
        self._threshold_horizontal_error = 2.0
        self.all_vertical_errors = np.empty((0),float)
        self._threshold_vertical_error = 1.0
        self.all_times = np.empty((0),float)
        self._threshold_time = 20.0
        self._average_horizontal_error = 0.0
        self._maximum_horizontal_error = 0.0
        self._average_vertical_error = 0.0
        self._maximum_vertical_error = 0.0
        self._mission_time = 0.0
        self._time0 = None
        self._mission_success = True
        
        #Visdom visualizer
        self._visdom_connected = False
        if visdom_available:
            self._v = visdom.Visdom()
            if self._v.check_connection():
                self._visdom_connected = True
                self._initialize_plots()
            else:
                self._visdom_connected = False
                print('For visual autograder start visdom server: python -m visdom.server')
        else:
            print('Visdom library not installed...')

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
        self.connection.set_sub_mode(PlaneMode.SUB_MODE_MANUAL.value)
        self.connection.cmd_moment(aileron, elevator, rudder, throttle)
    
    def cmd_ascenddescend(self, roll, airspeed, sideslip, throttle):
        self.connection.set_sub_mode(PlaneMode.SUB_MODE_ASCENDDESCEND.value)
        self.connection.cmd_moment(roll, airspeed, sideslip, throttle)

    def cmd_moment(self, roll_moment, pitch_moment, yaw_moment, thrust):
        """Command the drone moments.

        Args:
            roll_moment: in Newton*meter
            pitch_moment: in Newton*meter
            yaw_moment: in Newton*meter
            thrust: upward force in Newtons
        """
        try:
            self.connection.cmd_moment(roll_moment, pitch_moment, yaw_moment, thrust)
        except Exception as e:
            # traceback.print_exc()
            pass
        
        
        
        
    
        
    @property
    def local_position_target(self):
        return np.array([self._target_north,self._target_east,self._target_down])
    
    @local_position_target.setter    
    def local_position_target(self, target):
        """Pass the local position target to the drone (not a command)"""
        self._target_north = target[0]
        self._target_east = target[1]
        self._target_down = target[2]
        t = 0 #TODO: pass along the target time
        try:
            self.connection.local_position_target(target[0], target[1], target[2], t)
        except:
            # traceback.print_exec()
            pass
        
        #Check for current xtrack error
        if self._time0 is None:
            self._time0 = time.clock()
        
        self._horizontal_error = self.calculate_horizontal_error()
        self.all_horizontal_errors = np.append(self.all_horizontal_errors,self._horizontal_error)
        #print(self._horizontal_error)
        self._vertical_error = self.calculate_vertical_error()
        self.all_vertical_errors = np.append(self.all_vertical_errors,self._vertical_error)
        self._mission_time = time.clock() - self._time0
        self.all_times = np.append(self.all_times,self._mission_time)
        self.check_mission_success()
        if self._visdom_connected:
            self._add_visual_data()
            
    @property
    def local_velocity_target(self):
        return np.array([self._target_velocity_north,self._target_velocity_east,self._target_velocity_down])
    
    @local_velocity_target.setter
    def local_velocity_target(self, target):
        """Pass the local velocity target to the drone (not a command)"""
        
        self._target_velocity_north = target[0]
        self._target_velocity_east = target[1]
        self._target_velocity_down = target[2]
        t = 0 #TODO: pass along the target time
        try:
            self.connection.local_velocity_target(target[0], target[1], target[2], t)
        except:
            # traceback.print_exec()
            pass
            
    @property
    def local_acceleration_target(self):
        return np.array([self._target_acceleration_north,self._target_acceleration_east,self._target_acceleration_down])
    
    @local_acceleration_target.setter
    def local_acceleration_target(self,target):
        self._target_acceleration_north = target[0]
        self._target_acceleration_east = target[1]
        self._target_acceleration_down = target[2]
        t = 0 #TODO: pass along the target time
        try:
            self.connection.local_acceleration_target(target[0],target[1],target[2], t)
        except:
            # traceback.print_exec()
            pass
    @property
    def attitude_target(self):
        return np.array([self._target_roll,self._target_pitch,self._target_yaw])
    
    @attitude_target.setter
    def attitude_target(self, target):
        """Pass the attitude target to the drone (not a command)"""
        self._target_roll = target[0]
        self._target_pitch = target[1]
        self._target_yaw = target[2]
        t = 0 #TODO: pass along the target time        
        try:
            self.connection.attitude_target(target[0], target[1], target[2], t)
        except:
            # traceback.print_exec()
            pass
            
    @property
    def body_rate_target(self):
        return np.array([self._target_roll_rate,self._target_pitch_rate,self._target_yaw_rate])
    
    @body_rate_target.setter
    def body_rate_target(self, target):
        """Pass the local position target to the drone (not a command)"""
        self._target_roll_rate = target[0]
        self._target_pitch_rate = target[1]
        self._target_yaw_rate = target[2]
        t = 0 #TODO: pass along the target time
        try:
            self.connection.body_rate_target(target[0],target[1],target[2], t)
        except:
            # traceback.print_exec()
            pass
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    @property
    def threshold_horizontal_error(self):
        """Maximum allowed xtrack error on the mission"""
        return self._threshold_xtrack
    
    @threshold_horizontal_error.setter
    def threshold_horizontal_error(self, threshold):
        if threshold > 0.0:
            self._threshold_horizontal_error = threshold
        else:
            print('Horizontal error threshold must be greater than 0.0')

    @property
    def threshold_vertical_error(self):
        """Maximum allowed xtrack error on the mission"""
        return self._threshold_vertical_error
    
    @threshold_vertical_error.setter
    def threshold_vertical(self, threshold):
        if threshold > 0.0:
            self._threshold_vertical_error = threshold
        else:
            print('Vertical error threshold must be greater than 0.0')
    
    @property
    def threshold_time(self):
        """Maximum mission time"""
        return self._threshold_time
    
    @threshold_time.setter
    def threshold_time(self,threshold):
        if threshold > 0.0:
            self._threshold_time = threshold
        else:
            print('Time threshold must be greater than 0.0')
            
            
    
    def load_test_trajectory(self,time_mult=1.0):
        """Loads the test_trajectory.txt
        
        Args:
            time_mult: a multiplier to decrease the total time of the trajectory
        
        """
        data  = np.loadtxt('test_trajectory.txt', delimiter=',', dtype='Float64')
        position_trajectory = []
        time_trajectory = []
        yaw_trajectory = []
        current_time = time.time()
        for i in range(len(data[:,0])):
            position_trajectory.append(data[i,1:4])
            time_trajectory.append(data[i,0]*time_mult+current_time)
        for i in range(0,len(position_trajectory)-1):
            yaw_trajectory.append(np.arctan2(position_trajectory[i+1][1]-position_trajectory[i][1],position_trajectory[i+1][0]-position_trajectory[i][0]))
        yaw_trajectory.append(yaw_trajectory[-1])
        return(position_trajectory,time_trajectory,yaw_trajectory)
    
    def calculate_horizontal_error(self):
        """Calcuate the error beteween the local position and target local position
        
        """
        target_position = np.array([self._target_north,self._target_east])
        return np.linalg.norm(target_position-self.local_position[0:2])
    
    def calculate_vertical_error(self):
        """Calculate the error in the vertical direction"""
        return np.abs(self._target_down-self.local_position[2])
    
    def print_mission_score(self):
        """Prints the maximum xtrack error, total time, and mission success

        """
        print('Maximum Horizontal Error: ', self._maximum_horizontal_error)
        print('Maximum Vertical Error: ', self._maximum_vertical_error)
        print('Mission Time: ', self._mission_time)
        print('Mission Success: ', self._mission_success)
        if self._visdom_connected:
            self._show_plots()
        
    def check_mission_success(self):
        """Check the mission success criterion (xtrack and time)
        
        """
        if self._horizontal_error > self._maximum_horizontal_error:
            self._maximum_horizontal_error = self._horizontal_error
            if self._maximum_horizontal_error > self._threshold_horizontal_error:
                self._mission_success = False
        if self._vertical_error > self._maximum_vertical_error:
            self._maximum_vertical_error = self._vertical_error
            if self._maximum_vertical_error > self._threshold_vertical_error:
                self._mission_success = False
        if self._mission_time > self._threshold_time:
            self._mission_success = False
        
        
     

    def _show_plots(self):
        self._horizontal_plot = self._v.line(self.all_horizontal_errors, X=self.all_times, opts=dict(title="Horizontal Error",xlabel="Time(s)",ylabel="Error (m)"))
        self._vertical_plot = self._v.line(self.all_vertical_errors, X=self.all_times, opts=dict(title="Vertical Error", xlabel="Time(s)", ylabel="Error (m)"))
        
    def _initialize_plots(self):
        #self._horizontal_plot = self._v.line(np.array([0.0]),X=np.array([0.0]),opts=dict(title="Horizontal Error",xlabel="Time(s)",ylabel="Error (m)"))
        pass

    def _add_visual_data(self):
        #self._v.line(np.array([self._horizontal_error]),X=np.array([self._mission_time]),win=self._horizontal_plot,update='append')
        pass
    
    def cmd_position(self, target_north, target_east, target_down, yaw):
        pass