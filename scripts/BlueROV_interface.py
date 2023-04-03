
#!/usr/bin/env python3

"""
Interface-object for BlueROV communication
"""

# Disable "Bare exception" warning
# pylint: disable=W0702

import time
from pymavlink import mavutil
import threading
from tools import *

import rospy
import sensor_msgs.msg
import subprocess
import nav_msgs.msg
import geometry_msgs.msg 


# 1 m = 100.380869
ATMOSPHERIC_PRESSURE = 1013
MILLIBAR_TO_METER =  1 / 100.380869



class BlueROV:
    def __init__(self, port):
        self.conn_ok = True
        self.check_connection()

        self.connection = mavutil.mavlink_connection('udpin:0.0.0.0:{}'.format(port))
        self.lock = threading.Lock()


        self.connection.wait_heartbeat(timeout=2)
        
        
        # if response == None:
        #     self.conn_ok = False
            
        
        self.frame_rate_IMU = 2
        self.frame_rate_AHRS = 2
        self.frame_rate_Pressure = 2


        ## camera should be in the same tilt every time. 
        ## range [-5000, 5000]
        self.camera_position = 0

        ## set intervals for the different messages
        

        ## thread handles
        self.UDP_reader_thread_running = False

        ## acceleration
        self.accx = 0
        self.accy = 0
        self.accz = 0

        # gyro
        self.gyrox = 0
        self.gyroy = 0
        self.gyroz = 0

        # magnetometer
        self.magx = 0
        self.magy = 0
        self.magz = 0

        self.IMU_time = 0

        # pressure / depth
        self.pressure = 0
        self.depth = 0
        self.pressure_time = 0

        # altitude / height
        self.altitude = 0

        # heading
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # heading - quaternion
        self.qx = 0
        self.qy = 0
        self.qz = 0
        self.qw = 0

        self.ahrs_time = 0
        self.bluerov_system_time = 0

        self.new_imu_message = False
        self.new_pressure_message = False

        ## DVL local ned -- to be sent back to the bluerov
        self.DVL_x = 0
        self.DVL_y = 0
        self.DVL_z = 0
        self.DVL_vx = 0
        self.DVL_vy = 0
        self.DVL_vz = 0
        self.DVL_altitude = 0 

        ## SLAM local ned -- to be sent back to the bluerov
        self.SLAM_x = 0
        self.SLAM_y = 0
        self.SLAM_z = 0

        self.utime_start = time.time()*1000

        


        self.camera_pitch = 0 # rad

        self.R_dvl_bluerov = np.eye(3)
        self.R_bluerov_world = np.eye(3)
        self.R_dvl_world = np.eye(3)

    def set_R_dvl_bluerov(self, R):
        self.R_dvl_bluerov = R
        
        yaw_rad = -self.yaw 
        self.R_bluerov_world[0][0] = np.cos(yaw_rad)
        self.R_bluerov_world[0][1] = -np.sin(yaw_rad)
        self.R_bluerov_world[1][0] = np.sin(yaw_rad)
        self.R_bluerov_world[1][1] = np.cos(yaw_rad)

        self.R_dvl_world = self.R_bluerov_world
        
    def is_conn_ok(self):
        return self.conn_ok

    def check_connection(self):
        response = subprocess.run(["ping", "-c", "1", "192.168.2.2"], stdout=subprocess.DEVNULL)

        #and then check the response...
        if response.returncode == 0:
            self.conn_ok = True
        else:
            self.conn_ok = False

        return self.conn_ok

    def start_UDP_reader_thread(self):
        self.UDP_reader_thread_running = True
        self.UDP_reader_thread = threading.Thread(target=self.func_UDP_reader_thread, name = "Bluerov UDP thread", args=())
        self.UDP_reader_thread.daemon = True
        self.UDP_reader_thread.start()
        print("[INFO] BlueROV thread started")

    def stop_UDP_reader_thread(self):
        self.UDP_reader_thread_running = False
        self.UDP_reader_thread.join()
        print("[INFO] BlueROV thread stopped")

    def func_UDP_reader_thread(self):
        self.set_message_interval()
        last_sent = time.time()
        request_interval_interval = 1

        while self.UDP_reader_thread_running:
            try:
                msg = self.connection.recv_match()

                if msg != None:
                    self.parse_msg(msg.to_dict())

                if time.time() - last_sent > request_interval_interval:
                    self.set_message_interval()
                    last_sent = time.time()
   
            except Exception as e:
                print(e)
                print(e.args)

    def set_message_interval(self):
        pass
        # self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, self.frame_rate_IMU, self.connection)
        # self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2, self.frame_rate_Pressure, self.connection)
        # self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, self.frame_rate_AHRS, self.connection)

    def request_message_interval(self, message_id: int, frequency_hz: float, master):
        """
        Request MAVLink message in a desired frequency,
        documentation for SET_MESSAGE_INTERVAL:
            https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

        Args:
            message_id (int): MAVLink message ID
            frequency_hz (float): Desired frequency in Hz
        """
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, # The MAVLink message ID
            1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
            0, 0, 0, 0, # Unused parameters
            0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
        )

    def look_at(self, tilt, roll=0, pan=0):
        """
        Moves gimbal to given position
        Args:
            tilt (float): tilt angle in centidegrees (0 is forward)
            roll (float, optional): pan angle in centidegrees (0 is forward)
            pan  (float, optional): pan angle in centidegrees (0 is forward)
        """
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            1,
            tilt,
            roll,
            pan,
            0, 0, 0,
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)

    def parse_msg(self, msg):
        
        if msg["mavpackettype"] == "RAW_IMU":
            self.accx = msg["xacc"]
            self.accy = msg["yacc"]
            self.accz = msg["zacc"]

            self.gyrox = msg["xgyro"]
            self.gyroy = msg["ygyro"]
            self.gyroz = msg["zgyro"]

            self.magx = msg["xmag"]
            self.magy = msg["ymag"]
            self.magz = msg["zmag"]

            self.IMU_time = time.time()

            with self.lock:
                self.new_imu_message = True
        
        elif msg["mavpackettype"] == "SCALED_PRESSURE2":
            # parse elements into variables
            self.pressure = msg["press_abs"]
            self.depth = -(self.pressure - ATMOSPHERIC_PRESSURE) * MILLIBAR_TO_METER
            self.pressure_time = time.time()

            with self.lock:
                self.new_pressure_message = True

        elif msg["mavpackettype"] == "AHRS2":
            self.roll = msg["roll"]
            self.pitch = msg["pitch"]
            self.yaw = msg["yaw"]

            self.qx, self.qy, self.qz, self.qw = get_quaternion_from_euler(self.roll, self.pitch, self.yaw)
            self.ahrs_time = time.time()

        elif msg["mavpackettype"] == "NAMED_VALUE_FLOAT":
            if msg["name"] == "CamTilt":
                ## mapping from [0,1] to [45,-45], and then convert to rads
                self.camera_pitch = (45 - msg["value"]*90)*np.pi/180

        elif msg["mavpackettype"] == "SYSTEM_TIME":
            self.bluerov_system_time = msg["time_boot_ms"]

    def get_orientation(self):
        return (self.roll, self.pitch, self.yaw)

    def get_pressure(self):
        return self.pressure

    def get_depth(self):
        return self.depth

    def get_acc(self):
        return (self.accx, self.accy, self.accz)

    def get_gyro(self):
        return (self.gyrox, self.gyroy, self.gyroz)

    def get_mag(self):
        return (self.magx, self.magy, self.magz)

    def set_SLAM_LOCAL_NED(self, x, y, z):
        self.SLAM_x = x
        self.SLAM_y = y
        self.SLAM_z = z

    def set_DVL_LOCAL_NED(self, x, y, z, vx, vy, vz, altitude):
        self.DVL_x = x
        self.DVL_y = y
        self.DVL_z = z
        self.DVL_vx = vx
        self.DVL_vy = vy
        self.DVL_vz = vz
        self.DVL_altitude = altitude

    def send_local_ned(self):
        
        x = self.R_dvl_world[0][0]*self.DVL_x + self.R_dvl_world[0][1] * self.DVL_y
        y = self.R_dvl_world[1][0]*self.DVL_x + self.R_dvl_world[1][1] * self.DVL_y
        # print(self.R_dvl_world)

        self.connection.mav.gps_input_send(
        0,  # Timestamp (micros since boot or Unix epoch)
        0,  # ID of the GPS for multiple GPS inputs
        # Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).
        # All other fields must be provided.
        (mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
         mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
         mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY),
        0,  # GPS time (milliseconds from start of GPS week)
        0,  # GPS week number
        3,  # 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
        int((59.9084299 + x*4/110000)*10000000),  # Latitude (WGS84), in degrees * 1E7
        int((10.7194626 + y*4/56000)*10000000),  # Longitude (WGS84), in degrees * 1E7
        0,  # Altitude (AMSL, not WGS84), in m (positive for up)
        1,  # GPS HDOP horizontal dilution of position in m
        1,  # GPS VDOP vertical dilution of position in m
        0,  # GPS velocity in m/s in NORTH direction in earth-fixed NED frame
        0,  # GPS velocity in m/s in EAST direction in earth-fixed NED frame
        0,  # GPS velocity in m/s in DOWN direction in earth-fixed NED frame
        0,  # GPS speed accuracy in m/s
        0,  # GPS horizontal accuracy in m
        0,  # GPS vertical accuracy in m
        7   # Number of satellites visible.
    )

    def send_altitude(self):
        # pass
        if self.DVL_altitude > 0:
            min_measurement = 5 # minimum valid measurement that the autopilot should use
            max_measurement = 500 # maximum valid measurement that the autopilot should use
            distance = self.DVL_altitude # You will need to supply the distance measurement
            sensor_type = mavutil.mavlink.MAV_DISTANCE_SENSOR_UNKNOWN
            sensor_id = 2
            orientation = mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270 # downward facing
            covariance = 0

            self.connection.mav.distance_sensor_send(
                                int((time.time() * 1000) - self.utime_start),
                                min_measurement,
                                max_measurement,
                                int(distance*100),
                                sensor_type,
                                sensor_id,
                                orientation,
                                covariance)
        



    def get_pressure_message(self):
        ## pressure
        pressure_msg = sensor_msgs.msg.FluidPressure()
        pressure_msg.header.frame_id = "bluerov_pressure"
        pressure_msg.header.stamp = rospy.Time.from_sec(self.pressure_time)
        pressure_msg.fluid_pressure = self.pressure

        return pressure_msg

    def get_depth_message(self):
        ## pressure converted to depth
        depth_msg = sensor_msgs.msg.FluidPressure()
        depth_msg.header.frame_id = "bluerov_depth"
        depth_msg.header.stamp = rospy.Time.from_sec(self.pressure_time)
        depth_msg.fluid_pressure = self.depth
        
        with self.lock:
            self.new_pressure_message = False

        return depth_msg
        
    def get_imu_message(self):
        ## acc, gyro and orientation
        imu_msg = sensor_msgs.msg.Imu()
        imu_msg.header.frame_id = "bluerov_imu"
        imu_msg.header.stamp = rospy.Time.from_sec(self.IMU_time)

        ## acceleration
        imu_msg.linear_acceleration.x = self.accx
        imu_msg.linear_acceleration.y = self.accy
        imu_msg.linear_acceleration.z = self.accz

        ## gyro
        imu_msg.angular_velocity.x = self.gyrox
        imu_msg.angular_velocity.y = self.gyroy
        imu_msg.angular_velocity.z = self.gyroz

        ## orientation
        x,y,z,w = get_quaternion_from_euler(self.roll, self.pitch, self.yaw)

        imu_msg.orientation.x = x
        imu_msg.orientation.y = y
        imu_msg.orientation.z = z
        imu_msg.orientation.w = w

        with self.lock:
            self.new_imu_message = False

        return imu_msg
        
    def get_imu_mag_message(self):
        ## mag
        imu_mag_msg = sensor_msgs.msg.MagneticField()
        imu_mag_msg.header.frame_id = "bluerov_imu_mag"
        imu_mag_msg.header.stamp = rospy.Time.from_sec(self.IMU_time)

        imu_mag_msg.magnetic_field.x = self.magx
        imu_mag_msg.magnetic_field.y = self.magy
        imu_mag_msg.magnetic_field.z = self.magz

        

        return imu_mag_msg

    def is_new_pressure_message(self):
        return self.new_pressure_message

    def is_new_imu_message(self):
        return self.new_imu_message
        
        


if __name__ == "__main__":
    
    ## implement test
    # port = 14000 # bluerov
    port = 14550 # sitl
    ROV = BlueROV(port)
    ROV.start_UDP_reader_thread()

    # ROV.set_altitude()
    # for i in range(100):
    #     # ROV.set_local_ned()
    #     time.sleep(0.1)

    print(ROV.get_depth())
    ROV.stop_UDP_reader_thread()



    