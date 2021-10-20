from dronekit import connect, VehicleMode
import time, threading
import numpy as np
import transformations as tf
from pymavlink import mavutil

class Drone:
    def __init__(self):
        self.vehicle = None
        self.get_translation = None
        self.translation = None
        self.get_yaw = None
        self.target_translation = [0, 0, 0]
        self.target_yaw = 0
        self.status = 0
        self.H_T265Ref_T265body = tf.quaternion_matrix([1, 0, 0, 0]) # H_T265Ref_T265body = tf.quaternion_matrix([data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z])
        self.H_T265Ref_T265body[0][3] = 0 # data.translation.x * scale_factor
        self.H_T265Ref_T265body[1][3] = 0 # data.translation.y * scale_factor
        self.H_T265Ref_T265body[2][3] = 0 # data.translation.z * scale_factor
        self.H_aeroRef_T265Ref  = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
        self.H_T265body_aeroBody = np.linalg.inv(self.H_aeroRef_T265Ref)
        self.H_aeroRef_aeroBody = self.H_aeroRef_T265Ref.dot( self.H_T265Ref_T265body.dot( self.H_T265body_aeroBody))
        self.linear_accel_cov = 0.01
        self.angular_vel_cov  = 0.01
        self.home_lat = 385351800
        self.home_lon = -1217531650
        self.home_alt = 200000
    
    def Connect(self):
        self.vehicle = connect('0.0.0.0:14550', wait_ready = False, baud = 921600, source_system = 1)
    
    def set_default_global_origin(self):
        msg = self.vehicle.message_factory.set_gps_global_origin_encode(
            int(self.vehicle._master.source_system),
            self.home_lat, 
            self.home_lon,
            self.home_alt
        )

        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        print("set_default_global_origin")

    def set_default_home_position(self):
        x = 0
        y = 0
        z = 0
        q = [1, 0, 0, 0]   # w x y z

        approach_x = 0
        approach_y = 0
        approach_z = 1

        msg = self.vehicle.message_factory.set_home_position_encode(
            int(self.vehicle._master.source_system),
            self.home_lat, 
            self.home_lon,
            self.home_alt,
            x,
            y,
            z,
            q,
            approach_x,
            approach_y,
            approach_z
        )

        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        print("set_default_home_position")
    
    def send_vision_position_estimate_message(self):
        rpy_rad = np.array( tf.euler_from_matrix(self.H_aeroRef_aeroBody, 'sxyz'))

        cov_pose    = self.linear_accel_cov * pow(10, 3 - 1)
        cov_twist   = self.angular_vel_cov  * pow(10, 1 - 1)
        covariance  = np.array([cov_pose, 0, 0, 0, 0, 0,
                                    cov_pose, 0, 0, 0, 0,
                                        cov_pose, 0, 0, 0,
                                        cov_twist, 0, 0,
                                            cov_twist, 0,
                                                cov_twist])

        # q = [self.data.rotation.x,self.data.rotation.y,self.data.rotation.z,self.data.rotation.w]
        # (roll, yaw, pitch) = tf.euler_from_quaternion(q)
        # if pitch > 0:
        #     pitch -= math.pi
        # else:
        #     pitch += math.pi
        # print([round(roll, 2),round(yaw, 2),round(pitch, 2)], end='\r')

        # Setup the message to be sent
        msg = self.vehicle.message_factory.vision_position_estimate_encode(
        # msg = vehicle.message_factory.att_pos_mocap_encode(
            self.current_time_us,            # us Timestamp (UNIX time or time since system boot)
            # H_aeroRef_aeroBody[0][3],   # Global X position
            # H_aeroRef_aeroBody[1][3],   # Global Y position
            # H_aeroRef_aeroBody[2][3],   # Global Z position
            # float(-self.translation[2]), # north
            # float(self.translation[0]), # east
            # float(-self.translation[1]), # down
            float(-self.translation[2]), # north
            float(self.translation[0]), # east
            float(-self.translation[1]), # down
            # 0.0,
            # 0.0,
            # 0.0,
            # rpy_rad[0],	                # Roll angle
            # rpy_rad[1],	                # Pitch angle
            # rpy_rad[2],	                # Yaw angle
            # -roll,
            # pitch,
            # yaw
            0.0,
            0.0,
            float(self.yaw),
            # covariance,                 # Row-major representation of pose 6x6 cross-covariance matrix
            # reset_counter               # Estimate reset counter. Increment every time pose estimate jumps.
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def Start(self):
        self.set_default_global_origin()
        self.set_default_home_position()
        self.drone_callback = threading.Thread(target=self.Run)
        self.drone_callback.start()

    def Run(self):
        while True:
            try:
                self.current_time_us = int(round(time.time() * 1000000))
                self.translation = self.get_translation()
                self.yaw = self.get_yaw()
                # self.translation = [0, 0, 0]
                self.send_vision_position_estimate_message()
            except KeyboardInterrupt:
                print("\nEXIT")
                os._exit(0)
            except:
                raise
            time.sleep(1/30)

    def arm_and_takeoff(self, altitude):
        print(self.vehicle.is_armable)
        self.vehicle.mode = VehicleMode("STABILIZE")
        self.vehicle.armed = True
        print("ARMED")
        while not self.vehicle.armed:
            time.sleep(1)
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.simple_takeoff(altitude)
        print("TAKEOFF")
        time.sleep(3)
        self.condition_yaw(0)
        self.goto_local(0, 0, -1)
        time.sleep(5)
        while self.status == 1:
            time.sleep(1)
            self.condition_yaw(self.target_yaw)
            self.goto_local(-self.target_translation[2], self.target_translation[0], -self.target_translation[1]) # north=z east=x down=-y


    def land(self):
        self.vehicle.mode = VehicleMode("LAND")
        print("LAND")
        self.vehicle.armed = False
        print("DISARMED")

    def goto_local(self, north, east, down):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,
            north, east, down,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        self.vehicle.send_mavlink(msg)

    def condition_yaw(self, yaw):
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            yaw,   # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            0, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def set_target(self, target):
        # print(target)
        self.target_translation = [target['x'], target['y'], target['z']]
        # print(self.target_translation)
        self.target_yaw = target['yaw']
        if self.status != target['status']:
            self.status = target['status']
            if self.status == 1:
                cmd_thread = threading.Thread(target=self.arm_and_takeoff, args=(.5, ))
                cmd_thread.start()
            elif self.status == 0:
                cmd_thread = threading.Thread(target=self.land)
                cmd_thread.start()
                
            print("STATUS CHANGE")
        
