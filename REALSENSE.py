import pyzed.sl as sl
import threading
import pyrealsense2 as rs
import math

class RealSense:
    def __init__(self):
        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.yaw = 0
        self.roll = 0
        self.pitch = 0
        self.rotation = [0,0,0]
        self.translation = [0,0,0]
        self.rs_callback = None

    def Start(self):
        self.rs_callback = threading.Thread(target=self.Run)
        self.rs_callback.start()
    
    def Stop(self):
        print("STOPPING")
        if self.rs_callback is not None:
            # self.zed_callback.kill()
            self.rs_callback.join()
            self.rs_callback = None

    def get_translation(self):
        return self.translation
    
    def get_rotation(self):
        return self.rotation
    
    def get_yaw(self):
        return self.yaw

    def get_rotation(self):
        return (self.pitch, self.roll, self.yaw)

    def Run(self):
        self.cfg.enable_stream(rs.stream.pose)
        self.pipe.start(self.cfg)

        while True:
            frames = self.pipe.wait_for_frames()
            pose = frames.get_pose_frame()

            if pose:
                data = pose.get_pose_data()

                q = data.rotation

                rx = 2 * (q.x*q.z + q.w*q.y)
                ry = 2 * (q.y*q.z - q.w*q.x)
                rz = 1 - 2 * (q.x*q.x + q.y*q.y)

                l = math.sqrt(rx*rx + ry*ry + rz*rz)

                rx /= l
                ry /= l
                rz /= l

                rx = -rx
                ry = -ry
                rz = -rz

                # 0.15m
                offset_length = 0.2

                px = data.translation.x - rx * offset_length
                py = data.translation.y - ry * offset_length
                pz = data.translation.z - rz * offset_length - offset_length

                # print("Position: {}".format(data.translation.x))
                # self.translation[0] = data.translation.x
                # self.translation[1] = data.translation.y
                # self.translation[2] = data.translation.z

                self.translation[0] = px
                self.translation[1] = py
                self.translation[2] = pz

                w = q.w
                x = -q.z
                y = q.x
                z = -q.y

                self.yaw = math.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z)

                sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
                cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
                self.pitch = math.atan2(sinr_cosp, cosr_cosp)


                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                self.roll = math.atan2(siny_cosp, cosy_cosp)


