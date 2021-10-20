import pyrealsense2 as rs
import math

pipe = rs.pipeline()
cfg = rs.config()

cfg.enable_stream(rs.stream.pose)
pipe.start(cfg)

while True:
    frames = pipe.wait_for_frames()
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

        # 0.2m
        offset_length = 0.14

        px = data.translation.x - rx * offset_length
        py = data.translation.y - ry * offset_length
        pz = data.translation.z - rz * offset_length - offset_length

        # print("position: {:.2f} {:.2f} {:.2f} rotation: {:.2f} {:.2f} {:.2f}".format(px, py, pz, rx, ry, rz), end='\r')

        w = q.w
        x = -q.z
        y = q.x
        z = -q.y

        yaw   =  math.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / math.pi


        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        pitch = math.atan2(sinr_cosp, cosr_cosp)  * 180.0 / math.pi


        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        roll = math.atan2(siny_cosp, cosy_cosp)  * 180.0 / math.pi

        print(roll, end='\r')