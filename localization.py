import pyrealsense2 as rs
import math as m
import pickle
import cv2

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

def callback_function(notif):
    print(notif)
    if notif.get_category() is rs.notification_category.pose_relocalization:
        print("Relocalization has happened!")
    exit(0)

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)


device = cfg.resolve(pipe).get_device()
pose_sensor = device.first_pose_sensor()
# pose_sensor.set_option(rs.option.enable_map_relocalization, 1)
pose_sensor.set_option(rs.option.enable_map_preservation, 1)
pose_sensor.set_notifications_callback(callback_function)

try:
    with open ('realsense.map', 'rb') as fp:
        map_buf = pickle.load(fp)
        pose_sensor.import_localization_map(map_buf)
        print("Imported the map!")
except IOError as e:
    print("Map file does not exist. Creating a new one.")

# Start streaming with requested config
pipe.start(cfg)




try:
    while(True):
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()

        # Fetch pose frame
        pose = frames.get_pose_frame()
        if pose:
            # Print some of the pose data to the terminal
            data = pose.get_pose_data()

            # Euler angles from pose quaternion
            # See also https://github.com/IntelRealSense/librealsense/issues/5178#issuecomment-549795232
            # and https://github.com/IntelRealSense/librealsense/issues/5178#issuecomment-550217609

            w = data.rotation.w
            x = -data.rotation.z
            y = data.rotation.x
            z = -data.rotation.y

            position = data.translation

            pitch =  -m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi;
            roll  =  m.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / m.pi;
            yaw   =  m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / m.pi;
            
            print("Frame #{}".format(pose.frame_number))
            print("RPY [deg]: Roll: {0:.7f}, Pitch: {1:.7f}, Yaw: {2:.7f}, Position: {3}".format(roll, pitch, yaw, position))



finally:
    pipe.stop()
    map_buf = pose_sensor.export_localization_map()

    with open('realsense.map', 'wb') as fp:
        pickle.dump(map_buf, fp)