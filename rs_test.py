from DRONE import Drone
from REALSENSE import RealSense
import sys, time, os, threading
from dronekit import connect, VehicleMode
import matplotlib.pyplot as plt
import time
import csv

drone = None
realsense = None

def main():
    global drone, realsense

    realsense = RealSense()
    realsense.Start()

    drone = Drone()
    drone.get_translation = realsense.get_translation
    drone.get_yaw = realsense.get_yaw
    drone.Connect()
    drone.Start()

    main_thread = threading.Thread(target=run)
    main_thread.start()

def run():
    global realsense, drone
    zed_pos_record = []
    drone_pos_record = []
    times = []
    count = 0
    start_time = time.time()
    drone_log = open("drone_log.csv", "w")
    zed_log = open("zed_log.csv", "w")
    
    drone_writer = csv.writer(drone_log, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    zed_writer = csv.writer(zed_log, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    while True:
        try:
            print(get_time_str() + get_drone_str() + get_zed_str(), end='\r')

        except KeyboardInterrupt:
            print("\nEXIT")
            os._exit(0)
        except:
            raise
        time.sleep(1/10)

def get_time_str():
    curr_time = int(round(time.time() * 100))
    return " Time: [{:6d}]".format(curr_time)

def get_drone_str():
    global drone
    if drone.vehicle.location.local_frame.north is not None:
        return(" Position (SOLO): [x(forward){:6.2f}, y(left){:6.2f}, z(up){:6.2f}] Yaw (SOLO): {:6.2f}".format(drone.vehicle.location.local_frame.north,-drone.vehicle.location.local_frame.east,-drone.vehicle.location.local_frame.down, drone.vehicle.attitude.yaw))
    else:
        return(" Position (SOLO): [x(forward){:6s}, y(left){:6s}, z(up){:6s}]".format("N/A", "N/A", "N/A"))

def get_zed_str():
    global realsense, drone
    if realsense.translation is not None and drone.vehicle.location.local_frame.north is not None:
        return(" Position: x(right):{:6.2f}, y(up):{:6.2f}, z(backward):{:6.2f}, Yaw: {:6.2f}".format(drone.translation[0], drone.translation[1], drone.translation[2], realsense.yaw))
    else:
        return(" Position: x(forward):{:6s}, y(left):{:6s}, z(up):{:6s}, Rotation: {:6s}, {:6s}, {:6s}".format("N/A", "N/A", "N/A", "N/A", "N/A", "N/A"))

if __name__ == "__main__":
    main()