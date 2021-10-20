from DRONE import Drone
from REALSENSE import RealSense
from SERVER import Server
import sys, time, os, threading
from dronekit import connect, VehicleMode
import matplotlib.pyplot as plt
import time
import csv

drone = None
realsense = None
server = None

def main():
    global drone, realsense, server

    realsense = RealSense()
    realsense.Start()

    drone = Drone()
    drone.get_translation = realsense.get_translation
    drone.get_yaw = realsense.get_yaw
    drone.Connect()
    drone.Start()

    server = Server()
    server.get_translation = realsense.get_translation
    server.get_yaw = realsense.get_yaw
    server.get_rotation = realsense.get_rotation
    server.set_target = drone.set_target
    server.Start()

    main_thread = threading.Thread(target=run)
    main_thread.start()

    # run()

def run():
    global realsense, drone

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
    #     print(" Position (NED): {:4f}".format(drone.vehicle.location.local_frame.north, ), end='\r')
    # print(zed.get_translation(), end='\r')

def get_zed_str():
    global realsense, drone
    if realsense.translation is not None and drone.vehicle.location.local_frame.north is not None:
        return(" Position: x(right):{:6.2f}, y(up):{:6.2f}, z(backward):{:6.2f}, Yaw: {:6.2f}".format(drone.translation[0], drone.translation[1], drone.translation[2], realsense.yaw))
    else:
        return(" Position: x(forward):{:6s}, y(left):{:6s}, z(up):{:6s}, Rotation: {:6s}, {:6s}, {:6s}".format("N/A", "N/A", "N/A", "N/A", "N/A", "N/A"))


if __name__ == "__main__":
    main()

