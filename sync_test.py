from DRONE import Drone
from REALSENSE import RealSense
from SYNCSERVER import Server
import sys, time, os, threading
from dronekit import connect, VehicleMode
import matplotlib.pyplot as plt
import time
import csv

realsense = None
server = None

def set_target_dummie(t):
    pass

def main():
    global realsense, server

    realsense = RealSense()
    realsense.Start()

    server = Server()
    server.get_translation = realsense.get_translation
    server.get_yaw = realsense.get_yaw
    server.get_rotation = realsense.get_rotation
    server.set_target = set_target_dummie
    server.Start()

    main_thread = threading.Thread(target=run)
    main_thread.start()

def run():
    while True:
        time.sleep(1/10)

if __name__ == "__main__":
    main()