from REALSENSE import RealSense
from RealsenseSocket import Server
import sys, time, os, threading
import time
import streaming_sender

realsense = None
server = None

def main():
    global drone, realsense, server

    realsense = RealSense()
    realsense.Start()

    server = Server()
    server.get_translation = realsense.get_translation
    server.get_yaw = realsense.get_yaw
    server.get_rotation = realsense.get_rotation
    server.Start()

    first_thread = threading.Thread(target=run)
    second_thread = threading.Thread(target=streaming_sender.main)
    first_thread.start()
    second_thread.start()

    # run()

def run():
    global realsense, drone

    while True:
        try:
            print(get_time_str() + get_realsense_str(), end='\r')
        except KeyboardInterrupt:
            print("\nEXIT")
            os._exit(0)
        except:
            raise
        time.sleep(1/10)

def get_time_str():
    curr_time = int(round(time.time() * 100))
    return " Time: [{:6d}]".format(curr_time)

def get_realsense_str():
    global realsense
    if realsense.translation is not None:
        return(" Position: x(right):{:6.2f}, y(up):{:6.2f}, z(backward):{:6.2f}, Yaw: {:6.2f}".format(realsense.translation[0], realsense.translation[1], realsense.translation[2], realsense.yaw))
    else:
        return(" Position: x(forward):{:6s}, y(left):{:6s}, z(up):{:6s}, Rotation: {:6s}, {:6s}, {:6s}".format("N/A", "N/A", "N/A", "N/A", "N/A", "N/A"))


if __name__ == "__main__":
    main()

