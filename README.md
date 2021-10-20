# Drone-Jetson-Unity Interface
The files that run on the Jetson Nano, coupled with the drone, to interface with Unity

## Process and Flow
- Drone connects to Drone Controller
- Drone controller turns on HotSpot (SoloLink)
- Jetson Nano connects to HotSpot
- PC Connects to HotSpot
- Unity now recognizes the IP address of the Jetson Nano
- There is a camera connected to the Jetson Nano
- Camera sends feed to Jetson Nano, who then sends the Feed to Unity over a websocket and a TCP connection
- When the drone controller moves the quadcopter, the realsense captures this frame and sends the movement to Unity
- Unity then translates the currentPosition of the player by that delta.
