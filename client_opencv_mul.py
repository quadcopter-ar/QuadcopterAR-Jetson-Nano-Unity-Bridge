import socket
import cv2
import numpy
import time
import threading

TCP_IP = '10.1.1.124'
# TCP_IP = '192.168.1.101'
TCP_PORT = 5001

tcp_thread_count = 5

socks = []

for i in range(tcp_thread_count):
    sock = socket.socket()
    socks.append(sock)
    socks[i].connect((TCP_IP, TCP_PORT+i))

capture = cv2.VideoCapture(0)

# set camera parameters
capture.set(5, 60)
# capture.set(3, 2560)
# capture.set(4, 720)

recv_data = b"NEXTFRAME"

loop_count = 0
curr_sock = 1

encode_param=[int(cv2.IMWRITE_JPEG_QUALITY), 85]

while True:
    if loop_count > tcp_thread_count - 1:
        recv_data = socks[loop_count % tcp_thread_count].recv(9)
    loop_count += 1


    # print(recv_data.decode("utf-8"))
    start = time.time()
    ret, frame = capture.read()

    read_time = time.time()-start

    # frame = cv2.resize(frame, (1280, 360))
    frame = cv2.flip(frame, 0)
    frame = cv2.flip(frame, 1)
    #cv2.imshow('CLIENT',frame)
    
    result, imgencode = cv2.imencode('.jpg', frame, encode_param)
    # data = numpy.array(frame)
    data = numpy.array(imgencode)
    stringData = data.tostring()

    proc_time = time.time()-start-read_time

    # print("Length="+ str(len(stringData)))
    # print(str(len(stringData)).encode('ascii').ljust(16))

    socks[loop_count % tcp_thread_count].send( str(len(stringData)).encode('utf-8').ljust(16))
    socks[loop_count % tcp_thread_count].send( stringData )
    # print("SOCK" + str(loop_count % tcp_thread_count) + " sent")

    send_time = time.time()-start-proc_time - read_time

    total_time = time.time()-start

    print("FrameRate: {:.2f}  Size:{} Read Time: {:.2f} Process Time:{:.2f}   Send Time: {:.2f}    ".format(1/total_time, len(stringData), read_time *1000, proc_time * 1000, send_time * 1000), end='\r')
    # print(stringData)

    # break
    time.sleep(1/60)

    

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
sock.close()

# decimg=cv2.imdecode(data,1)
# cv2.imshow('CLIENT',decimg)
cv2.waitKey(0)
cv2.destroyAllWindows() 
