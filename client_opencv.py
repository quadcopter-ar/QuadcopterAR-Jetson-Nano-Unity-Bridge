import socket
import cv2
import numpy
import time
import threading

TCP_IP = '10.1.1.124'
# TCP_IP = '192.168.1.101'
TCP_PORT = 5001

sock = socket.socket()

print(TCP_IP)
sock.connect((TCP_IP, TCP_PORT))

capture = cv2.VideoCapture(0)
capture.set(5, 60)
# capture.set(3, 1280)
# capture.set(4, 720)

recv_data = b"NEXTFRAME"

encode_param=[int(cv2.IMWRITE_JPEG_QUALITY), 85]

frame = None
stringData = None
isProcessed = True
isNew = False

def Capture():
    global frame, isProcessed
    while True:
        if isProcessed:
            ret, frame = capture.read()
            isProcessed = False
            time.sleep(1/60)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

def Process():
    global frame, stringData, isProcessed, isNew
    while True:
        if isProcessed == False:
            # frame = cv2.resize(frame, (1280, 720))
            frame = cv2.flip(frame, 0)
            frame = cv2.flip(frame, 1)
            result, imgencode = cv2.imencode('.jpg', frame, encode_param)
            data = numpy.array(imgencode)
            stringData = data.tostring()
            isProcessed = True
            isNew = True
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def Send():
    global stringData, sock, isProcessed, isNew
    while True:
        if isNew:
            sock.send( str(len(stringData)).encode('utf-8').ljust(16))
            sock.send( stringData )
            isNew = False
            recv_data = sock.recv(9)
            print("Size:{}".format(len(stringData)), end='\r')
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
            
    
    sock.close()
    cv2.waitKey(0)
    cv2.destroyAllWindows() 

cap_t = threading.Thread(target=Capture)
cap_t.start()

proc_t = threading.Thread(target=Process)
proc_t.start()

send_t = threading.Thread(target=Send)
send_t.start()

'''
while True:
    # print(recv_data.decode("utf-8"))
    start = time.time()
    ret, frame = capture.read()

    read_time = time.time()-start

    frame = cv2.resize(frame, (1280, 720))
    frame = cv2.flip(frame, 0)
    frame = cv2.flip(frame, 1)
    
    encode_param=[int(cv2.IMWRITE_JPEG_QUALITY), 85]
    result, imgencode = cv2.imencode('.jpg', frame, encode_param)
    data = numpy.array(imgencode)
    stringData = data.tostring()

    proc_time = time.time()-start-read_time

    sock.send( str(len(stringData)).encode('utf-8').ljust(16))


    sock.send( stringData )
    
    # break
    # time.sleep(1/25)
    recv_data = sock.recv(9)

    send_time = time.time()-start-proc_time - read_time

    total_time = time.time()-start

    print("FrameRate: {:.2f}  Size:{} Read Time: {:.2f} Process Time:{:.2f}   Send Time: {:.2f}    ".format(1/total_time, len(stringData), read_time *1000, proc_time * 1000, send_time * 1000), end='\r')

    
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
sock.close()


# decimg=cv2.imdecode(data,1)
# cv2.imshow('CLIENT',decimg)
cv2.waitKey(0)
cv2.destroyAllWindows() 

'''