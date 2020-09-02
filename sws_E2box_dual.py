##########################################################################
import serial
import struct
import glob
import threading
import time
import math
from datetime import datetime
from picamera import PiCamera
from multiprocessing import Process, Queue, Value, Array

##########################################################################
# gui for EBIMU24GV5
##########################################################################
#flag = 0
flag = Array('i', [0,0,0,0])
RPM = 0
dist = 0
cnt = 0
pi = math.pi
data = []
threads = []
###########################################################################
def RCV_IMU(s, i):
    t = []
    global data
    global RPM
    global dist
    cnt = 0
    time_ref = time.time()
    while True:
        # initialize : make .csv and write first row
        if (flag[i] == 0):
            #create csv file
            date = datetime.now().strftime('%y%m%d_%H%M%S')
            file_path = ('/media/pi/728EB4FE8EB4BC43/'+date+'('+str(i)+').csv')
            print("IMU "+str(i)+" start time :" ,date)
            file_ = open(file_path, 'a')
            file_.write("sequence,GyroX,GyroY,GyroZ,AccelX,AccelY,AccelZ,MagnetX,MagnetY,MagnetZ\r\n")
            start = time.time()
            flag[i] = 1

        
        # IMU records data to .csv
        if (flag[i] == 1):
            data = s.read_until( b'UU')
#            print(len(data))
#            continue
            if len(data) == 34:
                data = struct.unpack('>BBhhhhhhhhhhhhhHhh', data)                    
                file_.write("%d, %f, %f, %f, %f, %f, %f, %f, %f, %f\r\n"%(cnt, data[5]/10, data[6]/10, data[7]/10, data[8]/1000, data[9]/1000, data[10]/1000, data[11]/10, data[12]/10, data[13]/10))
            
            cnt = cnt + 1
    
            
        if (time.time() - start)>=300: # 5min 300, 1min 60 #cnt >= 300000: after 5 min (1000Hz receiving)
            print(str(cnt)+ ","+str(cnt/3000) +"%"  + " IMU " + str(i) + " end : ", time.time() - start)
            flag[i] = 2            
        # if record end, initialize variables and return to start
        if (flag[i] == 2):
            print(f'flag[{i}]')


        if(flag[0] == 2 and flag[1] == 2):
            flag[0] = 0
            flag[1] = 0
            cnt = 0

        
def RCV_cam():

    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 40
    flag_cam = 0
    print(flag[0])
    
    try:
        while True:
    #        if flag == 0 :

            if (flag[0] == 1 and flag_cam == 0):
                print("camera start")
                start = time.time()
                camera.start_preview()
                camera.start_recording('/media/pi/728EB4FE8EB4BC43/'+datetime.now().strftime('%y%m%d_%H%M%S')+'.h264')
                flag_cam = 1
                
            if (flag[0] == 2) :
                print("cam end :", time.time() - start)
                camera.stop_recording()
                flag_cam = 0

#                 camera.stop_preview()
#                camera.close()
    except KeyboardInterrupt:
        camera.close()
    
########################### main #################################
# determine USBserial device
port_result = []
ports = glob.glob('/dev/ttyUSB*')

for port in ports:
    print(port)
    try:
        s = serial.Serial(port)
        s.close()
        port_result.append(port)
    except (OSError, serial.SerialException):
        pass


#Connect Serial
ser_ = []
for port in port_result:
    ser_.append(serial.Serial(port, 921600))
#    print(ser_[-1])

# Check all sensor on
while True:
    for s in ser_ :
       if ( s.read(size = 1)) : cnt = cnt + 1
    if cnt == len(ser_): break


for iter in range(len(ser_)):
   threads.append(Process(target = RCV_IMU, args = (ser_[iter], iter)))

threads.append(Process(target = RCV_cam))


print(threads)

for iter in range(len(threads)) :
    threads[iter].start()
# 
for iter in range(len(threads)) :
    threads[iter].join()
