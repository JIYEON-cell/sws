##########################################################################
import serial
import struct
import glob
import threading
import time
import math
from datetime import datetime

from multiprocessing import Process, Queue

##########################################################################
# gui for EBIMU24GV5
##########################################################################
#flag = 0
RPM = 0
dist = 0
cnt = 0
pi = math.pi
data = []
threads = []
###########################################################################
def RCV_IMU(s, i):
    t = []
    flag=0
    global data
    global RPM
    global dist
    cnt = 0
    pi = math.pi
    Euler = 0
    Euler_ = 0
    timestamp = 0
    timestamp_ = 0
    pos_threshold = 0.5 * pi
    neg_threshold = -pos_threshold
    time_ref = time.time()
    while True:
        # initialize : make .csv and write first row
        if (flag == 0):
            #create csv file
            date = datetime.now().strftime('%y%m%d_%H%M%S')
#            file_path = ('/media/pi/8935-96CD/'+date+'('+str(i)+').csv')
            print("IMU "+str(i)+" start time :" ,date)
#            file_ = open(file_path, 'a')
#            file_.write("sequence,EulerR,EulerP,EulerY,GyroX,GyroY,GyroZ,AccelX,AccelY,AccelZ,MagnetX,MagnetY,MagnetZ,RPM,time\r\n")
            start = time.time()
            
            flag = 1

        # IMU records data to .csv
        if (flag == 1):
         #   print(flag)
           
            data = s.read_until( b'UU')

#            print(len(data))
#            continue
            if len(data) == 34:
                data = struct.unpack('>BBhhhhhhhhhhhhhHhh', data)

                Euler_ = Euler
                timestamp_ = timestamp
                
                Euler = float(data[4] / 100)
                timestamp = time.time()
                if (0 < Euler and Euler < 180) and (-180 < Euler_ and Euler_ < 0):
                    t.append(timestamp_)
                if len(t) == 2 : # time period during one rotation
                    RPM = 60 / (t[1] - t[0])
                    
                    radius_vehicle = 0.365 # unit : meter(m)
                    dist = dist + pi * 2 * radius_vehicle # if wheel rotate once, update driven distance
                    t.remove(t[0])
             #   print("%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d, %.3f\r\n"%(cnt, data[2]/100, data[3]/100, data[4]/100, data[5]/10, data[6]/10, data[7]/10, data[8]/1000, data[9]/1000, data[10]/1000, data[11]/10, data[12]/10, data[13]/10, RPM, time.time() - start))
#                file_.write("%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d, %.3f\r\n"%(cnt, data[2]/100, data[3]/100, data[4]/100, data[5]/10, data[6]/10, data[7]/10, data[8]/1000, data[9]/1000, data[10]/1000, data[11]/10, data[12]/10, data[13]/10, RPM, time.time() - start))
            cnt = cnt + 1
        
        if (time.time() - start)>=60: #cnt >= 300000: after 5 min (1000Hz receiving)
            flag = 2
            
        
        # if record end, initialize variables and return to start
        if (flag == 2):
            print(str(cnt) + " IMU " + str(i) + " end : ", time.time() - start)
            flag = 0
            cnt = 0

# if you use usb cam, uncomment this
# problem : videowriter recorded video time does not match with real recording time

    

        
########################### main #################################
# determine USBserial device
port_result = []
ports = glob.glob('/dev/ttyUSB*')

for port in ports:
    try:
        s = serial.Serial(port)
        s.close()
        port_result.append(port)
    except (OSError, serial.SerialException):
        pass

#Connect Serial
ser_ = []
for port in port_result:
    print(port)
    ser_.append(serial.Serial(port, 921600))
    print(ser_[-1])

# Check all sensor on
while True:
    for s in ser_ :
       if ( s.read(size = 1)) : cnt = cnt + 1
    if cnt == len(ser_): break

for iter in range(len(ser_)):
   threads.append(Process(target = RCV_IMU, args = (ser_[iter], iter)))

print(threads[-1])

for iter in range(len(threads)) :
    threads[iter].start()
# 
for iter in range(len(threads)) :
    threads[iter].join()

