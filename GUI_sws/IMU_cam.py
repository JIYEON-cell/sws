##########################################################################
import serial
import struct
import glob
import threading
import time
import cv2
import math
from datetime import datetime

from tkinter import *
import tkinter.font as tkFont
from PIL import Image, ImageTk
##########################################################################
# gui for EBIMU24GV5
##########################################################################
flag = 0
pi = math.pi
data = []
RPM = 0
dist = 0
def RCV_IMU(s, i):
    t = []
    global flag
    global data
    global RPM
    global dist
    cnt = 0
    pi = math.pi
    while True:
        if (flag == 0):
    #########################################create csv file
            date = datetime.now().strftime('%y%m%d_%H%M%S')
            file_path = ('/home/pi/Desktop/IMU_hex_testing/'+date+'('+str(i)+').csv')
            print("IMU "+str(i)+" start time :" ,date)
            file_ = open(file_path, 'a')
            file_.write("sequence,EulerR,EulerP,EulerY,GyroX,GyroY,GyroZ,AccelX,AccelY,AccelZ,MagnetX,MagnetY,MagnetZ,RPM\r\n")
            start = time.time()
            
            flag = 1
            
            while True:
                data = s.read_until( b'UU')
                if len(data) == 32:
                    data = struct.unpack('>BBhhhhhhhhhhhhHhh', data)

                    Euler = float(data[4] / 100) * pi / 180
                    timestamp = time.time()
                    time_ref = timestamp
                    
                    break

        if (flag == 1):
            data = s.read_until( b'UU')
            if len(data) == 32:
                data = struct.unpack('>BBhhhhhhhhhhhhHhh', data)

                Euler_ = Euler
                timestamp_ = timestamp
                
                Euler = float(data[4] / 100) * pi / 180
                timestamp = time.time()
                
                
                if Euler * Euler_ < -1:
                    t.append(timestamp_)
                '''
                if Euler * Euler_ < -8.8 :
                    Tp = (timestamp_ - time_ref) + (pi - Euler) * (timestamp - timestamp_) / (Euler_ + Euler)
                    RPM = 60 / Tp
                    time_ref = timestamp
                '''
                if len(t) == 2 : # time period during one rotation
                    RPM = 60 / (t[1] - t[0])
                    
                    dist = dist + pi*2*0.305
                    t.remove(t[0])
                
                '''
                diff = Euler - Euler_
                if diff == 0.0 : continue

                Tp = timestamp + (((pi - Euler) * (timestamp - timestamp_)) / (diff))
                RPM = 60 / Tp

                print(RPM)
                '''
                
                file_.write("%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d\r\n"%(cnt, data[2]/100, data[3]/100, data[4]/100, data[5]/10, data[6]/10, data[7]/10, data[8]/1000, data[9]/1000, data[10]/1000, data[11]/10, data[12]/10, data[13]/10, RPM))
            cnt = cnt + 1
        
        if cnt >= 300000:
            flag = 3
        
        if (flag == 3):
            print("IMU " + str(i) + " end : ", time.time() - start)
            flag = 0
            cnt = 0

        

def RCV_cap():
    global flag
    fps = 10
    while True:
        if flag == 0 :
            date = datetime.now().strftime('%y%m%d_%H%M%S')
            cap = cv2.VideoCapture(-1)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter('/home/pi/Desktop/IMU_hex_testing/'+date+'.mp4',fourcc,fps,(640,480))
            start = time.time()
            print("cam start : ", date)
        if flag == 1:
            cv2.waitKey(83)
            ret, frame = cap.read()
            out.write(frame)

        if (flag == 2) :
            print("cam end :", time.time() - start)
            cap.release()
            out.release()
            flag = 3
#            delay = delay + 5
            
def GUI():
    global data_flag
    global count
    root = Tk() #
    root.title('MHE_MeasureWindow')
    root.geometry('720x456+0+0')
    root.resizable(False, False)

    background_image=PhotoImage(file = "_bg.png")
    background_label = Label(root, image=background_image)
    background_label.place(x=0, y=0, relwidth=1, relheight=1)

    frame = Frame(root, width = 150, background = 'white')
    frame.place(anchor='nw', relx = 0.05, rely = 0.08)

    fontstyle = tkFont.Font(family = 'Courier', size = 12)

    lbl_accelx = Label(frame, text = "Accel_X", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 10, background = 'white')
    lbl_accelx.grid(row = 2, column = 0)
    lbl_accely = Label(frame, text = "Accel_Y", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 10, background = 'white')
    lbl_accely.grid(row = 2, column = 1)
    lbl_accelz = Label(frame, text = "Accel_Z", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 10, background = 'white')
    lbl_accelz.grid(row = 2, column = 2)
    lbl_gyrox = Label(frame, text = "Gyro_X", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 10, background = 'white')
    lbl_gyrox.grid(row = 4, column = 0)
    lbl_gyroy = Label(frame, text = "Gyro_Y", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 10, background = 'white')
    lbl_gyroy.grid(row = 4, column = 1)
    lbl_gyroz = Label(frame, text = "Gyro_Z", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 10, background = 'white')
    lbl_gyroz.grid(row = 4, column = 2)
    lbl_rotcnt = Label(frame, text = "distance", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 10, background = 'white')
    lbl_rotcnt.grid(row = 0, column = 0)
    lbl_Dist = Label(frame, text = "RPM", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 10, background = 'white')
    lbl_Dist.grid(row = 0, column = 1)

    var = []
    lbl_txt = []
    
    for iter in range(8) :
        var.append(StringVar())
        lbl_txt.append(Label(frame, textvariable = var[-1], font = fontstyle, pady=2, background = 'white'))
    
    lbl_txt[0].grid(row = 3, column = 0)
    lbl_txt[1].grid(row = 3, column = 1)
    lbl_txt[2].grid(row = 3, column = 2)
    lbl_txt[3].grid(row = 5, column = 0)
    lbl_txt[4].grid(row = 5, column = 1)
    lbl_txt[5].grid(row = 5, column = 2)
    lbl_txt[6].grid(row = 1, column = 0)
    lbl_txt[7].grid(row = 1, column = 1)
    '''
    var list : 0 ~ 10
    accelx
    accely
    accelz
    gyrox
    gyroy
    gyroz
    velx
    vely
    velz
    rot.count
    dist

    '''
    while flag == 1 :
        if type(data) != type((1,)) or len(data) != 17  : continue

        var[0].set(data[8]/1000)
        var[1].set(data[9]/1000)
        var[2].set(data[10]/1000)

        var[3].set(data[5]/10)
        var[4].set(data[6]/10)
        var[5].set(data[7]/10)
        var[6].set(round(dist, 3))
        var[7].set(int(RPM))

        frame.update()
###########################################################################start
threads = []
cnt = 0

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
    ser_.append(serial.Serial(port, 921600))

# Check all sensor on
while True:
    for s in ser_ :
       if ( s.read(size = 1)) : cnt = cnt + 1
    if cnt == len(ser_): break
    

for iter in range(len(ser_)):
   threads.append(threading.Thread(target = RCV_IMU, args = (ser_[iter], iter)))

threads.append(threading.Thread(target = GUI))

for iter in range(len(threads)) :
    threads[iter].start()

for iter in range(len(threads)) :
    threads[iter].join()
