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
RPM = 0
dist = 0
cnt = 0
pi = math.pi
data = []
threads = []
###########################################################################
def RCV_IMU(s, i):
    t = []
    global flag
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
            file_path = ('/home/pi/Desktop/IMU_hex_testing/'+date+'('+str(i)+').csv')
            print("IMU "+str(i)+" start time :" ,date)
            file_ = open(file_path, 'a')
            file_.write("sequence,EulerR,EulerP,EulerY,GyroX,GyroY,GyroZ,AccelX,AccelY,AccelZ,MagnetX,MagnetY,MagnetZ,RPM,time\r\n")
            start = time.time()
            
            flag = 1

        # IMU records data to .csv
        if (flag == 1):
            data = s.read_until( b'UU')
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
                file_.write("%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d, %.3f\r\n"%(cnt, data[2]/100, data[3]/100, data[4]/100, data[5]/10, data[6]/10, data[7]/10, data[8]/1000, data[9]/1000, data[10]/1000, data[11]/10, data[12]/10, data[13]/10, RPM, time.time() - start))
            cnt = cnt + 1
        
        if cnt >= 300000: # after 5 min (1000Hz receiving)
            flag = 2
        
        # if record end, initialize variables and return to start
        if (flag == 2):
            print("IMU " + str(i) + " end : ", time.time() - start)
            flag = 0
            cnt = 0

# if you use usb cam, uncomment this
# problem : videowriter recorded video time does not match with real recording time

def RCV_cam():
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
            frame = cv2.flip(frame, 0)
            cv2.imshow("frame", frame)
            out.write(frame)

        if (flag == 2) :
            print("cam end :", time.time() - start)
            cap.release()
            out.release()
            flag = 3
#            delay = delay + 5
    
def GUI():
    global flag
    cnt = 0
    time_ref = time.time()

    root = Tk() #
    root.title('MHE_MeasureWindow')
    root.geometry('720x456+0+0')
    root.resizable(False, False)

    background_image=PhotoImage(file = "/home/pi/Desktop/sws/GUI_sws/_bg.png")
    background_label = Label(root, image=background_image)
    background_label.place(x=0, y=0, relwidth=1, relheight=1)

    frame = Frame(root, width = 150, background = 'white')
    frame.place(anchor='nw', relx = 0.04, rely = 0.09)

    fontstyle = tkFont.Font(family = 'Courier', size = 10)

    lbl_euler = Label(frame, text = "Euler(RPY)", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 12, background = 'white')
    lbl_euler.grid(row = 0, column = 0)    
    lbl_gyro = Label(frame, text = "Gyro(XYZ)", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 12, background = 'white')
    lbl_gyro.grid(row = 1, column = 0)
    lbl_accel = Label(frame, text = "Accel(XYZ)", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 12, background = 'white')
    lbl_accel.grid(row = 2, column = 0)
    lbl_magnet = Label(frame, text = "Magnet(XYZ)", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 12, background = 'white')
    lbl_magnet.grid(row = 3, column = 0)
    lbl_battery = Label(frame, text = "Battery", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 12, background = 'white')
    lbl_battery.grid(row = 4, column = 0)
    lbl_distance = Label(frame, text = "distance", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 12, background = 'white')
    lbl_distance.grid(row = 5, column = 0)
    lbl_RPM = Label(frame, text = "RPM", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 12, background = 'white')
    lbl_RPM.grid(row = 6, column = 0)
    lbl_stat = Label(frame, text = "status", font = fontstyle, pady =2, relief = 'solid', borderwidth = 1, width = 12, background = 'white')
    lbl_stat.grid(row = 7, column = 0)
    '''
    lbl_Dist = Label(frame, text = "distance", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 10, background = 'white')
    lbl_Dist.grid(row = 0, column = 0)
    lbl_RPM = Label(frame, text = "RPM", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 10, background = 'white')
    lbl_RPM.grid(row = 0, column = 1)
    
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

    lbl_bat = Label(frame, text = "battery", font = fontstyle, pady = 2, relief = 'solid', borderwidth = 1, width = 10, background = 'white')
    lbl_bat.grid(row = 6, column = 0)    
    lbl_stat = Label(frame, text = "status", font = fontstyle, pady =2, relief = 'solid', borderwidth = 1, width = 10, background = 'white')
    lbl_stat.grid(row = 6, column = 1)
    '''

    var = []
    lbl_txt = []
    
    for iter in range(8) :
        var.append(StringVar())
        lbl_txt.append(Label(frame, textvariable = var[-1], relief = 'solid', width = 30, borderwidth = 1, font = fontstyle, pady=2, background = 'white'))
        lbl_txt[iter].grid(row = iter, column = 1)

    while True:
        if flag == 0 :
            var[10].set("initializing...")
            frame.update()

    # when IMU records data, update GUI window
        if type(data) != type((1,)) : continue # update GUI by second
        var[5].set('%.3f'%dist)
        lbl_txt[5].update()
        var[6].set('%.3f'%RPM)
        lbl_txt[6].update()
        data_ = data[:]
        Euler = data[4] / 100
        var[0].set(Euler)
        lbl_txt[0].update()

        '''
        Euler = [ x / 100 for x in data[2:5] ]
        Gyro = [ x / 10 for x in data[5:8] ]
        Accel = [ x / 1000 for x in data[8:11] ]
        Magnet = [ x / 10 for x in data[11:14] ]
        battery = data[14]

        var[0].set(Euler)
        var[1].set(Gyro)
        var[2].set(Accel)
        var[3].set(Magnet)
        var[4].set(battery)
        '''
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
    ser_.append(serial.Serial(port, 921600))

# Check all sensor on
while True:
    for s in ser_ :
       if ( s.read(size = 1)) : cnt = cnt + 1
    if cnt == len(ser_): break

for iter in range(len(ser_)):
   threads.append(threading.Thread(target = RCV_IMU, args = (ser_[iter], iter)))

# if you use usb cam, uncomment this
#threads.append(threading.Thread(target = RCV_cam))

threads.append(threading.Thread(target = GUI))

for iter in range(len(threads)) :
    threads[iter].start()

for iter in range(len(threads)) :
    threads[iter].join()
