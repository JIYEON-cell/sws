##########################################################################
import serial
import struct
import glob
import threading
from picamera import PiCamera
import time
import cv2
import math
from datetime import datetime
from multiprocessing import Process, Queue, Value, Array

from tkinter import *
import tkinter.font as tkFont
from PIL import Image, ImageTk
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
    flag = 0
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

    date = datetime.now().strftime('%y%m%d_%H%M%S')
    root = Tk() #
    if i == 0:
        camera = PiCamera()
        root.title('IMU - right')
        root.geometry('720x456+360+0')
    else:
        root.title('IMU - left')
        root.geometry('720x456+0+0')
    root.resizable(False, False)

    background_image=PhotoImage(file = "/home/pi/Desktop/sws/GUI_sws/_bg.png")
    background_label = Label(root, image=background_image)
    background_label.place(x=0, y=0, relwidth=1, relheight=1)

    frame = Frame(root, width = 150, background = 'white')
    frame.place(anchor='nw', relx = 0.04, rely = 0.09)

    fontstyle = tkFont.Font(family = 'Courier', size = 10)

    lbl_euler = Label(frame, text = "Euler(RPY)(deg)", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 18, background = 'white')
    lbl_euler.grid(row = 0, column = 0)    
    lbl_gyro = Label(frame, text = "Gyro(XYZ)(DPS)", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 18, background = 'white')
    lbl_gyro.grid(row = 1, column = 0)
    lbl_accel = Label(frame, text = "Accel(XYZ)(g)", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 18, background = 'white')
    lbl_accel.grid(row = 2, column = 0)
    lbl_magnet = Label(frame, text = "Magnet(XYZ)(uT)", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 18, background = 'white')
    lbl_magnet.grid(row = 3, column = 0)
    lbl_battery = Label(frame, text = "Battery(%)", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 18, background = 'white')
    lbl_battery.grid(row = 4, column = 0)
#    lbl_distance = Label(frame, text = "distance(km)", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 18, background = 'white')
#    lbl_distance.grid(row = 5, column = 0)
#    lbl_RPM = Label(frame, text = "RPM", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 18, background = 'white')
#    lbl_RPM.grid(row = 6, column = 0)
    lbl_stat = Label(frame, text = "status", font = fontstyle, pady =2, relief = 'solid', borderwidth = 1, width = 18, background = 'white')
    lbl_stat.grid(row = 6, column = 0)

    var = []
    lbl_txt = []
    
    for iter in range(7) :
        var.append(StringVar())
        lbl_txt.append(Label(frame, textvariable = var[-1], relief = 'solid', width = 22, borderwidth = 1, font = fontstyle, pady=2, background = 'white'))
        lbl_txt[iter].grid(row = iter, column = 1)



    while True:
        # initialize : make .csv and write first row
        if (flag == 0):
            #create csv file
            
           # file_path = ('/home/pi/Desktop/IMU_hex_testing/'+date+'('+str(i)+').csv')
            
            
            file_path = ('/media/pi/869EE7369EE71E05/'+date+'('+str(i)+').csv')
            print("IMU "+str(i)+" start time :" ,date)
            file_ = open(file_path, 'a')
            file_.write("sequence,EulerR,EulerP,EulerY,GyroX,GyroY,GyroZ,AccelX,AccelY,AccelZ,MagnetX,MagnetY,MagnetZ,RPM,time\r\n")
            start = time.time()

            if i == 0 :
                print('picam start')
                camera.resolution = (640, 480)
                camera.framerate = 10
           #     camera.start_preview(fullscreen = False, window = (480, 250,320, 240))
           #     camera.start_recording('/home/pi/Desktop/sws/GUI_sws/'+date+'_cam.h264')
                camera.start_recording('/media/pi/869EE7369EE71E05/'+date+'_cam.h264')

            flag = 1

        # IMU records data to .csv
        if (flag == 1):
            data = s.read_until( b'UU')
            if len(data) == 34:
                data = struct.unpack('>BBhhhhhhhhhhhhhHhh', data)

#                 Euler_ = Euler
#                 timestamp_ = timestamp
#                 
#                 Euler = float(data[4] / 100)
#                 timestamp = time.time()
#                 if (0 < Euler and Euler < 180) and (-180 < Euler_ and Euler_ < 0):
#                     t.append(timestamp_)
#                 if len(t) == 2 : # time period during one rotation
#                     RPM = 60 / (t[1] - t[0])
#                     
#                     radius_vehicle = 0.365 # unit : meter(m)
#                     dist = dist + pi * 2 * radius_vehicle # if wheel rotate once, update driven distance
#                     t.remove(t[0])
                file_.write("%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d, %.3f\r\n"%(cnt, data[2]/100, data[3]/100, data[4]/100, data[5]/10, data[6]/10, data[7]/10, data[8]/1000, data[9]/1000, data[10]/1000, data[11]/10, data[12]/10, data[13]/10, RPM, time.time() - start))

                if cnt % 100 == 0 : #every 1 second, update GUI

                    var_Euler = '|'.join([ f'{(x / 100):.2f}' for x in data[2:5] ])
                    var_Gyro = '|'.join([ f'{(x / 10):.1f}' for x in data[5:8] ])
                    var_Accel = '|'.join([ f'{(x / 1000):.3f}' for x in data[8:11] ])
                    var_Magnet = '|'.join([ f'{(x / 10):.1f}' for x in data[11:14] ])
                    var_battery = data[14]

                    var[0].set(var_Euler)
                    var[1].set(var_Gyro)
                    var[2].set(var_Accel)
                    var[3].set(var_Magnet)
                    var[4].set(var_battery)                
#                    var[5].set(f'{dist/1000:.3f}')
#                    var[6].set(f'{RPM:.3f}')
                    var[6].set("recording...")

                    frame.update()

            cnt = cnt + 1
        
        if time.time() - start >= 300: # after 5 min (1000Hz receiving)
            flag = 0
            print(str(cnt)+","+str(cnt/3000) +"%" + "IMU" +str(i)+"end:", time.time()-start)
            cnt = 0

            if i == 0:
                print('cam end')
                camera.stop_recording()
                #camera.stop_preview()
            start = time.time()

        # if record end, initialize variables and return to start
#        if (flag == 2):
#            print("IMU " + str(i) + " end : ", time.time() - start)
#            flag = 0
#            cnt = 0

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
   threads.append(Process(target = RCV_IMU, args = (ser_[iter], iter)))

# if you use usb cam, uncomment this
#threads.append(threading.Thread(target = RCV_cam))

for iter in range(len(threads)) :
    threads[iter].start()

for iter in range(len(threads)) :
    threads[iter].join()
