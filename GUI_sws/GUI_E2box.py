7##########################################################################
import serial
import struct
import glob
import threading
import time
from time import sleep
#import cv2
import math
from datetime import datetime

from tkinter import *
import tkinter.font as tkFont
#from PIL import Image, ImageTk
from multiprocessing import Process, Value, Array, Manager
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
def RCV_IMU(s, i, arr_flag, arr_batt, arr_per):
    t = []
#    global flag
    global data
    global RPM
    global dist
    cnt = 0
    pi = math.pi
    Euler = 0
    Euler_ = 0
    timestamp = 0
    timestamp_ = 0
    flag =0

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
#            print(len(data))
            if len(data) == 34:
                data = struct.unpack('>BBhhhhhhhhhhhhhHhh', data)
                '''
                Euler_ = Euler
                timestamp_ = timestamp
                
                Euler = float(data[4] / 100) * pi / 180
                timestamp = time.time()
                '''
                Euler_ = Euler
                timestamp_ = timestamp
                
                Euler = float(data[4] / 100)
                timestamp = time.time()
                '''
                # threshold determine - when Euler(before) goes to 180 > Euler(after) goes to -180 (± pi radian)
                if Euler * Euler_ < -1:
                    t.append(timestamp_)
                '''
                # zero crossing(Falling edge) determine
                if (0 < Euler and Euler < 180) and (-180 < Euler_ and Euler_ < 0):
                    t.append(timestamp_)

                if len(t) == 2 : # time period during one rotation
                    RPM = 60 / (t[1] - t[0])
                    radius_vehicle = 0.365 # unit : meter(m)
                    
                    dist = dist + pi * 2 * radius_vehicle # if wheel rotate once, update driven distance

                    t.remove(t[0])
                
                file_.write("%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d, %.3f\r\n"%(cnt, data[2]/100, data[3]/100, data[4]/100, data[5]/10, data[6]/10, data[7]/10, data[8]/1000, data[9]/1000, data[10]/1000, data[11]/10, data[12]/10, data[13]/10, RPM, time.time() - start))
            cnt = cnt + 1
        
        if time.time() - start >= 300 : # after 5 min (1000Hz receiving)
            flag = 0
            print("IMU " + str(i) + " end count : ", cnt)
            print("IMU " + str(i) + " end time : ", time.time() - start)

            arr_batt[i]=data[14]
            arr_per[i]=cnt/3000
            start = time.time()
            cnt = 0

            arr_flag[i] = 1
            s.flushInput()
        '''
        # if record end, initialize variables and return to start
        if (flag == 2):
            print("IMU " + str(i) + " end : ", time.time() - start)
            flag = 0
            cnt = 0
        ''' 

# if you use usb cam, uncomment this
# problem : videowriter recorded video time does not match with real recording time
'''
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
'''

def GUI(arr_flag, arr_batt, arr_per):
    per = arr_per[:]

    root = Tk()
    root.title('MHE_MeasureWindow')
    root.geometry('720x456+0+0')
    root.resizable(False, False)
    
    background_image=PhotoImage(file = "_bg.png")
    background_label = Label(root, image=background_image)
    background_label.place(x=0, y=0, relwidth=1, relheight=1)
    
    frame_batt = Frame(root, width = 150, background = 'white')
    frame_batt.place(anchor='nw', relx = 0.05, rely = 0.08)
    
    frame_stat = Frame(root, width = 150, background = 'white')
    frame_stat.place(anchor='nw', relx = 0.05, rely = 0.18)    

    fontstyle = tkFont.Font(family = 'Courier', size = 8)

    lbl_batt = []
    var_batt = []
    lbl_var_batt = []

    lbl_stat = []
    var_stat = []
    lbl_var_stat = []    
    
    for iter in range(4):
        lbl_batt.append(Label(frame_batt, text = "IMU%d"%iter, font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 10, background = 'white'))
        lbl_batt[-1].grid(row = 0,column=iter)
   
        var_batt.append(StringVar())
        lbl_var_batt.append(Label(frame_batt, textvariable = var_batt[-1], font = fontstyle, width=10, pady=2, background = 'white'))
        lbl_var_batt[-1].grid(row=1,column=iter)

        lbl_stat.append(Label(frame_stat, text = "IMU%d"%iter, font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 9, background = 'white'))
        lbl_stat[-1].grid(row=iter,column=0)

        var_stat.append(StringVar())
        lbl_var_stat.append(Label(frame_stat, textvariable = var_stat[-1], font = fontstyle, pady=2, background = 'white'))
        lbl_var_stat[-1].grid(row=iter,column=1)
   
    root.update()
    while True :
        for iter in range(4):
            if arr_flag[iter]==1:
                var_batt[iter].set(arr_batt[iter])
                frame_batt.update()
                var_stat[iter].set("received : %.2f"%(arr_per[iter]) + "% | time : " + datetime.now().strftime('%H:%M:%S'))
                frame_stat.update()
                arr_flag[iter]=0

########################### main #################################
# determine USBserial device
port_result = []
ports = glob.glob('/dev/ttyUSB*')

for port in ports:
    port_result.append(port)

#Connect Serial
ser_ = []
for port in port_result:
    ser_.append(serial.Serial(port, 921600))
# Check all sensor on
#while True:
#    for s in ser_ :
#       if ( s.read(size = 1)) : cnt = cnt + 1
#    if cnt == len(ser_): break


flag= Array('i',[0,0,0,0])
batt_Array = Array('d',[0,0,0,0])
percent_Array = Array('d',[0,0,0,0])


for iter in range(len(ser_)):
   threads.append(Process(target = RCV_IMU, args = (ser_[iter], iter, flag,  batt_Array, percent_Array)))
   print(threads[-1])

# if you use usb cam, uncomment this
#threads.append(threading.Thread(target = rcv_cam))

threads.append(Process(target = GUI, args = (flag, batt_Array, percent_Array, )))


for iter in range(len(threads)) :
    threads[iter].start()

for iter in range(len(threads)) :
    threads[iter].join()
