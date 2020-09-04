##########################################################################
import serial
import struct
import glob
import threading
import time
from datetime import datetime
from picamera import PiCamera
from multiprocessing import Process, Queue, Value, Array, Lock

from tkinter import *
import tkinter.font as tkFont
from PIL import Image, ImageTk
##########################################################################
# gui for EBIMU24GV5
##########################################################################
#flag = 0

flag = Array('i', [0,0,0,0])
final_list = Array('i', [0,0,0,0,0,0,0,0,0,0])
lock=Lock()
 
# sensor_accel = Array('b', 1)
# sensor_magnet = Array('b', 1)
# sensor_bat = Value('d', 0.0)

cnt = 0
data = []
threads=[]
###########################################################################
def RCV_IMU(s, i):
    t = []
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
            if(i == 0):
                if cnt % 200  == 0 : #every 1 second, update GUI
                    for n, x in enumerate (data[5:15]):
                        final_list[n] = x                 

            if (time.time() - start)>=300: # 5min 300, 1min 60 #cnt >= 300000: after 5 min (1000Hz receiving)
                print(str(cnt)+ ","+f'{(cnt / 3000):.2f}'+"%"  + " IMU " + str(i) + " end : ", time.time() - start)
                flag[i] = 2            
        # if record end, initialize variables and return to start
        if (flag[i] == 2):
#            print(f'flag[{i}]')
            cnt = 0

            if(flag[0] == 2 and flag[1] == 2):
                flag[0] = 0
                flag[1] = 0

        
def RCV_cam():


    flag_cam = 0
    print(flag[0])
    
    try:
        while True:
    #        if flag == 0 :

            if (flag[0] == 1 and flag[1] == 1 and flag_cam == 0):
                print("camera start")
                start = time.time()
#                 camera.start_preview()
                camera = PiCamera()
                camera.resolution = (640, 480)
                camera.framerate = 40
                camera.start_recording('/media/pi/728EB4FE8EB4BC43/'+datetime.now().strftime('%y%m%d_%H%M%S')+'.h264')
                flag_cam = 1
                
            if (flag[0] == 2 and flag[1] == 1 and flag_cam == 1) :
                print("cam end :", time.time() - start)
                camera.stop_recording()
                flag_cam = 0
                camera.close()

#                 camera.stop_preview()
#                camera.close()
    except KeyboardInterrupt:
        camera.close()

def GUI():
    past_value = 0
    root = Tk()
    root.title('MHE_MeasureWindow')
    root.geometry('720x456+0+0')
    root.resizable(False, False)

    background_image=PhotoImage(file = "/home/pi/Documents/sws/GUI_sws/_bg.png")
    background_label = Label(root, image=background_image)
    background_label.place(x=0, y=0, relwidth=1, relheight=1)

    frame = Frame(root, width = 150, background = 'white')
    frame.place(anchor='nw', relx = 0.03, rely = 0.1)

    fontstyle = tkFont.Font(family = 'Courier', size = 12)

  
    lbl_gyro = Label(frame, text = "Gyro(DPS)", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 12, background = 'white')
    lbl_gyro.grid(row = 0, column = 0)
    lbl_accel = Label(frame, text = "Accel(g)", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 12, background = 'white')
    lbl_accel.grid(row = 1, column = 0)
    lbl_magnet = Label(frame, text = "Magnet(uT)", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 12, background = 'white')
    lbl_magnet.grid(row = 2, column = 0)
    lbl_battery = Label(frame, text = "Battery(%)", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 12, background = 'white')
    lbl_battery.grid(row = 3, column = 0)
    #    lbl_distance = Label(frame, text = "distance(km)", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 18, background = 'white')
    #    lbl_distance.grid(row = 5, column = 0)
    #    lbl_RPM = Label(frame, text = "RPM", font = fontstyle, pady=2, relief = 'solid', borderwidth = 1, width = 18, background = 'white')
    #    lbl_RPM.grid(row = 6, column = 0)
    lbl_stat = Label(frame, text = "status", font = fontstyle, pady =2, relief = 'solid', borderwidth = 1, width = 12, background = 'white')
    lbl_stat.grid(row = 4, column = 0)
    
    var = []
    lbl_txt = []
    
    for iter in range(5) :
        var.append(StringVar())
        lbl_txt.append(Label(frame, textvariable = var[-1], relief = 'solid', width = 22, borderwidth = 1, font = fontstyle, pady=2, background = 'white'))
        lbl_txt[iter].grid(row = iter, column = 1)
     
  
#     frame.update()
    # when IMU records data, update GUI window
    while True:
#         if flag[0] == 1 :
  #      if type(data) != type((1,))  : continue
        

        if(past_value == final_list[4]):
            continue
        past_value = final_list[4]
        
        

        var_Gyro = '|'.join([ f'{(x / 10):.1f}' for x in final_list[0:3] ])
        var_Accel = '|'.join([ f'{(x / 1000):.3f}' for x in final_list[3:6] ])
        var_Magnet = '|'.join([ f'{(x / 10):.1f}' for x in final_list[6:9] ])
        var_battery = final_list[9]


# 
        var[0].set(var_Gyro)
        var[1].set(var_Accel)
        var[2].set(var_Magnet)
        var[3].set(var_battery)                
#       var[5].set(f'{dist/1000:.3f}')
#       var[6].set(f'{RPM:.3f}')
        var[4].set("recording...")

        frame.update()
        
            #           sensor_data[0] = '|'.join([ f'{(x / 10):.1f}' for x in data[5:8] ])
    #                 sensor_data[1] = '|'.join([ f'{(x / 1000):.3f}' for x in data[8:11] ])
    #                 sensor_data[2] = '|'.join([ f'{(x / 10):.1f}' for x in data[11:14] ])
        
#         print(var_Gyro, var_Accel, var_Magnet, var_battery)
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

# threads.append(Process(target = RCV_cam))
threads.append(Process(target = GUI))


print(threads)

for iter in range(len(threads)) :
    threads[iter].start()
# 
for iter in range(len(threads)) :
    threads[iter].join()
