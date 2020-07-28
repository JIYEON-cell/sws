##########################################################################
import serial
import struct
import glob
import threading
import time
import cv2

import sys
from bluepy.btle import Scanner, DefaultDelegate
from bluepy.btle import UUID, Peripheral

from time import sleep
from datetime import datetime
##########################################################################
dict_state = {'init':0, 'IMU_on':1, 'cam_on': 2, 'IMU_off':3, 'cam_off':4}
################################################################
# init : initial state - not recording / create record file    #
# IMU_on :  IMU recording start                                #
# cam_on :  if IMU recording start >> cam recording start      #
# IMU_off : if cnt ==15000 (about 3min) >> stop recording IMU  #
# cam_off : if IMU stop recording >> stop recording cam, \     #
#           return to init state                               #
################################################################

target_name = "ELA_106"   # target device name
target_address = None     # target device address
motion_service_uuid = "ef680400-9b35-4933-9b10-52ffa9740042"
motion_char_uuid = "ef680406-9b35-4933-9b10-52ffa9740042"
count = 0
data_flag = 0
threads = []
##########################################################################
class MyDelegate(DefaultDelegate):
    #Constructor (run once on startup)
    def __init__(self, params):
        DefaultDelegate.__init__(self)

    #func is caled on notifications
    def handleNotification(self, cHandle, data):
        print
        motion_data = struct.unpack('hhhhhhhhh',data) # 18byte
        #print(motion_data)
        
        global mAccelerometerX
        global mAccelerometerY
        global mAccelerometerZ
        global mGyroscopeX
        global mGyroscopeY
        global mGyroscopeZ
        global data_flag
        global count
        mAccelerometerX = motion_data[0]
        mAccelerometerY = motion_data[1]
        mAccelerometerZ = motion_data[2]
        mGyroscopeX = motion_data[3]
        mGyroscopeY = motion_data[4]
        mGyroscopeZ = motion_data[5]

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print ("Discovered device", dev.addr)
        elif isNewData:
            print ("Received new data from", dev.addr)
############################################################################
def RCV_IMU(p):
    global data_flag

    count = 0
    file_path = ""
    while True:       
        if p.waitForNotifications(1.0):#handleNotification() called
            if data_flag == dict_state['init']:
                    date = datetime.now().strftime('%Y%m%d_%H%M%S')
                    file_path = ('/home/pi/Documents/sws/'+ date + '(' +p.addr + ')' +'.csv')
                    with open(file_path, 'a') as out_file:
                        print(mAccelerometerX, "|", p.addr)
                        out_file.write("sequence,AccelerometerX,AccelerometerY,AccelerometerZ,GyroscopeX,GyroscopeY,GyroscopeZ\n")
                    print('IMU record start : ' + file_path)
                    data_flag = dict_state['IMU_on']

            if data_flag == dict_state['cam_on']:
                with open(file_path, 'a') as out_file:
                    out_file.write("%d,%d,%d,%d,%d,%d,%d\n"%(count,mAccelerometerX, mAccelerometerY, mAccelerometerZ, mGyroscopeX, mGyroscopeY, mGyroscopeZ))
                count = count + 1
                if count == 15000: 
                    data_flag = dict_state['IMU_off']
        
            if data_flag == dict_state['cam_off']:
                count = 0
                data_flag = dict_state['init']

           
        continue
    print ("Waiting... Waited more than one sec for notification")
    
def RCV_cam():
    global data_flag
    while True:
        if data_flag == dict_state['IMU_on'] : # initialize
            date = datetime.now().strftime('%Y%m%d_%H%M%S')
            cam = cv2.VideoCapture(-1)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter('/home/pi/Documents/sws/'+date+'.mp4',fourcc, 10.0,(640,480))
            start = time.time()
            data_flag = dict_state['cam_on']
            print("cam start : ", date)
            
        if data_flag == dict_state['cam_on']: # recording
            cv2.waitKey(70)
            ret, frame = cam.read()
            out.write(frame)

        if data_flag == dict_state['IMU_off'] : # when IMU recording end
            print("record end :", time.time() - start)
            cam.release()
            out.release()
            data_flag = dict_state['cam_off'] # finish camera record

###########################################################################start
scanner = Scanner()
devices = scanner.scan(6.0)
Peripherals = []

for dev in devices:
    for (adtype, desc, value) in dev.getScanData():
        if target_name == value:
            print(value)
            target_address = dev.addr
            print(dev.addr)
            # create peripheral class
            Peripherals.append(Peripheral(target_address,"random"))
            break

for p in Peripherals :
    #Get MotionService
    MotionService=p.getServiceByUUID(motion_service_uuid)
    p.setDelegate( MyDelegate(p) )
    # Get The Motion-Characteristics
    MotionC = MotionService.getCharacteristics(motion_char_uuid)[0]
    #Get The handle tf the  Button-Characteristics
    hMotionC=MotionC.getHandle()+1
    # Turn notifications on by setting bit0 in the CCC more info on:
    p.writeCharacteristic(hMotionC, struct.pack('<bb', 0x01, 0x00), withResponse=True)
    print ("Notification is turned on for Raw_data")
    t = threading.Thread(target = RCV_IMU, args = (p,))
    threads.append(t)

threads.append(threading.Thread(target = RCV_cam))

for iter in range(len(threads)) :
    threads[iter].start()

for iter in range(len(threads)) :
    threads[iter].join()
