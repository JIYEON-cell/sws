import struct
import sys
from bluepy.btle import Scanner, DefaultDelegate
from bluepy.btle import UUID, Peripheral
from time import sleep
from datetime import datetime

target_name = "ELA_106"   # target device name
target_address = None
motion_service_uuid = "ef680400-9b35-4933-9b10-52ffa9740042"
motion_char_uuid = "ef680406-9b35-4933-9b10-52ffa9740042"
count = 0

class MyDelegate(DefaultDelegate):
    #Constructor (run once on startup)

    def __init__(self, params):
        DefaultDelegate.__init__(self)

    #func is caled on notifications
    def handleNotification(self, cHandle, data):
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
        data_flag = True
        count += 1
        mAccelerometerX = motion_data[0]
        mAccelerometerY = motion_data[1]
        mAccelerometerZ = motion_data[2]
        mGyroscopeX = motion_data[3]
        mGyroscopeY = motion_data[4]
        mGyroscopeZ = motion_data[5]
        
        if count == 15000: # Every 5min
            count = 0
            data_flag = False

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print ("Discovered device", dev.addr)
        elif isNewData:
            print ("Received new data from", dev.addr)

def reconn_device(addr) :
    scanner = Scanner()
    print("Finding disconnected device... : ", addr)
    devices = scanner.scan(6.0)
    for dev in devices :
        if addr == dev.addr:
            return Peripheral(dev.addr,"random")


with open("./conn_dev", 'r') as f:
    conn_dev = f.readlines()
print(conn_dev)

scanner = Scanner()
devices = scanner.scan(6.0)
Peripherals = []

for dev in devices:
    for (adtype, desc, value) in dev.getScanData():
        if target_name == value:
            print(value)
            target_address = dev.addr
            if target_address in conn_dev :
                print(target_address)
            else :
                with open("./conn_dev", 'a') as f:
                    f.write(target_address)
            Peripherals.append(Peripheral(target_address, "random"))
            break

for p in Peripherals:
    #Get MotionService
    MotionService=p.getServiceByUUID(motion_service_uuid)
    p.setDelegate( MyDelegate(p) )
    # Get The Motion-Characteristics
    MotionC = MotionService.getCharacteristics(motion_char_uuid)[0]
    #Get The handle tf the  Button-Characteristics
    hMotionC=MotionC.getHandle()+1
    # Turn notifications on by setting bit0 in the CCC more info on:
    p.writeCharacteristic(hMotionC, struct.pack('<bb', 0x01, 0x00), withResponse=True)
    print ("Notification is turned on for Raw_data : ", p.addr)

while True:
    for p in Peripherals:
        try:
            if p.waitForNotifications(1.0):
                #handleNotification() called
                if data_flag == True:
                    print(count)
                    continue
        except :
            print("Waiting... Waited more than one sec for notification : ", p.addr)
            p_reconn = reconn_device(p.addr)
            Peripherals[Peripherals.index(p)] = p_reconn
            #Get MotionService
            MotionService = p.disconnect()
            MotionService=p_reconn.getServiceByUUID(motion_service_uuid)
            p_reconn.setDelegate( MyDelegate(p_reconn) )
            # Get The Motion-Characteristics
            MotionC = MotionService.getCharacteristics(motion_char_uuid)[0]
            #Get The handle tf the  Button-Characteristics
            hMotionC = MotionC.getHandle() + 1
            # Turn notifications on by setting bit0 in the CCC more info on:
            p_reconn.writeCharacteristic(hMotionC, struct.pack('<bb', 0x01, 0x00), withResponse=True)
            print ("Notification is turned on for Raw_data : ", p_reconn.addr)