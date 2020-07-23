import struct
import sys
from bluepy.btle import Scanner, DefaultDelegate
from bluepy.btle import UUID, Peripheral
from picamera import PiCamera
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
            camera.stop_recording()
            camera.stop_preview()

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print ("Discovered device", dev.addr)
        elif isNewData:
            print ("Received new data from", dev.addr)

scanner = Scanner()
devices = scanner.scan(6.0)
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 10

for dev in devices:
    #print ("Device %s (%s), RSSI=%d dB" % (dev.addr, dev.addrType, dev.rssi))

    for (adtype, desc, value) in dev.getScanData():
     #   print ("  %s = %s" % (desc, value))
        if target_name == value:
            print(value)
            target_address = dev.addr
            print(dev.addr)
            break

p = Peripheral(target_address,"random")

#Get MotionService
MotionService=p.getServiceByUUID(motion_service_uuid)
p.setDelegate( MyDelegate(p) )
# Get The Motion-Characteristics
MotionC = MotionService.getCharacteristics(motion_char_uuid)[0]
#Get The handle tf the  Button-Characteristics
hMotionC=MotionC.getHandle()+1
#print("  0x"+ format(hMotionC,'X') )

# Turn notifications on by setting bit0 in the CCC more info on:
p.writeCharacteristic(hMotionC, struct.pack('<bb', 0x01, 0x00), withResponse=True)
print ("Notification is turned on for Raw_data")


while True:
    
    if p.waitForNotifications(1.0):
           #handleNotification() called

        if data_flag == True:
            print(count)
            if count == 1:
                global file_path
                date = datetime.now().strftime('%Y%m%d_%H%M%S')
                file_path = ('/home/pi/Documents/sws/'+ date +'.csv')
                with open(file_path, 'a') as out_file:
                    out_file.write("sequence,AccelerometerX,AccelerometerY,AccelerometerZ,GyroscopeX,GyroscopeY,GyroscopeZ\n")
#
                
#                file_path = ('/media/pi/ZYRUS/'+ date +'.txt')
                print(file_path)
                camera.start_preview()
                camera.start_recording('/home/pi/Documents/sws/'+ date +'.h264')
#                camera.start_recording('/media/pi/ZYRUS/'+ date +'.h264')
                
            with open(file_path, 'a') as out_file:
                out_file.write("%d,%d,%d,%d,%d,%d,%d\n"%(count,mAccelerometerX, mAccelerometerY, mAccelerometerZ, mGyroscopeX, mGyroscopeY, mGyroscopeZ))
#                camera.annotate_text = "%d,%d,%d,%d,%d,%d"%(mAccelerometerX, mAccelerometerY, mAccelerometerZ, mGyroscopeX, mGyroscopeY, mGyroscopeZ)

    continue

    print ("Waiting... Waited more than one sec for notification")
           

        #   p.disconnect()
    # Perhaps do something else here

 