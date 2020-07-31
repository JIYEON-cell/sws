##########################################################################
import struct
import cv2
import threading
import sys
import bluepy.btle
from bluepy.btle import *
from time import sleep
import time
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
#########################################################################
target_name = "ELA_106"   # target device name
target_address = None     # target device address
motion_service_uuid = "ef680400-9b35-4933-9b10-52ffa9740042"
motion_char_uuid = "ef680406-9b35-4933-9b10-52ffa9740042"
count = 0
data_flag = 0
threads = []
Peripherals = []
##########################################################################
class MyDelegate(DefaultDelegate):                    #Constructor (run once on startup)
    def __init__(self, params):
        DefaultDelegate.__init__(self)

    def handleNotification(self, cHandle, data):      #func is caled on notifications
        motion_data = struct.unpack('hhhhhhhhh',data) # 18byte
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

'''
# prevent raising exception : now error state returns none, not exception
class _BluepyHelper(BluepyHelper):
    def _writeCmd(self, cmd):

        ''''''
        if self._helper is None:
            raise BTLEInternalError("Helper not started (did you call connect()?)")
        DBG("Sent: ", cmd)
        ''''''

        if self._helper is None:
            self._startHelper()
        DBG("Sent: ", cmd)

        ##############################################
        
        self._helper.stdin.write(cmd)
        self._helper.stdin.flush()

    def _waitResp(self, wantType, timeout=None):
        ''''''
        while True:
            if self._helper.poll() is not None:
                raise BTLEInternalError("Helper exited")

            if timeout:
                fds = self._poller.poll(timeout*1000)
                if len(fds) == 0:
                    DBG("Select timeout")
                    return None

            rv = self._helper.stdout.readline()
            DBG("Got:", repr(rv))
            if rv.startswith('#') or rv == '\n' or len(rv)==0:
                continue

            resp = BluepyHelper.parseResp(rv)
            if 'rsp' not in resp:
                raise BTLEInternalError("No response type indicator", resp)

            respType = resp['rsp'][0]
            if respType in wantType:
                return resp
            elif respType == 'stat':
                if 'state' in resp and len(resp['state']) > 0 and resp['state'][0] == 'disc':
                    self._stopHelper()
                    raise BTLEDisconnectError("Device disconnected", resp)
            elif respType == 'err':
                errcode=resp['code'][0]
                if errcode=='nomgmt':
                    raise BTLEManagementError("Management not available (permissions problem?)", resp)
                elif errcode=='atterr':
                    raise BTLEGattError("Bluetooth command failed", resp)
                else:
                    raise BTLEException("Error from bluepy-helper (%s)" % errcode, resp)
            elif respType == 'scan':
                # Scan response when we weren't interested. Ignore it
                continue
            else:
                raise BTLEInternalError("Unexpected response (%s)" % respType, resp)
        ''''''

        while True:
            if self._helper.poll() is not None:
                return None

            if timeout:
                fds = self._poller.poll(timeout*1000)
                if len(fds) == 0:
                    DBG("Select timeout")
                    return None

            rv = self._helper.stdout.readline()
            
            DBG("Got:", repr(rv))
            if rv.startswith('#') or rv == '\n' or len(rv)==0:
                continue

            resp = BluepyHelper.parseResp(rv)
            if 'rsp' not in resp:
                return None

            respType = resp['rsp'][0]
            if respType in wantType:
                return resp
            elif respType == 'stat':
                if 'state' in resp and len(resp['state']) > 0 and resp['state'][0] == 'disc':
                    #self._stopHelper()
                    return 'rec'
            elif respType == 'err':
                errcode=resp['code'][0]
                if errcode=='nomgmt':
                    return None
                elif errcode=='atterr':
                    return None
                else:
                    return None
            elif respType == 'scan':
                # Scan response when we weren't interested. Ignore it
                continue
            else:
                return None
'''
'''
class _Peripheral(_BluepyHelper) :

    def __init__(self, deviceAddr=None, addrType=ADDR_TYPE_PUBLIC, iface=None):
        BluepyHelper.__init__(self)
        self._serviceMap = None # Indexed by UUID
        (self.deviceAddr, self.addrType, self.iface) = (None, None, None)

        if isinstance(deviceAddr, ScanEntry):
            self._connect(deviceAddr.addr, deviceAddr.addrType, deviceAddr.iface)
        elif deviceAddr is not None:
            self._connect(deviceAddr, addrType, iface)

    def setDelegate(self, delegate_): # same as withDelegate(), deprecated
        return self.withDelegate(delegate_)

    def __enter__(self):
        return self

    def _getResp(self, wantType, timeout=None):
        if isinstance(wantType, list) is not True:
            wantType = [wantType]

        while True:
            resp = self._waitResp(wantType + ['ntfy', 'ind'], timeout)
            
            if resp is None:
                resp = self._waitResp(['stat'] + ['ntfy', 'ind'], timeout)
                
                if resp is None : return None

                if resp is 'rec' :
                    self._connect("f9:44:f4:fa:6a:ea", "random")
                return None

            respType = resp['rsp'][0]

            if respType == 'ntfy' or respType == 'ind':
                hnd = resp['hnd'][0]
                data = resp['d'][0]
                if self.delegate is not None:
                    self.delegate.handleNotification(hnd, data)
                if respType not in wantType:
                    continue
            return resp

    def _connect(self, addr, addrType=ADDR_TYPE_PUBLIC, iface=None):
        if len(addr.split(":")) != 6:
            raise ValueError("Expected MAC address, got %s" % repr(addr))
        if addrType not in (ADDR_TYPE_PUBLIC, ADDR_TYPE_RANDOM):
            raise ValueError("Expected address type public or random, got {}".format(addrType))
        self._startHelper(iface)
        self.addr = addr
        self.addrType = addrType
        self.iface = iface
        if iface is not None:
            self._writeCmd("conn %s %s %s\n" % (addr, addrType, "hci"+str(iface)))
        else:
            self._writeCmd("conn %s %s\n" % (addr, addrType))
        rsp = self._getResp('stat')
        
        ''''''        
        while rsp['state'][0] == 'tryconn':
            rsp = self._getResp('stat')
        if rsp['state'][0] != 'conn':
            self._stopHelper()
            raise BTLEDisconnectError("Failed to connect to peripheral %s, addr type: %s" % (addr, addrType), rsp)
        ''''''
        
        while rsp['state'][0] == 'tryconn':
            rsp = self._getResp('stat')
            if rsp['state'][0] != 'conn':
                self._writeCmd("conn %s %s\n" % (addr, addrType))
            else:
                break

    def __exit__(self, type, value, traceback):
        self.disconnect()

    def connect(self, addr, addrType=ADDR_TYPE_PUBLIC, iface=None):
        if isinstance(addr, ScanEntry):
            self._connect(addr.addr, addr.addrType, addr.iface)
        elif addr is not None:
            self._connect(addr, addrType, iface)

    def disconnect(self):
        if self._helper is None:
            return
        # Unregister the delegate first
        self.setDelegate(None)

        self._writeCmd("disc\n")
        self._getResp('stat')
        self._stopHelper()

    def discoverServices(self):
        self._writeCmd("svcs\n")
        rsp = self._getResp('find')
        starts = rsp['hstart']
        ends   = rsp['hend']
        uuids  = rsp['uuid']
        nSvcs = len(uuids)
        assert(len(starts)==nSvcs and len(ends)==nSvcs)
        self._serviceMap = {}
        for i in range(nSvcs):
            self._serviceMap[UUID(uuids[i])] = Service(self, uuids[i], starts[i], ends[i])
        return self._serviceMap

    def getState(self):
        status = self.status()
        return status['state'][0]

    @property
    def services(self):
        if self._serviceMap is None:
            self._serviceMap = self.discoverServices()
        return self._serviceMap.values()

    def getServices(self):
        return self.services

    def getServiceByUUID(self, uuidVal):
        uuid = UUID(uuidVal)
        if self._serviceMap is not None and uuid in self._serviceMap:
            return self._serviceMap[uuid]
        
        ''''''
        self._writeCmd("svcs %s\n" % uuid)
        rsp = self._getResp('find')
        ''''''

        while(True) :
            self._writeCmd("svcs %s\n" % uuid)
            rsp = self._getResp('find')
            if rsp != None : break

        if 'hstart' not in rsp:
            raise BTLEGattError("Service %s not found" % (uuid.getCommonName()), rsp)
        svc = Service(self, uuid, rsp['hstart'][0], rsp['hend'][0])
        
        if self._serviceMap is None:
            self._serviceMap = {}
        self._serviceMap[uuid] = svc
        return svc

    def _getIncludedServices(self, startHnd=1, endHnd=0xFFFF):
        # TODO: No working example of this yet
        self._writeCmd("incl %X %X\n" % (startHnd, endHnd))
        return self._getResp('find')

    def getCharacteristics(self, startHnd=1, endHnd=0xFFFF, uuid=None):
        cmd = 'char %X %X' % (startHnd, endHnd)
        if uuid:
            cmd += ' %s' % UUID(uuid)
        self._writeCmd(cmd + "\n")
        rsp = self._getResp('find')
        nChars = len(rsp['hnd'])
        return [Characteristic(self, rsp['uuid'][i], rsp['hnd'][i],
                               rsp['props'][i], rsp['vhnd'][i])
                for i in range(nChars)]

    def getDescriptors(self, startHnd=1, endHnd=0xFFFF):
        self._writeCmd("desc %X %X\n" % (startHnd, endHnd) )
        # Historical note:
        # Certain Bluetooth LE devices are not capable of sending back all
        # descriptors in one packet due to the limited size of MTU. So the
        # guest needs to check the response and make retries until all handles
        # are returned.
        # In bluez 5.25 and later, gatt_discover_desc() in attrib/gatt.c does the retry
        # so bluetooth_helper always returns a full list.
        # This was broken in earlier versions.
        resp = self._getResp('desc')
        ndesc = len(resp['hnd'])
        return [Descriptor(self, resp['uuid'][i], resp['hnd'][i]) for i in range(ndesc)]

    def readCharacteristic(self, handle):
        self._writeCmd("rd %X\n" % handle)
        resp = self._getResp('rd')
        return resp['d'][0]

    def _readCharacteristicByUUID(self, uuid, startHnd, endHnd):
        # Not used at present
        self._writeCmd("rdu %s %X %X\n" % (UUID(uuid), startHnd, endHnd))
        return self._getResp('rd')

    def writeCharacteristic(self, handle, val, withResponse=False):
        # Without response, a value too long for one packet will be truncated,
        # but with response, it will be sent as a queued write
        cmd = "wrr" if withResponse else "wr"
        self._writeCmd("%s %X %s\n" % (cmd, handle, binascii.b2a_hex(val).decode('utf-8')))
        return self._getResp('wr')

    def setSecurityLevel(self, level):
        self._writeCmd("secu %s\n" % level)
        return self._getResp('stat')

    def unpair(self):
        self._mgmtCmd("unpair")

    def pair(self):
        self._mgmtCmd("pair")

    def setMTU(self, mtu):
        self._writeCmd("mtu %x\n" % mtu)
        return self._getResp('stat')

    def waitForNotifications(self, timeout):
        resp = self._getResp(['ntfy','ind'], timeout)
        return (resp != None)

    def __del__(self):
        self.disconnect()
'''
############################################################################
def RCV_IMU(p, iter_thread, len_thread):
    global data_flag                    # check recording status
                                        # init(gen file) > on rec. IMU > on rec.cam > off rec.IMU > off rec.cam > init > ...
    global file_flag                    # check file generation for each IMU
    #bluepy.btle.Debugging = True
    file_flag = [0 for iter in range(len_thread)]
    file_path = ''
    count = 0
    #p_reconn = ''

    while True:    
        if p.waitForNotifications(1.0) : #handleNotification() called 
            if data_flag == dict_state['init'] and file_flag[iter_thread] == 0:
                # if thread(iter) does not generated file before record
                date = datetime.now().strftime('%Y%m%d_%H%M%S')
                file_path = ('/home/pi/Documents/sws/'+ date + '(' +p.addr + ')' +'.csv')
                with open(file_path, 'a') as out_file:
                    out_file.write("sequence,AccelerometerX,AccelerometerY,AccelerometerZ,GyroscopeX,GyroscopeY,GyroscopeZ\n")
                file_flag[iter_thread] = 1
                print('record start : ' + file_path)

                if sum(file_flag) == len_thread: # if all thread has generated file
                    data_flag = dict_state['IMU_on']  
                

            if data_flag == dict_state['cam_on']:
                
                with open(file_path, 'a') as out_file:
                    out_file.write("%d,%d,%d,%d,%d,%d,%d\n"%(count,mAccelerometerX, mAccelerometerY, mAccelerometerZ, mGyroscopeX, mGyroscopeY, mGyroscopeZ))
                
                #print(count, '||' ,mAccelerometerX, mAccelerometerY, mAccelerometerZ, mGyroscopeX, mGyroscopeY, mGyroscopeZ)
                
                count = count + 1
                if (count == 60000) and (iter_thread == (len_thread - 1)): # if last IMU thread finished recording 
                    data_flag = dict_state['IMU_off']
                
            if (data_flag == dict_state['cam_off']):
                # if camera off & at last thread > return to init. state
                count = 0
                file_flag[iter_thread] = 0
                if sum(file_flag) == 0: ## if all record stopped
                    data_flag = dict_state['init']

        '''
        else :
            tmp_addr = p.addr                                
            p.disconnect()
            p_reconn = Peripheral(tmp_addr,"random")
            #Get MotionService
            p_reconn.setDelegate( MyDelegate(p_reconn) )
            MotionService=p_reconn.getServiceByUUID(motion_service_uuid)
            # Get The Motion-Characteristics
            MotionC = MotionService.getCharacteristics(motion_char_uuid)[0]
            #Get The handle tf the  Button-Characteristics
            hMotionC = MotionC.getHandle() + 1
            # Turn notifications on by setting bit0 in the CCC more info on:
            p_reconn.writeCharacteristic(hMotionC, struct.pack('<bb', 0x01, 0x00), withResponse=True)
            
            p = p_reconn
            print ("select-timeout : reconnection... - ", p_reconn.addr) 
            '''
        continue
        print ("Waiting... Waited more than one sec for notification")

def RCV_cam():
    global data_flag
    while True:   
        if data_flag == dict_state['IMU_on'] : # initialize
            '''
            date = datetime.now().strftime('%Y%m%d_%H%M%S')
            cam = cv2.VideoCapture(-1)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter('/home/pi/Documents/sws/'+date+'.mp4',fourcc, 10.0,(640,480))
            start = time.time()
            '''
            data_flag = dict_state['cam_on']
            
            #print("cam start : ", date)
            '''
        if data_flag == dict_state['cam_on']: # recording
            
            cv2.waitKey(70)
            _, frame = cam.read()
            out.write(frame)
            '''
        if data_flag == dict_state['IMU_off'] : # when IMU recording en
            '''
            print("record end :", time.time() - start)
            cam.release()
            out.release()
            '''
            data_flag = dict_state['cam_off'] # finish camera record

###########################################################################start
def main():
    #bluepy.btle.Debugging = True
    scanner = Scanner()
    devices = scanner.scan(3.0)
    for dev in devices:
        for (_, _, value) in dev.getScanData():
            if target_name == value:
                target_address = dev.addr
                # create peripheral class
                Peripherals.append(Peripheral(target_address,"random"))
                break

    for iter in range(len(Peripherals)) :
        Peripherals[iter].setDelegate( MyDelegate(Peripherals[iter]) )
        
        # Get MotionService
        MotionService=Peripherals[iter].getServiceByUUID(motion_service_uuid)
        # Get The Motion-Characteristics
        MotionC = MotionService.getCharacteristics(motion_char_uuid)[0]
        # Get The handle tf the  Button-Characteristics
        hMotionC = MotionC.getHandle()+1
        # Turn notifications on by setting bit0 in the CCC more info on:
        Peripherals[iter].writeCharacteristic(hMotionC, struct.pack('<bb', 0x01, 0x00), withResponse=True)

        print (Peripherals[iter].addr + " : Notification is turned on for Raw_data")
        t = threading.Thread(target = RCV_IMU, args = (Peripherals[iter],iter, len(Peripherals)))
        threads.append(t)
    
    threads.append(threading.Thread(target = RCV_cam))

    for iter in range(len(threads)) :
        threads[iter].start()

    for iter in range(len(threads)) :
        threads[iter].join()

if __name__ == '__main__':
    main()