import serial               #import serial pacakge
from time import sleep
import webbrowser           #import package for opening link in browser
import sys                  #import system package
import math

def GPS_Info():
    global NMEA_buff
    global lat_in_degrees
    global long_in_degrees
    nmea_time = []
    nmea_latitude = []
    nmea_longitude = []
    nmea_time = NMEA_buff[0]                    #extract time from GPGGA string
    nmea_latitude = NMEA_buff[1]                #extract latitude from GPGGA string
    nmea_longitude = NMEA_buff[3]               #extract longitude from GPGGA string
    
    print('buff :', NMEA_buff)
    print("NMEA Time: ", nmea_time,'\n')
    print ("NMEA Latitude:", nmea_latitude,"NMEA Longitude:", nmea_longitude,'\n')

    lat = float(nmea_latitude)                  #convert string into float for calculation
    longi = float(nmea_longitude)               #convertr string into float for calculation
    
    if nmea_latitude is '' or nmea_longitude is '':
        print("===========no available GPS data===========")
    else:
        lat = float(nmea_latitude)                  #convert string into float for calculation
        longi = float(nmea_longitude)               #convertr string into float for calculation
        
        lat_in_degrees = convert_to_degrees(lat)    #get latitude in degree decimal format
        long_in_degrees = convert_to_degrees(longi) #get longitude in degree decimal format
          
#convert raw NMEA string into degree decimal format   
def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "%.4f" %(position)
    return position
    

def deg2rad(deg):
    return (deg * math.pi / 180.0)

def rad2deg(rad):
    return (rad * 180.0 / math.pi)

def distance(lat1, lon1, lat2, lon2):
    theta = lon1 - lon2
    dist = math.sin(deg2rad(lat1)) * math.sin(deg2rad(lat2)) + \
            math.cos(deg2rad(lat1)) * math.cos(deg2rad(lat2)) * \
            math.cos(deg2rad(theta))
    dist = math.acos(dist)
    dist = rad2deg(dist)
    dist = dist * 60 * 1.1515
    dist = dist * 1.609344
    return dist



gpgga_info = "$GPGGA,"
ser = serial.Serial ("/dev/serial0")
GPGGA_buffer = 0
NMEA_buff = 0
lat_in_degrees = 0
long_in_degrees = 0
mem_dist = []
count = 0
point_dist = 0

try:
    while True:
        received_data = str(ser.readline())
        GPGGA_data_available = received_data.find(gpgga_info)
        if (GPGGA_data_available>0):
            GPGGA_buffer = received_data.split("$GPGGA,",1)[1]
            NMEA_buff = (GPGGA_buffer.split(','))
            GPS_Info()           #get time, latitude, longitude
 
            count+=1
            lat_in_degrees = float(lat_in_degrees)
            long_in_degrees = float(long_in_degrees)
             
            print("lat in degrees:", lat_in_degrees," long in degree: ", long_in_degrees, '\n')
            
            if count == 1:
                global file_path
                date = datetime.now().strftime('%Y%m%d_%H%M%S')
                file_path = ('/home/pi/Documents/sws/'+ date +'.csv')
                with open(file_path, 'a') as out_file:
                    out_file.write("sequence,lat_in_degrees,long_in_degrees\n")

            
            if count%10 == 0:            
                mem_dist.append(lat_in_degrees)
                mem_dist.append(long_in_degrees)

                with open(file_path, 'a') as out_file:
                    out_file.write("%d,%d,%d\n"%(count,lat_in_degrees, long_in_degrees))
#  
               
               
            if len(mem_dist) == 4:            
                point_dist = distance(mem_dist[0],mem_dist[1], mem_dist[2], mem_dist[3])
                point_dist+=point_dist
                count = 0
                print("point_dist:", point_dist, '\n')

#            map_link = 'http://maps.google.com/?q=' + lat_in_degrees + ',' + long_in_degrees    #create link to plot location on Google map
#            print("<<<<<<<<press ctrl+c to plot location on google maps>>>>>>\n")               #press ctrl+c to plot on map and exit 
#            print("------------------------------------------------------------\n")
                        
except KeyboardInterrupt:
    webbrowser.open(map_link)        #open current position information in google map
    sys.exit(0)

            


