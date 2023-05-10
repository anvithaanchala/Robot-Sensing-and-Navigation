#!/usr/bin/env python3
#-*- coding: utf-8 -*-
from imu_driver.msg import Vectornav
from imu_driver.srv import convert_to_quaternion, convert_to_quaternionResponse
import serial
import rospy
import utm
from std_msgs.msg import String
import sys
import time	
import math
import matplotlib.pyplot as plt
from scipy import integrate
from scipy.signal import butter, filtfilt
import numpy as np
import requests, json
import googlemaps
import tf

def talker():

    pub = rospy.Publisher('imu', Vectornav, queue_size=10)
    
    port2 = '/dev/ttyUSB1'
    rate2 = rospy.Rate(1) 
    ser2= serial.Serial( port= port2, baudrate=4800, timeout = 5)
    

    now = rospy.get_rostime()

    # Wait for the service to become available
    #rospy.wait_for_service('/convert_to_quaternion')
    #quat_client = rospy.ServiceProxy('/convert_to_quaternion', convert_to_quaternion)
    
    
    accel_x = []
    gps_dist = 0
    i=1
    initial_v = 0
    # enter your api key here
    api_key ='AIzaSyBUFBQLqNywE1mEXc6yZgqpiT5aaTfDGjE'

    # Requires API key
    gmaps = googlemaps.Client(key=api_key)
    orig =  '4-24 Albemarle St, Boston, MA 02115'


    dest = '42.345418, -71.082261'

# DONT RUN THE LINE BELOW IN A FOR LOOP IT WILL COST ME A LOT OF MONEY
    #result = gmaps.distance_matrix(orig, dest, mode='walking')["rows"][0]["elements"][0]
    

# debug/test value
    #result = {'distance': {'text': '2.2 km', 'value': 2232}, 'duration': {'text': '30 mins', 'value': 1828}, 'status': 'OK'}
    #result = {'distance': {'text': '0.6 km', 'value': 621}, 'duration': {'text': '8 mins', 'value': 460}, 'status': 'OK'}
    #result = {'distance': {'text': '0.7 km', 'value': 703}, 'duration': {'text': '9 mins', 'value': 521}, 'status': 'OK'}
    result = {'distance': {'text': '0.5 km', 'value': 479}, 'duration': {'text': '6 mins', 'value': 376}, 'status': 'OK'}

    print(result)

# collect distance in meters
    dist = result['distance']['value']
# collect time in seconds
    gtime = result['duration']['value']

# googles estimated velocity in m/s
    gveloc = dist/gtime

    gps_last = [0,0]
    gps_point = [0,0]
    total_dist = 0
    t0 = time.time()
    print(t0)
    msg = Vectornav()
    while not rospy.is_shutdown():
        sentence1 = ser1.readline()
        sentence3 = sentence1.decode()
        elementsi = sentence3.split(',')

        sentence2 = ser2.readline()
        sentence4 = sentence2.decode()
        elementsg = sentence4.split(',')
        #print(sentence3)
        #print(sentence4)
     
        if '$GPGGA' in elementsg[0]:
            #elements = sentence4.split(',')
            #print(sentence4)
            time_str = elementsg[1]
            latitude_str = elementsg[2]
            longitude_str = elementsg[4]
            altitude = float(elementsg[9])
            hdop = elementsg[8]

            sec = float(time_str[:2]) * 60 * 60 + float(time_str[2:4]) * 60 + float(time_str[4:6]) #calculates the number of seconds 
            nsec = (float(time_str[6:])) * 10e9 #calculates the number of nano secinds 

            latitude_decimal = float(latitude_str[:2]) + (float(latitude_str[2:]) / 60) #converting the latitude values into decimal 
            longitude_decimal = (float(longitude_str[:3]) + (float(longitude_str[3:]) / 60)) * -1 #converting the longitude values into decimal
	
            utm_coord = utm.from_latlon(latitude_decimal, longitude_decimal) #using the UTM library function to convert from decimal to utm coordinates
            zone = utm_coord[2] #getting the zone
            letter = utm_coord[3] #getting the letter
            utm_northing = utm_coord[0] #northing data
            utm_easting = utm_coord[1] #easting data
            gps_point = [utm_northing, utm_easting]
            
           
        
        
        
        if '$VNYMR' in elementsi[0]:
            #print(sentence3)
            #print(gps_point)
            #print(gps_last)
            #elements = sentence3.split(',')
            #roll = float(elements[3])
            #pitch = float(elements[2])
            #yaw = float(elements[1])

            

            #angl_z1 = elements[12]
            #angl_z = float(angl_z1[:-5])

            #imu.angular_velocity.x = float(elements[10])
            #msg.imu.angular_velocity.y = float(elements[11])
            #msg.imu.angular_velocity.z = angl_z

            msg.imu.linear_acceleration.x = float(elementsi[7])
            msg.imu.linear_acceleration.y = float(elementsi[8])
            #msg.imu.linear_acceleration.z = float(elementsi[9])
    
            #msg.mag_field.magnetic_field.x = float(elements[4])
            #msg.mag_field.magnetic_field.y = float(elements[5])
            #msg.mag_field.magnetic_field.z = float(elements[6])
            

            # Set the message header timestamp
            msg.Header.stamp = rospy.Time.now()
            
            
            
             
            
            accel_x.append(msg.imu.linear_acceleration.x)
            #print('imu go')
            rate1.sleep()
            
        tcalc = float(time.time() - t0)
        if tcalc > 10*(10^11) :
        #Analyzer
           print(accel_x)
           t0 = time.time()
           gps_dist = 0
           if gps_last[0] == 0:
               gps_last[0] = gps_point[0]
               gps_last[1] = gps_point[1]
            	
           else:
               print(gps_point)
               print(gps_last)
               gps_dist = ((gps_point[0] - gps_last[0])**2 + (gps_point[1] - gps_last[1])**2)**.5
               gps_last[0] = gps_point[0]
               gps_last[1] = gps_point[1]
               
          
           IMUveloc = gps_dist/10          
           mean = np.mean(accel_x)
           accel_x = accel_x - mean
           fc = 10  # Cut-off frequency for low-pass filter in Hz
           #fs = 1 / (current_time[1] - current_time[0])  # Sampling frequency
           fs = 40
           b, a = butter(2, fc / (fs / 2))
           accel_x = filtfilt(b, a, accel_x)
                
           velocity_x = integrate.cumtrapz(accel_x, initial=initial_v)
           velocity_x = abs(velocity_x)
           velocity_x = np.median(velocity_x)
           print("velocity=",velocity_x)
           initial_v = initial_v + velocity_x
           accel_x = []
               
               
           GPSveloc = gps_dist/10
    
           AVGveloc = (IMUveloc + GPSveloc)/2
           
           if AVGveloc < .01: AVGveloc = .01
               
           scaling = gveloc/AVGveloc
           print("scaling=",scaling)
           total_dist = total_dist + gps_dist
           #print("total dist=",total_dist)
           remdist = dist - total_dist
           print("remaining_dist=",remdist)
           gtimerem = remdist*gtime/dist
           print("gtime=",gtimerem)
           IMU_time = scaling*gtimerem
           print("IMU_time=",IMU_time)
           msg.imu_velocity = velocity_x
           msg.remaining_distance = remdist
           msg.remaining_time = IMU_time
           msg.remaining_google_time = gtimerem
           msg.gpsvelocity = GPSveloc
           msg.avgvelocity = AVGveloc
           print(AVGveloc)
           pub.publish(msg)
           
           
           msg.Header.frame_id= "imu1_frame"
           
           # rospy.loginfo("Current system time: {}".format(rospy.Time.now()))
if __name__ == '__main__':
    try:
        rospy.init_node('talker')
        port1 = '/dev/ttyUSB0'
        rate1 = rospy.Rate(40)
        ser1 = serial.Serial(port=port1, baudrate=115200, bytesize=8, timeout=5, stopbits=serial.STOPBITS_ONE)
        ser1.write(b"$VNWRG,07,40*XX")
        talker()
    except rospy.ROSInterruptException:
        pass
           
           
	



