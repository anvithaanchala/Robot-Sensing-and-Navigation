#!/usr/bin/env python3
#-*- coding: utf-8 -*-
from gps_driver.msg import gps_msg #imports gps msg from gps_driver

import serial
import rospy
import utm
from std_msgs.msg import String
import sys
rospy.init_node('talker')
port = rospy.get_param("~port_number")
rate = rospy.Rate(1) 
ser= serial.Serial( port= port, baudrate=4800, timeout = 5)
pub= rospy.Publisher('gps',gps_msg,queue_size=10)

try:
    while not rospy.is_shutdown(): 
        sentence = str(ser.readline())
        if 'GPGGA' in str(sentence):
            print(sentence)
            elements = sentence.split(',')
            time_str = elements[1]
            latitude_str = elements[2]
            longitude_str = elements[4]
            altitude = (elements[8])

            sec = float(time_str[:2]) * 60 * 60 + float(time_str[2:4]) * 60 + float(time_str[4:6]) #calculates the number of seconds 
            nsec = (float(time_str[6:])) * 10e6 #calculates the number of nano secinds 

            latitude_decimal = float(latitude_str[:2]) + float(latitude_str[2:]) / 60 #converting the latitude values into decimal 
            longitude_decimal = (float(longitude_str[:3]) + float(longitude_str[3:]) / 60) * -1 #converting the longitude values into decimal

            utm_coord = utm.from_latlon(latitude_decimal, longitude_decimal) #using the UTM library function to convert from decimal to utm coordinates
            zone = utm_coord[2] #getting the zone
            letter = utm_coord[3] #getting the letter
            utm_northing = utm_coord[0] #northing data
            utm_easting = utm_coord[1] #easting data

            msg = gps_msg()
            msg.Latitude = latitude_decimal
            msg.Longitude = longitude_decimal
            msg.UTM_northing = utm_northing
            msg.UTM_easting = utm_easting
            msg.Altitude = float(altitude)
            msg.Zone = zone
            msg.Letter = letter
            msg.Header.stamp.secs = int(sec)
            msg.Header.stamp.nsecs = int(nsec)
            msg.Header.stamp.frame_id = "GPS1_FRAME"
            print(sec, nsec)

            pub.publish(msg)
            rate.sleep()

except rospy.ROSInterruptException:
    pass
