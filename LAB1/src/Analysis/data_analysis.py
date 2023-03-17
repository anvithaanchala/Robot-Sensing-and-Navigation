#!/usr/bin/env python3
#-*- coding: utf-8 -*-
import bagpy
from bagpy  import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
binwidth = 10
moving_data=bagreader('/home/anvitha99/LAB1/src/Data/2023-02-05-17-15-19.bag')
stat_data=bagreader('/home/anvitha99/LAB1/src/Data/Stationary_Centennial.bag')
stat_occl_data=bagreader('/home/anvitha99/LAB1/src/Data/Stationary_occluded.bag')

movfree = moving_data.message_by_topic('/gps')
statfree = stat_data.message_by_topic('/gps')
statoccl = stat_occl_data.message_by_topic('/gps')

dt_movfree = pd.read_csv(movfree)
dt_statfree = pd.read_csv(statfree)
dt_statoccl = pd.read_csv(statoccl)

dt_statfree['UTM_northing_std']= dt_statfree['UTM_northing']- dt_statfree['UTM_northing'].min()
dt_statfree['UTM_easting_std']= dt_statfree['UTM_easting']- dt_statfree['UTM_easting'].min()
dt_statoccl['UTM_northing_std']= dt_statoccl['UTM_northing']- dt_statoccl['UTM_northing'].min()
dt_statoccl['UTM_easting_std']= dt_statoccl['UTM_easting']- dt_statoccl['UTM_easting'].min()
dt_movfree['UTM_northing_std']= dt_movfree['UTM_northing']- dt_movfree['UTM_northing'].min()
dt_movfree['UTM_easting_std']= dt_movfree['UTM_easting']- dt_movfree['UTM_easting'].min()
dt_statfree['Header.stamp.secs']= dt_statfree['Header.stamp.secs'] -77578
dt_statoccl['Header.stamp.secs']= dt_statfree['Header.stamp.secs'] 
dt_movfree['Header.stamp.secs']= dt_statfree['Header.stamp.secs'] 
dt_statfree['Altitude']= dt_statfree['Altitude']
dt_statfree['UTM_northing_std']= dt_statfree['UTM_northing']- dt_statfree['UTM_northing'].min()

dt_statfree['UTM_northing_ctd_st']= dt_statfree['UTM_northing']- 327793.625
dt_statfree['UTM_easting_ctd_st']= dt_statfree['UTM_easting']- 4689310.5
dt_statoccl['UTM_northing_ctd_sto']= dt_statoccl['UTM_northing']- 327819.28125
dt_statoccl['UTM_easting_ctd_sto']= dt_statoccl['UTM_easting']- 4689275.5
dt_movfree['UTM_northing_ctd_mo']= (dt_movfree['UTM_northing']- 327637.53125)**2
dt_movfree['UTM_easting_ctd_mo']= (dt_movfree['UTM_easting']- 4689254)

dt_statfree[['UTM_northing_std','UTM_easting_std']].plot.scatter(x='UTM_northing_std',y='UTM_easting_std')
plt.xlabel("UTM Northing (m)")
plt.ylabel("UTM Easting (m)")
plt.title("Standardized UTM Coordinates for stationary data collection ")
dt_statoccl[['UTM_northing_std','UTM_easting_std']].plot.scatter(x='UTM_northing_std',y='UTM_easting_std')
plt.xlabel("UTM northing (m)")
plt.ylabel("UTM Easting (m)")
plt.title("Standardized UTM Coordinates for stationary occluded data collection ")
dt_movfree[['UTM_northing_std','UTM_easting_std']].plot.scatter(x='UTM_northing_std',y='UTM_easting_std')
plt.xlabel("UTM northing (m)")
plt.ylabel("UTM Easting (m)")
plt.title("Standardized UTM Coordinates for moving data collection ")


dt_statfree[['Header.stamp.secs','Altitude']].plot.scatter(x='Header.stamp.secs',y='Altitude')
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.title("Time vs Altitude -Stationary  ")
dt_statoccl[['Header.stamp.secs','Altitude']].plot.scatter(x='Header.stamp.secs',y='Altitude')
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.title("Time vs Altitude -Occluded ")
dt_movfree[['Header.stamp.secs','Altitude']].plot.scatter(x='Header.stamp.secs',y='Altitude')
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.title("Time vs Altitude -Moving")


dt_movfree[['UTM_northing_ctd_mo','UTM_easting_ctd_mo']].plot.scatter(x='UTM_northing_ctd_mo',y='UTM_easting_ctd_mo')
plt.xlabel("UTM northing (m)")
plt.ylabel("UTM Easting (m)")
plt.title("Error distribution Coordinates for moving data collection ")

binwidth = 1
data = dt_statoccl['UTM_northing_ctd_sto']
bins = np.arange(min(data), max(data) + binwidth, binwidth)
plt.hist(data, bins=bins)
plt.xlabel('Northing data for stationary occluded data')
plt.ylabel('Frequency')
plt.figure()

binwidth = 1
data = dt_statfree['UTM_northing_ctd_st']
bins = np.arange(min(data), max(data) + binwidth, binwidth)
plt.hist(data, bins=bins)
plt.xlabel('Northing data for stationary data')
plt.ylabel('Frequency')
plt.figure()

binwidth = 1
data = dt_statfree['UTM_easting_ctd_st']
bins = np.arange(min(data), max(data) + binwidth, binwidth)
plt.hist(data, bins=bins)
plt.xlabel('Easting data for stationary data')
plt.ylabel('Frequency')
plt.figure()

binwidth = 1
data = dt_statoccl['UTM_easting_ctd_sto']
bins = np.arange(min(data), max(data) + binwidth, binwidth)
plt.hist(data, bins=bins)
plt.xlabel('Easting data for stationary occluded data')
plt.ylabel('Frequency')
plt.figure()


binwidth = 1
data = dt_statoccl['UTM_northing_ctd_sto']
bins = np.arange(min(data), max(data) + binwidth, binwidth)
plt.hist(data, bins=bins)
plt.xlabel('Northing data for stationary occluded data')
plt.ylabel('Frequency')
plt.figure()








plt.show()
