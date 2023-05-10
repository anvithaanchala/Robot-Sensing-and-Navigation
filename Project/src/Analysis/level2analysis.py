import matplotlib.pyplot as plt
import csv
import numpy as np

x1 = []
imu_v1 = []
gps_v1 = []
avg_v1 = []
remdist1 = []
remtime1 = []
remtimeg1 = []

x2 = []
imu_v2 = []
gps_v2 = []
avg_v2 = []
remdist2 = []
remtime2 = []
remtimeg2 = []

i = 0
t0 = 0
with open('/home/benny/catkin_ws/src/EECE5554/Project/forward1/imu.csv','r') as csvfile1:
    plot1 = csv.reader(csvfile1, delimiter = ',')
    for row in plot1:
        if i > 0:
            if t0 == 0:
                t0 = float(row[0])
            if float(row[33]) > 700:
                continue
            imu_v1.append(float(row[31]))
            remdist1.append(float(row[32]))
            remtime1.append(float(row[33]))
            remtimeg1.append(float(row[34]))
            gps_v1.append(float(row[42]))
            avg_v1.append((float(row[31]) + float(row[42]))/2)
            x1.append(float(row[0]) - t0)
        i += 1

i = 0
t0 = 0
with open('/home/benny/catkin_ws/src/EECE5554/Project/backward/imu.csv','r') as csvfile2:
    plot1 = csv.reader(csvfile2, delimiter = ',')
    for row in plot1:
        if i > 0:
            if t0 == 0:
                t0 = float(row[0])
            if float(row[33]) > 700:
                continue
            imu_v2.append(float(row[31]))
            remdist2.append(float(row[32]))
            remtime2.append(float(row[33]))
            remtimeg2.append(float(row[34]))
            gps_v2.append(float(row[42]))
            avg_v2.append((float(row[31]) + float(row[42]))/2)
            x2.append(float(row[0]) - t0)
        i += 1


#plt.scatter(x1, remtime, label='calculated distance')
x1 = np.array(x1)
#a,b = np.polyfit(x1, remtime, 1)
#plt.plot(x1, x1*a + b, label='scaled estimated time')
plt.plot(x2, imu_v2, label = 'Velocity from IMU')
plt.plot(x2, gps_v2, label = 'Velocity from GPS')
plt.plot(x2, avg_v2, label = 'Scaling Velocity')
#plt.plot(x1, y3, label="z")
plt.title('Velocities of backward dataset')
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.legend()
plt.show()