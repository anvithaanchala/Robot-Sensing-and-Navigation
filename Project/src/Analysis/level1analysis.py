import matplotlib.pyplot as plt
import csv
import numpy as np

x1 = []
imu_v = []
remdist = []
remtime = []
remtimeg = []

i = 0
t0 = 0
with open('/home/benny/catkin_ws/src/EECE5554/Project/level1/imu.csv','r') as csvfile1:
    plot1 = csv.reader(csvfile1, delimiter = ',')
    for row in plot1:
        if i > 0:
            if t0 == 0:
                t0 = float(row[0]) 
            imu_v.append(float(row[31]))
            remdist.append(float(row[32]))
            remtime.append(float(row[33]))
            remtimeg.append(float(row[34]))
            x1.append(float(row[0]) - t0)
        i += 1

#plt.scatter(x1, remtime, label='calculated distance')
x1 = np.array(x1)
a,b = np.polyfit(x1, remtime, 1)
#plt.plot(x1, x1*a + b, label='scaled estimated time')
plt.plot(x1, imu_v, label = 'estimated time without scaling')
#plt.plot(x1, y2, label="y")
#plt.plot(x1, y3, label="z")
plt.title('IMU calculated velocity')
plt.xlabel("Time (s)")
plt.ylabel("IMU velocity (m/s)")
#plt.legend()
plt.show()