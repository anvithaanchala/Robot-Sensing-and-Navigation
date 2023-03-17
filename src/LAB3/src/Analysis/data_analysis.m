
%reading data from 3 different rosbag files and using select to filter
%messages in the topic "imu"
%linear acceleration ,angular accelaration ,mag field and
%orientation(quaternions to roll pitch and yaw 
imu_stat = select(rosbag('imu_data.bag'),'Topic','/imu');
imu_group = select(rosbag('group.bag'),'Topic','/imu');
imu_d = select(rosbag('LocationD.bag'),'Topic','/vectornav');
% Reads messages from rosbag and formats data as a structure 
imu_stat_1 = readMessages(imu_stat,'DataFormat','struct');
imu_group_1 = readMessages(imu_group,'DataFormat','struct');
%imu_d_1 = readMessages(imu_d,'DataFormat','struct');
mag_x = cell2mat(cellfun(@(m) [m.MagField.MagneticField_.X],imu_stat_1,'UniformOutput',false));
mag_y = cell2mat(cellfun(@(m) [m.MagField.MagneticField_.Y],imu_stat_1,'UniformOutput',false));
mag_z = cell2mat(cellfun(@(m) [m.MagField.MagneticField_.Z],imu_stat_1,'UniformOutput',false));
quat = cell2mat(cellfun(@(m) [m.IMU.Orientation.W m.IMU.Orientation.X m.IMU.Orientation.Y m.IMU.Orientation.Z], imu_stat_1, 'UniformOutput',false));
eul = quat2eul(quat);
angVel_x = cell2mat(cellfun(@(m) [m.IMU.AngularVelocity.X],imu_stat_1,'UniformOutput',false));
angVel_y = cell2mat(cellfun(@(m) [m.IMU.AngularVelocity.Y],imu_stat_1,'UniformOutput',false));
angVel_z = cell2mat(cellfun(@(m) [m.IMU.AngularVelocity.Z],imu_stat_1,'UniformOutput',false));
linAccel_x = cell2mat(cellfun(@(m) [m.IMU.LinearAcceleration.X ],imu_stat_1,'UniformOutput',false));
linAccel_y = cell2mat(cellfun(@(m) [m.IMU.LinearAcceleration.Y],imu_stat_1,'UniformOutput',false));
linAccel_z = cell2mat(cellfun(@(m) [m.IMU.LinearAcceleration.Z],imu_stat_1,'UniformOutput',false));
time = cellfun(@(m) str2double(strcat(num2str(m.Header.Stamp.Sec-imu_stat_1{1}.Header.Stamp.Sec),'.',num2str(m.Header.Stamp.Nsec))),imu_stat_1);
%-------------------------------------------------------------%
figure('Name', 'Magnetic field data');
subplot(2,2,1)
plot(time(1:end),mag_x,'.')
title("Magnetic Field X")
xlabel("Time (s)")
ylabel("Magnetic Field_x (T)")

subplot(2,2,2)
plot(time(1:end),mag_y,'.')
title("Magnetic Field Y")
xlabel("Time (s)")
ylabel("Magnetic Field_y (T)")

subplot(2,2,3)
plot(time(1:end),mag_z,'.')
title("Magnetic Field Z")
xlabel("Time (s)")
ylabel("Magnetic Field_z (T)")
%-------------------------------------------------------------%
figure('Name', 'Linear accelaration data');
subplot(2,2,1)
plot(time(1:end),linAccel_x,'.')
title("Linear Accelaration X")
xlabel("Time (s)")
ylabel("Linear_Accelaration_x (m/s^2)")

subplot(2,2,2)
plot(time(1:end),linAccel_y,'.')
title("Linear Accelaration Y")
xlabel("Time (s)")
ylabel("Linear_Accelaration_y (m/s^2)")

subplot(2,2,3)
plot(time(1:end),linAccel_z,'.')
title("Linear Accelaration Z")
xlabel("Time (s)")
ylabel("Linear_Accelaration_z (m/s^2)")
%-------------------------------------------------------------%
figure('Name', 'Gyro data');
subplot(2,2,2)
plot(time(1:end),angVel_x,'.')
title("Gyro X")
xlabel("Time (s)")
ylabel("Gyro_x (m/s^2)")

subplot(2,2,3)
plot(time(1:end),angVel_y,'.')
title("Gyro Y")
xlabel("Time (s)")
ylabel("Gyro_y (m/s^2)")

subplot(2,2,4)
plot(time(1:end),angVel_z,'.')
title("Gyro Z")
xlabel("Time (s)")
ylabel("Gyro_z (m/s^2)")
%-------------------------------------------------------------%
figure('Name', 'Linear accelaration distribution');
subplot(2,2,1)
hist(linAccel_x)
title("Linear Accelaration X")
xlabel("Time (s)")
ylabel("Linear_Accelaration_x distribution(m/s^2)")
mean_x = mean(linAccel_x);
std_x = std(linAccel_x);
median_x = median(linAccel_x);
text(0.7, 0.8, ['Mean = ' num2str(mean_x)], 'Units', 'normalized')
text(0.7, 0.85, ['Std = ' num2str(std_x)], 'Units', 'normalized')
text(0.7, 0.9, ['Median = ' num2str(median_x)], 'Units', 'normalized')

subplot(2,2,2)
hist(linAccel_y)
title("Linear Accelaration Y")
ylabel("Linear_Accelaration_y distribution(m/s^2)")
mean_y = mean(linAccel_y);
std_y = std(linAccel_y);
median_y = median(linAccel_y);
text(0.6, 0.6, ['Mean = ' num2str(mean_y)], 'Units', 'normalized')
text(0.6, 0.7, ['Std = ' num2str(std_y)], 'Units', 'normalized')
text(0.6, 0.8, ['Median = ' num2str(median_y)], 'Units', 'normalized')

subplot(2,2,3)
hist(linAccel_z)
title("Linear Accelaration Z")
ylabel("Linear_Accelaration_z distribution(m/s^2)")
mean_z = mean(linAccel_z);
std_z = std(linAccel_z);
median_z = median(linAccel_z);
text(0.7, 0.9, ['Mean = ' num2str(mean_z)], 'Units', 'normalized')
text(0.7, 0.8, ['Std = ' num2str(std_z)], 'Units', 'normalized')
text(0.7, 1, ['Median = ' num2str(median_z)], 'Units', 'normalized')
%-------------------------------------------------------------%
figure('Name', 'Gyro data distribution x');
hist(angVel_x)
title("Gyro X")
ylabel("Gyro_x distribution")
gyro_x = mean(angVel_x);
gyro_x = std(angVel_x);
text(0.7, 0.9, ['Mean = ' num2str(gyro_x)], 'Units', 'normalized')
text(0.7, 0.8, ['Std = ' num2str(gyro_x)], 'Units', 'normalized')

figure('Name', 'Gyro data distribution y');
hist(angVel_y)
title("Gyro Y")
ylabel("Gyro_y distriibution")
gyro_y = mean(angVel_y);
gyro_y = std(angVel_y);
text(0.7, 0.9, ['Mean = ' num2str(gyro_y)], 'Units', 'normalized')
text(0.7, 0.8, ['Std = ' num2str(gyro_y)], 'Units', 'normalized')

subplot(2,2,3)
hist(angVel_z)
title("Gyro Z")
ylabel("Gyro_z distribution")
gyro_z = mean(angVel_z);
gyro_z = std(angVel_z);
text(0.7, 0.9, ['Mean = ' num2str(gyro_z)], 'Units', 'normalized')
text(0.7, 0.8, ['Std = ' num2str(gyro_z)], 'Units', 'normalized')
%----------------------------------------------------------------------%
% Extract roll, pitch, and yaw angles from eul matrix
roll = eul(:,3);
pitch = eul(:,2);
yaw = eul(:,1);

% Plot roll against time
figure('Name', 'Roll,pitch and yaw');
subplot(2,2,1)
plot(time, roll, 'r');
xlabel('Time (s)');
ylabel('Roll (rad)');

% Plot pitch against time
subplot(2,2,2)
plot(time, pitch, 'g');
xlabel('Time (s)');
ylabel('Pitch (rad)');

% Plot yaw against time
subplot(2,2,3)
plot(time, yaw, 'b');
xlabel('Time (s)');
ylabel('Yaw (rad)');







