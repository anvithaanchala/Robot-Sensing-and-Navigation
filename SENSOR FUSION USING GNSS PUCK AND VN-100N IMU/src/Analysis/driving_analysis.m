%Created by Anvitha Anchala 
%Mag calibration :soft and hard iron distortion
%Transformation 1 : Fit ellipse 
%Transformation 2 : Align the centre using affine transformation 
%Trasnformation 3 : Fit into a circle 
%selecting a rosbag file with the topic specified
 Data_driving_imu = select(rosbag('data_driving.bag'),'Topic','/imu');
 Data_driving_gps = select(rosbag('data_driving.bag'),'Topic','/gps');
 Data_going_in_circles_imu= select(rosbag('data_going_in_circles.bag'),'Topic','/imu');
 Data_going_in_circles_gps= select(rosbag('data_going_in_circles.bag'),'Topic','/gps');
 %reading messages and storing them in a struct 
 Data_drive_imu =readMessages(Data_driving_imu,'DataFormat','struct');
 Data_drive_gps=readMessages(Data_driving_gps,'DataFormat','struct');
 Data_circles_gps=readMessages(Data_going_in_circles_gps,'DataFormat','struct');
 Data_circles_imu=readMessages(Data_going_in_circles_imu,'DataFormat','struct');
 %---------------Driving in circlesdata-----------------------------------%
 %storing magnetometer data in mag_x ,mag_y and mag_z
mag_x = cell2mat(cellfun(@(m) [m.MagField.MagneticField_.X],Data_circles_imu,'UniformOutput',false));
mag_y = cell2mat(cellfun(@(m) [m.MagField.MagneticField_.Y],Data_circles_imu,'UniformOutput',false));
mag_z = cell2mat(cellfun(@(m) [m.MagField.MagneticField_.Z],Data_circles_imu,'UniformOutput',false));
%Converting from teslas to gauss
mag_x = 100000*mag_x;
mag_y = 100000*mag_y;
mag_z = 100000*mag_z;
%------Driving data--------------------%
%Getting magnetometer data 
d_mag_x = cell2mat(cellfun(@(m) [m.MagField.MagneticField_.X],Data_drive_imu,'UniformOutput',false));
d_mag_y = cell2mat(cellfun(@(m) [m.MagField.MagneticField_.Y],Data_drive_imu,'UniformOutput',false));
d_mag_z = cell2mat(cellfun(@(m) [m.MagField.MagneticField_.Z],Data_drive_imu,'UniformOutput',false));
%Converting from teslas to gauss
d_mag_x = 100000*d_mag_x;
d_mag_y = 100000*d_mag_y;
d_mag_z = 10000*d_mag_z;
%changing from quaternions to roll,pitch and yaw
quat = cell2mat(cellfun(@(m) [m.Imu.Orientation.W m.Imu.Orientation.X m.Imu.Orientation.Y m.Imu.Orientation.Z], Data_drive_imu, 'UniformOutput',false));
eul = quat2eul(quat);
imu_yaw = eul(:,3);
%Angular velocity from gyroscope
angVel_x = cell2mat(cellfun(@(m) [m.Imu.AngularVelocity.X],Data_drive_imu,'UniformOutput',false));
angVel_y = cell2mat(cellfun(@(m) [m.Imu.AngularVelocity.Y],Data_drive_imu,'UniformOutput',false));
angVel_z = cell2mat(cellfun(@(m) [m.Imu.AngularVelocity.Z],Data_drive_imu,'UniformOutput',false));
%Linear accelaration from accelorometer 
linAccel_x = cell2mat(cellfun(@(m) [m.Imu.LinearAcceleration.X ],Data_drive_imu,'UniformOutput',false));
linAccel_y = cell2mat(cellfun(@(m) [m.Imu.LinearAcceleration.Y],Data_drive_imu,'UniformOutput',false));
linAccel_z = cell2mat(cellfun(@(m) [m.Imu.LinearAcceleration.Z],Data_drive_imu,'UniformOutput',false));
%time from header stamp
time = cellfun(@(m) str2double(strcat(num2str(m.Header.Stamp.Sec-Data_drive_imu{1}.Header.Stamp.Sec),'.',num2str(m.Header.Stamp.Nsec))),Data_drive_imu);
%-----------------------GPS data---------------------%
gps_Easting = cellfun(@(m) double(m.UTMEasting),Data_drive_gps);
gps_Northing = cellfun(@(m) double(m.UTMNorthing),Data_drive_gps);
gps_Altitude = cellfun(@(m) double(m.Altitude),Data_drive_gps);
gps_Latitude=cellfun(@(m) double(m.Latitude),Data_drive_gps);
gps_Longitude=cellfun(@(m) double(m.Longitude),Data_drive_gps);
gps_secs=cellfun(@(m) double(m.Header.Stamp.Sec),Data_drive_gps);
gps_nsecs=cellfun(@(m) double(m.Header.Stamp.Nsec),Data_drive_gps);
%time from gps 
time_gps = gps_secs + 10^(-9)*gps_nsecs;
time_gps =  time_gps- time_gps(1);
gps_Easting=gps_Easting-gps_Easting(1);
gps_Northing=gps_Northing-gps_Northing(1);
gps_combine = [gps_Easting,gps_Northing];

%--------Magnetic Calibration----------------------
%Used fit ellipse function to obtain x offset ,y offset and biasing angle 
figure(1);
plot(mag_x,mag_y);
hold on;
ellipsefit= fit_ellipse(mag_x,mag_y);
xlabel('magnetic field x (Gauss)')
ylabel('magnetic field y (Gauss)')
grid on;
title('magnetic field in y vs magnetic field in x')

theta = -ellipsefit.phi;
scale = ellipsefit.short_axis/ellipsefit.long_axis;
x_offset = ellipsefit.X0_in;
y_offset = ellipsefit.Y0_in;
centre_x=ellipsefit.a;
centre_y=ellipsefit.b;
rotation_mat = [cos(theta-0.1464331242) sin(theta-0.1464331242);
     -sin(theta-0.1464331242) cos(theta-0.1464331242)];
scaling_mat = [scale 0;0 1];
% Soft Iron Scaling Correction
% Combined Soft Iron Correction
correction_mat = rotation_mat*scaling_mat;
B = transpose([mag_x-x_offset,mag_y-y_offset]);
corrected_mag = correction_mat*B;
cmag_x = corrected_mag(1,:);
cmag_y = corrected_mag(2,:);
%plot of mag x vs y before and after calibration 
figure(2);
plot(mag_x,mag_y);
hold on
[r,varargout]=circfit(cmag_x,cmag_y);
plot(cmag_x,cmag_y);
grid on;
axis equal;
title("Soft Iron and Hard Iron Magnetometer Correction");
xlabel("Magnetic Field, X (Gauss)");
ylabel("Magnetic Field, Y (Gauss)");
legend("Raw Magnetometer","Corrected Magnetometer");

%obtained si and hi offsets from driving in circles data
%we will use that for driving data 
%Magnetometer calibration for driving : We take the same hard iron offset
%as it's independent of the surrounding and soft iron changes are
%negligible 

% Soft Iron Scaling Correction
% Combined Soft Iron Correction

rotation_mat_1= [cos(theta-0.1464331242) -sin(theta-0.1464331242);
     sin(theta-0.1464331242) cos(theta-0.1464331242)];
dB = rotation_mat_1*transpose([d_mag_x-x_offset,d_mag_y-y_offset]);
dcorrection_mat = rotation_mat*scaling_mat;
dcorrected_mag = dcorrection_mat*dB;
dcmag_x = dcorrected_mag(1,:);
dcmag_y = dcorrected_mag(2,:);
%calculating raw and corrected yaw
magd_yaw_raw = (atan2(-d_mag_y,d_mag_x));
if(d_mag_y<0)
     magd_yaw_raw=magd_yaw_raw+pi;
end
magd_yaw_corr = transpose((atan2(-dcmag_y,dcmag_x)));
figure(3)
plot(time,unwrap(magd_yaw_corr));
grid on;
hold on;
plot(time,unwrap(magd_yaw_raw));
hold off;
legend('corrected magnetometer yaw','raw magnetometer yaw')
xlabel('time (seconds)')
ylabel('yaw (radians)')
title('Yaw vs Time')
%-----------------Gyroscope calculations-----------------
%yaw angle from gyroscope 
gyro_yaw = cumtrapz(time,angVel_z);
magd_yaw_corr = magd_yaw_corr - magd_yaw_corr(1);
%Gyro yaw vs mag yaw plot
figure(4)
plot(time,unwrap(gyro_yaw))
hold on;
grid on;
plot(time, unwrap(magd_yaw_corr))
legend('gyroscope yaw', 'magnetometer yaw')
xlabel('time (seconds)')
ylabel('yaw (radians)')
title('Yaw from Magnetometer and Yaw integrated from Gyro')
hold off;

%-------------------Using low pass high pass and comp filter----------------------------------------%
%low pass ,high pass and complementary filter 
%using low pass filter on magnetometer yaw 
%using hpf on yaw integrated from gyro 
%using complementary filter on both 
%Plotting all  three in the same graph
low_magd_yaw_corr= lowpass((unwrap(magd_yaw_corr)),0.0001,40);
high_gyro_yaw=highpass(unwrap(gyro_yaw),0.0225,40);
compFilter =unwrap(low_magd_yaw_corr+high_gyro_yaw);
figure(5);
plot(time,low_magd_yaw_corr);
grid on;
hold on;
plot(time,high_gyro_yaw);
hold on;
plot(time,unwrap(compFilter));
legend('lpf mag yaw', 'hpf gyro yaw','comp filter')
xlabel('time (seconds)')
ylabel('yaw (radians)')
title('LPF yaw,HPF yaw and Complimentary filter')
hold off;


%---------------------------Velocity calculations------------------------------------------%
%velocity estimate from accelerometer and gps 
%integrate acceleration x from imu using cumtrapz as we moved in the
%positive x directionn 
imu_velocity = cumtrapz(time,linAccel_x);
%velocity from gps using utm northing and easting 
velocity_gps= zeros(length(time_gps)-1,1);
for i = 1:length(time_gps)-1
    velocity_gps(i) = norm(gps_combine(i+1,:)-gps_combine(i,:))/(time_gps(i+1)-time_gps(i));
end
velocity_gps = [0,transpose(velocity_gps)];
velocity_gps= transpose(velocity_gps);
vel_gps_truesiz = velocity_gps;
velocity_gps = interp(velocity_gps,40);

figure(6)
plot(velocity_gps)
grid on; 
hold on;
plot(imu_velocity)
xlabel('time (seconds)')
ylabel('velocity (meter/second)')
title('Velocity estimate from GPS and IMU before adjustment')
legend('velocity gps','velocity imu')
hold off;
% Plotting acceleration 
 figure(7)
 plot(time,linAccel_x)
xlabel('time (seconds)')
ylabel('Acceleration (meter/second^2)')
title('Acceleratuon estimate from IMU  ')
legend('Acceleration imu')

%Correction of forward velocity using imu 
%Remove the forward bias by taking the mean and subtracting it 

%------------------------------Correcting velocity---------------------------------------------%
%% 
% subtract the bias from the stationary vehicle
% assume that imu_velocity contains the velocity readings from the IMU
% find the indices where the velocity is close to zero
zero_vel_indices = find(abs(velocity_gps) < 0.00759); % adjust the threshold as necessary

for i = 1:length(zero_vel_indices)
    if i==length(zero_vel_indices)-1
        mean_bias = mean(linAccel_x(zero_vel_indices(i,1):zero_vel_indices(i+1,1)));
        linAccel_x_corrected(zero_vel_indices(i,1):zero_vel_indices(i+1,1)) = linAccel_x(zero_vel_indices(i,1):zero_vel_indices(i+1,1)) - mean_bias; 
        break
    
    else
        mean_bias = mean(linAccel_x(zero_vel_indices(i,1):zero_vel_indices(i+1,1)));
        linAccel_x_corrected(zero_vel_indices(i,1):zero_vel_indices(i+2,1)) = linAccel_x(zero_vel_indices(i,1):zero_vel_indices(i+2,1))-mean_bias;
    end
end


% get a corrected velocity
imu_v = cumtrapz(transpose(linAccel_x_corrected*(1/40)));
%synchronize IMU and GPS velocities by aligning peaks
[~, lag] = xcorr(imu_v, velocity_gps);
[~, I] = max(abs(xcorr(imu_v, velocity_gps)));
lagDiff = lag(I);

%shift IMU velocity signal by lag difference
if lagDiff > 0
    imu_v_shifted = [imu_v(lagDiff+1:end); zeros(lagDiff,1)];
else
    imu_v_shifted = [zeros(abs(lagDiff),1); imu_v(1:end+lagDiff)];
end

%plot corrected velocities




% % calculate the scaling factor
scale_factor = mean(velocity_gps) / mean(imu_v_shifted);

% scale the IMU velocity
imu_v_shifted = imu_v_shifted * scale_factor;
% 
% % plot the scaled velocity
figure(8)
plot(time(1:end-9), imu_v_shifted)
hold on; grid on;
plot(time(1:end-9), velocity_gps)
title("Forward Velocity Comparison - Corrected IMU (Scaled)")
xlabel("Time (s)")
ylabel('Velocity (m/s)')
legend("IMU (Scaled)","GPS velocity")


%-------------------------------Displacement calc--------------------
%displacement from imu and gps 
displacement_imu=cumtrapz(imu_v_shifted);
displacement_gps=cumtrapz(velocity_gps);
%plot corrected displacements
figure(9)
plot(time(1:end-9), displacement_imu)
hold on; grid on;
plot(time(1:end-9), displacement_gps)
title("Displacement Comparison -IMU vs GPS")
xlabel("Time (s)")
ylabel('Displacement(m)')
legend("IMU","GPS displacement")

%Dead reckoning of IMU 
%comparing wx with y (yaw rate*velocity with lateral x acceleration 

wX_dot = angVel_z(1:end-9).*imu_v_shifted;
y_ddot = linAccel_y;

figure(10)

grid on
plot(y_ddot,'.')
hold on;
plot(wX_dot,'.')
title("Lateral Acceleration Comparison")
xlabel("Time (s)")
ylabel("Acceleration (m/s^2)")
legend("y''","wX'")
%---------------------------------------------------------%
%Finding northing and easting data from imu complimentary filter 
%plotting imu northing and easting along with gps northing and easting and
%comparing the path 
s1 = sin(compFilter(1:end-9));
c1 = cos(compFilter(1:end-9));
s2=sin(magd_yaw_corr(1:end-9));
c2=cos(magd_yaw_corr(1:end-9));
%using comp filter
ve = -imu_v_shifted.*c1;
vn = imu_v_shifted.*s1;
xe = cumtrapz(time(1:end-9),ve);
xn = cumtrapz(time(1:end-9),vn);
xe =transpose(xe);
xn=transpose(xn);
xe=xe-xe(1);
xn=xn-xn(1);
%using corrected mag yaw
ve1 = -imu_v_shifted.*c2;
vn1 = imu_v_shifted.*s2;
xe1 = cumtrapz(time(1:end-9),ve1);
xn1 = cumtrapz(time(1:end-9),vn1);
xe1 =transpose(xe);
xn1=transpose(xn);

figure(11)
plot(gps_Easting, gps_Northing)
hold on
plot(xe, xn)
grid on
xlabel("Easting(m)")
ylabel("Northing(m)")
legend("gps", "imu")
title("Dead reckoning :Complimentary filter yaw")



figure(12)
plot(gps_Easting, gps_Northing)
hold on
plot(xe1, xn1)
grid on
xlabel("Easting(m)")
ylabel("Northing(m)")
legend("gps", "imu")
title("Dead reckoning :Corrected Magnetometer yaw")
    
    



