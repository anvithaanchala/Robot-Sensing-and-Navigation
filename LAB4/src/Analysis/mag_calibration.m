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
 %storing magnetometer data in mag_x ,mag_y and mag_z
mag_x = cell2mat(cellfun(@(m) [m.MagField.MagneticField_.X],Data_circles_imu,'UniformOutput',false));
mag_y = cell2mat(cellfun(@(m) [m.MagField.MagneticField_.Y],Data_circles_imu,'UniformOutput',false));
mag_z = cell2mat(cellfun(@(m) [m.MagField.MagneticField_.Z],Data_circles_imu,'UniformOutput',false));
% %Converting from teslas to gauss
mag_x = 100000*mag_x;
mag_y = 100000*mag_y;
mag_z = 100000*mag_z;
%changing from quaternions to roll,pitch and yaw

%time from header stamp
time = cellfun(@(m) str2double(strcat(num2str(m.Header.Stamp.Sec-Data_drive_imu{1}.Header.Stamp.Sec),'.',num2str(m.Header.Stamp.Nsec))),Data_drive_imu);
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










