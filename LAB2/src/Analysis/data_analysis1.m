
%reading data from 4 different rosbag files and using select to filter
%messages in the topic "gps"
open_stat = select(rosbag('stationary_centennial.bag'),'Topic','/gps');
open_move = select(rosbag('walking_centennial.bag'),'Topic','/gps');
occl_move = select(rosbag('walking_berakhis.bag'),'Topic','/gps');
occl_stat = select(rosbag('stationary_berakhis.bag'),'Topic','/gps');

% Reads messages from rosbag and formats data as a structure 
open_st = readMessages(open_stat,'DataFormat','struct');
open_mo = readMessages(open_move,'DataFormat','struct');
occl_st = readMessages(occl_stat,'DataFormat','struct');
occl_mo = readMessages(occl_move,'DataFormat','struct');


 %using cellfun to store data as arrays for Northing,Easting ,Altitude,time
 %and fix quality for Open area stationary data collection
open_s_E = cellfun(@(m) double(m.UTMEasting),open_st);
open_s_N = cellfun(@(m) double(m.UTMNorthing),open_st);
open_s_A = cellfun(@(m) double(m.Altitude),open_st);
open_s_T = cellfun(@(m) double(m.Header.Stamp.Sec),open_st);
open_s_Q = cellfun(@(m) double(m.FixQuality),open_st);

%using cellfun to store data as arrays for Northing,Easting ,Altitude,time
 %and fix quality for open area walking data collection
 open_m_E =cellfun(@(m) double(m.UTMEasting),open_mo);
 open_m_N =cellfun(@(m) double(m.UTMNorthing),open_mo);
 open_m_A =cellfun(@(m) double(m.Altitude),open_mo);
open_m_T = cellfun(@(m) double(m.Header.Stamp.Sec),open_mo);
open_m_Q = cellfun(@(m) double(m.FixQuality),open_mo);

 %using cellfun to store data as arrays for Northing,Easting ,Altitude,time
 %and fix quality for Occluded area stationary data collection
occl_s_E = cellfun(@(m) double(m.UTMEasting),occl_st);
occl_s_N = cellfun(@(m) double(m.UTMNorthing),occl_st);
occl_s_A = cellfun(@(m) double(m.Altitude),occl_st);
occl_s_T = cellfun(@(m) double(m.Header.Stamp.Sec),occl_st);
occl_s_Q = cellfun(@(m) double(m.FixQuality),occl_st);

 %using cellfun to store data as arrays for Northing,Easting ,Altitude,time
 %and fix quality for Occluded area walking data collection
occl_m_E = cellfun(@(m) double(m.UTMEasting),occl_mo);
occl_m_N = cellfun(@(m) double(m.UTMNorthing),occl_mo);
occl_m_A = cellfun(@(m) double(m.Altitude),occl_mo);
occl_m_T = cellfun(@(m) double(m.Header.Stamp.Sec),occl_mo);
occl_m_Q = cellfun(@(m) double(m.FixQuality),occl_mo);

%Open area stationary data analysis
last_index_op = length(open_s_N);
last_val_op_e = open_s_E(last_index_op);
last_val_op = open_s_N(last_index_op);
mse_sum_N = 0;
mse_sum_E = 0;
for i = 1:last_index_op
    true_E = last_val_op_e-min(open_s_E); % last value is taken as the known value
    true_N = last_val_op-min(open_s_N); % last value is taken as the known value 

    % Measured easting and northing coordinates of stationary GPS receiver
    meas_E = open_s_E(i)-min(open_s_E);
    meas_N = open_s_N(i)-min(open_s_N);

    % Calculate squared differences and add to sum
    diff_E = true_E - meas_E;
    diff_N = true_N - meas_N;
   mse_sum_N = mse_sum_N + (diff_N^2);
    mse_sum_E = mse_sum_E + (diff_E^2);
end
mse_mean_N = mse_sum_N/ last_index_op;
mse_mean_E = mse_sum_E/ last_index_op;
rmse_N = sqrt(mse_mean_N);
rmse_E = sqrt(mse_mean_E);



disp("RMSE for northing  for stationary GPS data: " + rmse_N + " m");
disp("RMSE for easting for stationary GPS data: " + rmse_E + " m")





figure('Name', 'Open area Stationary data collection');
title("Stationary Recording - Clear")
fixq = open_s_Q == 4;%fix quality is 4 for fix and 5 for float
%Plotting scatter plots for Easting vs Northing for open area stationary data collection
subplot(2,2,1)
plot(open_s_E(fixq)-min(open_s_E), open_s_N(fixq)-min(open_s_N), 'g.',...
    open_s_E(~fixq)-min(open_s_E), open_s_N(~fixq)-min(open_s_N), 'r.') %plotting fix values in green and float values in red 
grid on
title("Northing vs. Easting")
xlabel("Easting (m)  ")
ylabel("Northing (m) " )
%Plotting Altitude vs Time 
subplot(2,2,2)
plot(open_s_T(fixq)-min(open_s_T), open_s_A(fixq), 'g.', ...
    open_s_T(~fixq)-min(open_s_T), open_s_A(~fixq), 'r.')
grid on
title("Altitude")
xlabel("Time (s)")
ylabel("Altitude (m)")
%Occluded area stationary data analysis
last_index_oc = length(occl_s_N);
last_val_oc_e = occl_s_E(last_index_oc);
last_val_oc = occl_s_N(last_index_oc);
oc_mse_sum_N = 0;
oc_mse_sum_E = 0;
for i = 1:last_index_oc
    oc_true_E = last_val_oc_e-min(occl_s_E); % last value is taken as the known value
    oc_true_N = last_val_oc-min(occl_s_N); % last value is taken as the known value 

    % Measured easting and northing coordinates of stationary GPS receiver
    oc_meas_E = occl_s_E(i)-min(occl_s_E);
    oc_meas_N = occl_s_N(i)-min(occl_s_N);

    % Calculate squared differences and add to sum
    oc_diff_E = oc_true_E - oc_meas_E;
    oc_diff_N = oc_true_N - oc_meas_N;
   oc_mse_sum_N = oc_mse_sum_N + (oc_diff_N^2);
    oc_mse_sum_E = oc_mse_sum_E + (oc_diff_E^2);
end
oc_mse_mean_N = oc_mse_sum_N/ last_index_oc;
oc_mse_mean_E = oc_mse_sum_E/ last_index_oc;
oc_rmse_N = sqrt(oc_mse_mean_N);
oc_rmse_E = sqrt(oc_mse_mean_E);



disp("RMSE for northing  for occluded stationary GPS data: " + oc_rmse_N + " m");
disp("RMSE for easting for occluded stationary GPS data: " + oc_rmse_E + " m")
%Plotting for occluded stationary data
figure('Name', 'Occluded area Stationary data collection');
title("Stationary Recording - Occluded")
fixq = occl_s_Q == 4;%fix quality is 4 for fix and 5 for float
%Plotting scatter plots for Easting vs Northing for occluded area stationary
%data collection
subplot(2,2,1)
plot(occl_s_E(fixq)-min(occl_s_E), occl_s_N(fixq)-min(occl_s_N), 'g.',...
    occl_s_E(~fixq)-min(occl_s_E), occl_s_N(~fixq)-min(occl_s_N), 'r.') %plotting fix values in green and float values in red 
grid on
title("Northing vs. Easting")
xlabel("Easting (m) ")
ylabel("Northing (m)")
%Plotting Altitude vs Time 
subplot(2,2,2)
plot(occl_s_T(fixq)-min(occl_s_T), occl_s_A(fixq), 'g.', ...
    occl_s_T(~fixq)-min(occl_s_T), occl_s_A(~fixq), 'r.')
grid on
title("Altitude")
xlabel("Time (s)")
ylabel("Altitude (m)")
%Occluded moving data 
figure('Name', 'Occluded area Moving data collection');
title("Moving Recording - Occluded")

fixq = occl_m_Q == 4;

% Subset data based on specified criteria
northing_thresh = 50;
easting_thresh = 80;
subset = occl_m_N- min(occl_m_N)< northing_thresh & occl_m_E - min(occl_m_E)< easting_thresh;

subplot(2,2,1)
plot(occl_m_E(fixq)-min(occl_m_E(fixq)), occl_m_N(fixq)-min(occl_m_N(fixq)), 'g.', ...
    occl_m_E(~fixq)-min(occl_m_E(~fixq)), occl_m_N(~fixq)-min(occl_m_N(~fixq)), 'r.')
grid on
title("Northing vs. Easting")
xlabel("Easting (m) ")
ylabel("Northing (m) ")

% compute best fit line and plot it
x = occl_m_E(subset)-min(occl_m_E(subset));
y = occl_m_N(subset)-min(occl_m_N(subset));
p = polyfit(x, y, 1);
xfit = [0 max(x)];
yfit = polyval(p, xfit);
hold on
plot(xfit, yfit, 'b-', 'LineWidth', 2)
hold off
% compute RMSE
yfit_all = polyval(p, x); %taking the polynomial values from the plot
residuals = y - yfit_all; %calculating residuals 
rmse = sqrt(mean(residuals.^2)); %calculating the root mean square error
disp("Root mean square error for Occluded moving data: " + rmse + " m");

subplot(2,2,2)
plot(occl_m_T(fixq)-min(occl_m_t), occl_m_A(fixq), 'g.', ...
    occl_m_T(~fixq)-min(occl_m_t), occl_m_A(~fixq), 'r.')

grid on
title("Altitude")
xlabel("Time (s)")
ylabel("Altitude (m)")

%Data Analysis for Moving data open area
figure('Name', 'Open area Moving data collection');
title("Moving Recording - Occluded")
fixq = open_m_Q == 4;
subplot(2,2,1)
plot(open_m_E-min(open_m_E), open_m_N-min(open_m_N), 'g.')
grid on
title("Northing vs. Easting")
xlabel("Easting (m) ")
ylabel("Northing (m) ")
% Fit ellipse to data points
x= open_m_E-min(open_m_E);
y=  open_m_N-min(open_m_N);
fit_ellipse(x,y)
hold on
plot(x,y,'b','LineWidth',1);
hold off
% compute RMSE
yfit_all = polyval(p, x); %taking the polynomial values from the plot
residuals = y - yfit_all; %calculating residuals 
rmse = sqrt(mean(residuals.^2)); %calculating the root mean square error
disp("Root mean square error for Open area moving data: " + rmse + " m");

subplot(2,2,2)
plot(open_m_T(fixq)-min(open_m_T), open_m_A(fixq), 'g.', ...
    open_m_T(~fixq)-min(open_m_T), open_m_A(~fixq), 'r.')
grid on
title("Altitude vs Time")
xlabel("Time (s)")
ylabel("Altitude(m)")
