% Select ROS bag file and topic
bag = rosbag('LocationA.bag');
imu_topic = select(bag,'Topic','/vectornav');

% Read messages from selected topic
imu_msgs = readMessages(imu_topic,'DataFormat','struct');

% Extract gyroscope data from messages and store in matrix
gyro_data = zeros(length(imu_msgs), 3);
for i = 1:length(imu_msgs)
    data_str = imu_msgs{i}.Data;
    tokens = strsplit(data_str, ',');

    % Check that tokens cell array has at least 13 elements
    if length(tokens) < 13
        warning('Skipping message %d: not enough tokens', i);
        continue;
    end

    % Extract gyroscope data for x, y, and z axes
    gyro_x = str2double(tokens{11});
    gyro_y = str2double(tokens{12});
    gyro_z_str = tokens{13};
    gyro_z_str_parts = strsplit(gyro_z_str, '*');

    % Check that gyro_z_str_parts cell array has at least 1 element
    if length(gyro_z_str_parts) < 1
        warning('Skipping message %d: not enough gyro_z_str_parts', i);
        continue;
    end

    gyro_z = str2double(gyro_z_str_parts{1});

    % Convert gyroscope data to radians per second
    gyro_data(i, :) = [gyro_x, gyro_y, gyro_z] * (pi/180);

end

% Calculate Allan deviation for gyro x
total_time = length(gyro_data) / 40; % compute total duration of data in seconds
tau = logspace(-2, log10(total_time/2), 20); % set 20 integration times between 0.01s and half of the total duration
[ad_x, ade_x, taus_x] = allan(gyro_data(:, 1)', tau);

% Plot Allan deviation for gyro x
figure;
subplot(2,2,1)
loglog(taus_x, ad_x);
xlabel('Integration time (s)');
ylabel('Allan deviation (rad/s)');
title('Allan deviation for gyro x');

% Calculate noise parameters for gyro x
N_x = sqrt(ad_x(1:end-2).*ad_x(3:end)./ad_x(2:end-1).^2);
K_x = sqrt(ad_x(2:end-1)./ad_x(1:end-2));
B_x = sqrt(ad_x(3:end)./ad_x(2:end-1));

% Print noise parameters for gyro x
fprintf('Noise parameters for gyro x:\n');
fprintf('N = %f, K = %f, B = %f\n', N_x(1), K_x(1), B_x(1));

% Calculate Allan deviation for gyro y
[ad_y, ade_y, taus_y] = allan(gyro_data(:, 2)', tau);

% Plot Allan deviation for gyro y
subplot(2,2,2)
loglog(taus_y, ad_y);
xlabel('Integration time (s)');
ylabel('Allan deviation (rad/s)');
title('Allan deviation for gyro y');

% Calculate noise parameters for gyro y
N_y = sqrt(ad_y(1:end-2).*ad_y(3:end)./ad_y(2:end-1).^2);
K_y = sqrt(ad_y(2:end-1)./ad_y(1:end-2));
B_y = sqrt(ad_y(3:end)./ad_y(2:end-1));

% Print noise parameters for gyro y
fprintf('Noise parameters for gyro y:\n');
fprintf('N = %f, K = %f, B = %f\n', N_y(1), K_y(1), B_y(1));

% Calculate Allan deviation for gyro z
[ad_z, ade_z, taus_z] = allan(gyro_data(:, 3)', tau);

% Plot Allan deviation for gyro z
subplot(2,2,3)
loglog(taus_z, ad_z);
xlabel('Integration time (s)');
ylabel('Allan deviation (rad/s)');
title('Allan deviation for gyro z');

% Calculate noise parameters for gyro z
N_z = sqrt(ad_z(1:end-2).*ad_z(3:end)./ad_z(2:end-1).^2);
K_z = sqrt(ad_z(2:end-1)./ad_z(1:end-2));
B_z = sqrt(ad_z(3:end)./ad_z(2:end-1));

% Print noise parameters for gyro z
fprintf('Noise parameters for gyro z:\n');
fprintf('N = %f, K = %f, B = %f\n', N_y(1), K_y(1), B_y(1));

t = (1:length(gyro_data))/40; % time vector in seconds
figure;
subplot(3,1,1)
plot(t, gyro_data(:,1));
xlabel('Time (s)');
ylabel('Gyro x (rad/s)');
title('Gyro x vs Time');
subplot(3,1,2)
plot(t, gyro_data(:,2));
xlabel('Time (s)');
ylabel('Gyro y (rad/s)');
title('Gyro y vs Time');
subplot(3,1,3)
plot(t, gyro_data(:,3));
xlabel('Time (s)');
ylabel('Gyro z (rad/s)');
title('Gyro z vs Time');