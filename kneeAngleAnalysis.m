% starter script for Coralie to extract the data from the .txt files from the myRIO
% recording and calibrate
%
% Dependencies:
% - importMyRIO.m
% - calibrateIMU.m


%% Load Calibration Data
clc;clearvars

fs = 200; %sample rate 

%select the folder with the IMU data: "CoraliePilot_Subj_1"
folder = uigetdir();

%load all the calibration matrices 
R_all = readmatrix([folder '\Calibration.txt']);

%get the rotation matrix and gyro offset for IMU 1 and 2
R_1 = R_all(1:3,:);
offset1 = R_all(13,:);
R_2 = R_all(4:6,:);
offset2 = R_all(14,:);

%% Load Trial Data

% select FOLDER with the trial of interest to load 
trialFolder = uigetdir(folder,'Select Trial(s)');
[rawData, trialName, ~] = importMyRIO('Path',trialFolder);


%apply calibration to selected data
thighIMU = calibrateIMU(rawData(:,4:9), R_1, offset1);
shankIMU = calibrateIMU(rawData(:,10:15), R_2, offset2);
time = rawData(:,1);

%rotation matrix
R = rotx(-30);
thighIMUaccel = thighIMU(:, 1:3) * R;
thighIMUang = thighIMU(:, 4:6) * R;
shankIMUaccel = shankIMU(:, 1:3) * R;
shankIMUang = shankIMU(:, 4:6) * R;

% thighIMUaccel = thighIMU(:, 1:3);
% thighIMUang = thighIMU(:, 4:6);
% shankIMUaccel = shankIMU(:, 1:3);
% shankIMUang = shankIMU(:, 4:6);


%apply complimentary filter
[IMUPitch_shank, IMURoll_shank] = complementaryFilter(shankIMUaccel, shankIMUang,fs, 0.995);
[IMUPitch_thigh, IMURoll_thigh] = complementaryFilter(thighIMUaccel, thighIMUang,fs, 0.995);


knee_angle_roll = IMURoll_thigh - IMURoll_shank;
knee_angle_pitch = IMUPitch_thigh - IMUPitch_shank;

T = table(time, knee_angle_pitch);
writetable(T,'knee_angles.txt');
type knee_angles.txt

figure
plot(time, IMURoll_thigh)
hold on
plot(time, IMUPitch_thigh)
title("Thigh Data")
xlabel("Time (s)")
ylabel("Angle")
legend("Roll", "Pitch")
hold off

figure
plot(time, IMURoll_shank)
hold on
plot(time, IMUPitch_shank)
title("Shank Data")
xlabel("Time (s)")
ylabel("Angle")
legend("Roll", "Pitch")
hold off

figure
plot(time, knee_angle_roll)
title("Knee Roll Flexion Angle vs. Time", "FontSize", 15)
xlabel("Time (s)", "FontSize", 13)
ylabel("Knee Flexion Angle (^{o})", "FontSize", 13)

figure
plot(time, knee_angle_pitch)
title("Knee Pitch Flexion Angle vs. Time", "FontSize", 15)
xlabel("Time (s)", "FontSize", 13)
ylabel("Knee Flexion Angle (^{o})", "FontSize", 13)



