%File: complementaryFilter.m
%Adapted from code by Liam Foulger
%Date created: 2020-10-30
%Last updated: 2023-03-06 
%
%[IMUPitch, IMURoll] = complementaryFilter(accel, gyro,fs, weight,NamePairArguments)
%
%Complementary filter that combines the accelerometer and gyro data as
%as seen in general formula:
%angle = (weight of gyro data)*(angle+gyrData*dt) + (1-weight)(accData)
%
%**assumes North/Forward(x)-East/Right(y)-Down(z) Orientation of sensor data**
%
%input units:
% - accelerometer data (m/s^2)
% - gyro data (deg/s)
% - sensor sampling rate (Hz)
% - weight of gyroscope data for comp filter (optional; default is 0.995 since best results found with that,
%   but can be 0.95 to <1)
% - NamePairArguments (optional):
%    - 'RemoveOffset': 1 (true) or 0 (false; default). Remove the offset
%    from the gyroscope before angle estimation. Ideally the offset should
%    be removed (or determined to be removed post hoc) via a calibration 
%    procedure before the data collection. 
%    - 'Showplots': 1 (true) or 0 (false; default). True if you want to
%    show the plots of the results
%    - 'FilterCutoff': Cutoff for optional butterworth filter (Hz). Default is 0 (no filter
%    applied).
%    - 'FilterOrder': Order for optional butterworth filter. Default is 2.
%    - 'FilterType':  Filter type for optional butterworth filter. Options
%    are: 'low' (default), 'high','stop', or 'bandpass'
%outputs:
% - tilt in degrees: Pitch (around Y axis) & Roll (around X axis)

function [IMUPitch, IMURoll] = complementaryFilter(accel, gyro,fs, weight, NamePairArguments)

    arguments 
        accel double
        gyro double
        fs double 
        weight (1,1) {mustBeNumeric} = 0.995
        NamePairArguments.RemoveOffset (1,1) {mustBeNumeric} = 0
        NamePairArguments.Showplots (1,1) {mustBeNumeric} = 0
        NamePairArguments.FilterCutoff (1,:) {mustBeNumeric} = 0
        NamePairArguments.FilterOrder (1,1) {mustBeNumeric} = 2
        NamePairArguments.FilterType (1,1) string = "low"
    end
   
    %get predicted angle from accelerometer only
    [pitchAccel, rollAccel] = getAccAngle(accel);
    
    %removes gyro offset (if selected) & switches sign 
    [pitchGyro,rollGyro] = convertGyro(gyro, NamePairArguments);
    
    %step-by-step complimentary filter 
    pitch = zeros(length(rollGyro),1);
    roll = zeros(length(rollGyro),1);
    pitch(1) = mean(pitchAccel(1:10));
    roll(1) = mean(rollAccel(1:10));
    for jj = 2:length(rollGyro)
        netAccel = sqrt(accel(jj,1)^2 + accel(jj,2)^2 +accel(jj,3)^2);
        if (netAccel > 4.9 && netAccel < 19.6) && ~isnan(pitchAccel(jj)) && ~isnan(pitchGyro(jj-1))
            %this checks if there is too much acceleration (so you would
            %ignore the accelerometer estimate), and that both accel and
            %gyro measures are not NaNs
            pitch(jj) = (pitch(jj-1) + pitchGyro(jj-1)/fs)*weight + pitchAccel(jj)*(1-weight);
            roll(jj) = (roll(jj-1) + rollGyro(jj-1)/fs)*weight + rollAccel(jj)*(1-weight);
        elseif ~isnan(pitchGyro(jj-1))
            pitch(jj) = (pitch(jj-1) + pitchGyro(jj-1)/fs);
            roll(jj) = (roll(jj-1) + rollGyro(jj-1)/fs);
        else
            pitch(jj) = pitch(jj-1);
            roll(jj) = roll(jj-1);
        end
    end
    
    %Plotting to visualize 
    if NamePairArguments.Showplots
        %calculate gyro only angle 
        gyro_anglePitch = zeros(length(gyro),1);
        gyro_angleRoll = zeros(length(gyro),1);
        for ii = 1:length(gyro)
            if ii == 1
                %starting at same point as accelerometer to account for initial IMU tilt
                gyro_anglePitch(ii) = mean(pitchAccel(1:10));
                gyro_angleRoll(ii) = mean(rollAccel(1:10));
            elseif ~isnan(pitchGyro(ii-1))
                gyro_anglePitch(ii) = gyro_anglePitch(ii-1) + pitchGyro(ii-1)/fs;
                gyro_angleRoll(ii) = gyro_angleRoll(ii-1) + rollGyro(ii-1)/fs;
            else
                gyro_anglePitch(ii) = gyro_anglePitch(ii-1);
                gyro_angleRoll(ii) = gyro_angleRoll(ii-1);
            end
        end
        
        t = (1/fs):(1/fs):(ii*(1/fs));
        %plotting comparison
        figure
        subplot(2,1,1)
        plot(t,pitchAccel,'c')
        hold on
        plot(t,gyro_anglePitch,'r')
        plot(t,pitch,'k')
        legend('Accelerometer Estimation','Gyroscope Estimation','Complementary Filter')
        legend('boxoff')
        title('Pitch')
        ylabel('Degrees')
        xlabel('Time (s)')
        box off
        subplot(2,1,2)
        plot(t,rollAccel,'c')
        hold on
        plot(t,gyro_angleRoll,'r')
        plot(t,roll,'k')
        title('Roll')
        xlabel('Time (s)')
        box off
    end
    
    %apply filter if selected
    if NamePairArguments.FilterCutoff > 0
        [b,a] = butter(NamePairArguments.FilterOrder, NamePairArguments.FilterCutoff./(fs/2),NamePairArguments.FilterType);
        IMUPitch = filtfilt(b,a,pitch);
        IMURoll = filtfilt(b,a,roll);
    else
        IMUPitch = pitch;
        IMURoll = roll;
    end
end

function [pitchAccel, rollAccel] = getAccAngle(accel)
    %returns the angle of the gravity vector from the accelerometer data
    %(m/s^2)
    %assumes North(x)-East(y)-Down(z) Orientation of sensor data

    %pitch: forward: +
%     pitch = atan2d(accel(:,1), -accel(:,3));
    pitch = atan2d(accel(:,1), sqrt(accel(:,3).^2 + accel(:,2).^2));
    %roll: right(east): +
%     roll = atan2d(accel(:,2), -accel(:,3));
    roll = atan2d(accel(:,2), sqrt(accel(:,3).^2 + accel(:,1).^2));
    pitchAccel = pitch;
    rollAccel = -roll; 
end

function [pitchGyro,rollGyro] = convertGyro(gyro, options)
    %converts gyroscope data to deg/s and removes sensor bias 
    %assumes North(x)-East(y)-Down(z) Orientation of sensor data
    %input: gyroscope data (rad/s)
    %output: pitch (fwd: +) and roll (right: +) velocities (degrees/s),
    %with bias removed
    
    if options.RemoveOffset
        x_offset = mean(gyro(:,1),'omitnan');
        y_offset = mean(gyro(:,2),'omitnan');

        pitch = gyro(:,2) - y_offset;
        roll = gyro(:,1) - x_offset;
    else 
        pitch = gyro(:,2);
        roll = gyro(:,1);
    end
       
    %need to switch sign of y angular velocity to get foward pitch: + 
    pitchGyro = pitch;
    rollGyro = roll;

end
