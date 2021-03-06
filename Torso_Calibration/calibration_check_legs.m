clear all; close all; clc;

% Calibration legs with use of multiple points.

% parameters
offset_cal1 = -0.23116;
% offset_cal1 =   -0.229848003028694;
offset_cal2 = -0.22321;

%% collect data
IMU_base_start = 0.000;
data = [...  // imu , calipher, encoder;.. // goals
        -0.105, 0.53137, 0.25118;... // 0.1
        -0.400, 0.57177, 0.29875;... // 0.4
        -0.606, 0.59921, 0.33148;... // 0.6
        -0.910, 0.63855, 0.37887;... // 0.9
        -0.605, 0.59931, 0.33148;... // 0.6
        -0.406, 0.57185, 0.29875;... // 0.4
        -0.106, 0.53140, 0.25118];   %% 0.1
IMU_base_end = 0.008;
IMU_base = (IMU_base_start+IMU_base_end)/2;


%% calculate angle joint 0
n = length(data);                                   % number of points
% N = linspace(1,n,n);                                % measurement points
N = [0.1 0.4 0.6 0.9 0.6 0.4 0.1].';                % measurement points
th0_imu = imu_angle0_to_angle0(abs(data(:,1)-IMU_base));
% th0_imu = abs(data(:,1)-IMU_base);                  % angle by imu
th0_cal = spring1_to_angle0(data(:,2)+offset_cal1); % angle by caliphers
th0_enc = spindle1_to_angle0(data(:,3));            % angle by encoders

% plot difference
figure; 
plot(N,th0_imu,N,th0_cal,'--',N,th0_enc,'--'); grid on; xlabel('Measurement N');
ylabel('angle 0 [rad]'); legend('imu','calipher','encoder');

% plot difference
diff_enc = th0_imu-th0_enc;
diff_cal = th0_imu-th0_cal;
figure; 
plot(N,diff_enc,N,diff_cal,'--'); grid on; xlabel('Measurement N'); 
ylabel('difference in angle 0 [rad]'); legend('enc','cal');

figure; 
plot(N,diff_enc./th0_imu,N,diff_cal./th0_imu,'--'); grid on; xlabel('Measurement N'); 
ylabel('rel. difference in angle 0 [rad]'); legend('enc','cal');

%% calculate offset change
cal = data(:,2)+offset_cal1;
cal_imu = angle0_to_spring1(th0_imu);
diff_cal_imu = cal_imu-cal;
figure; plot(N,diff_cal_imu); grid on; xlabel('measurement position');
ylabel('calipher length difference [m]');

avg_diff = mean(diff_cal_imu);

offset_cal1_new = offset_cal1+avg_diff;

s = sprintf(['\n The new offset of calipher 1 is:\n',...
    '\t %7.5f,\n'],offset_cal1_new);
disp(s)



