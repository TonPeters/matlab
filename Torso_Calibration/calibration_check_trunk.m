clear all; close all; clc;

% Calibration legs with use of multiple points.

% parameters
offset_cal1 = -0.23116;
% offset_cal1 =   -0.229848003028694;
offset_cal2 = -0.22321;

%% collect data
IMU_base_start = 0.009;
IMU_leg_start = -0.807;
data = [...  // imu , calipher, encoder;.. // goals
        -0.380, 0.56400, 0.36623;... // 1.1
        -0.781, 0.59390, 0.39626;... // 1.5
        -1.192, 0.62511, 0.42745;... // 1.9
        -1.522, 0.65205, 0.45513;... // 2.3
        -1.189, 0.62519, 0.42745;... // 1.9
        -0.783, 0.59399, 0.39626;... // 1.5
        -0.382, 0.56400, 0.36623];   %% 1.1
IMU_leg_end = -0.805;
IMU_base_end = 0.006;
IMU_base = (IMU_base_start+IMU_base_end)/2;
IMU_leg = (IMU_leg_start+IMU_leg_end)/2;
IMU_th0 = imu_angle0_to_angle0(abs(IMU_leg-IMU_base));

%% calculate angle joint 0
n = length(data);                                   % number of points
% N = linspace(1,n,n);                                % measurement points
N = [1.1 1.5 1.9 2.3 1.9 1.5 1.1].';                % measurement points
th2_imu = imu_angle2_to_angle2(abs(data(:,1)-IMU_base),IMU_th0);
% th2_imu = abs(data(:,1)-IMU_ref);                  % angle by imu
th2_cal = spring2_to_angle2(data(:,2)+offset_cal2); % angle by caliphers
th2_enc = spindle2_to_angle2(data(:,3));            % angle by encoders

% plot difference
figure; 
plot(N,th2_imu,N,th2_cal,'--',N,th2_enc,'--'); grid on; xlabel('Measurement N');
ylabel('angle 0 [rad]'); legend('imu','calipher','encoder');

% plot difference
diff_enc = th2_imu-th2_enc;
diff_cal = th2_imu-th2_cal;
figure; 
plot(N,diff_enc,N,diff_cal,'--'); grid on; xlabel('Measurement N'); 
ylabel('difference in angle 0 [rad]'); legend('enc','cal');

figure; 
plot(N,diff_enc./th2_imu,N,diff_cal./th2_imu,'--'); grid on; xlabel('Measurement N'); 
ylabel('rel. difference in angle 0 [rad]'); legend('enc','cal');

%% calculate offset change
cal = data(:,2)+offset_cal2;
cal_imu = angle2_to_spring2(th2_imu);
diff_cal_imu = cal_imu-cal;
figure; plot(N,diff_cal_imu); grid on; xlabel('measurement position');
ylabel('calipher length difference [m]');

avg_diff = mean(diff_cal_imu);

offset_cal2_new = offset_cal2+avg_diff;

s = sprintf(['\n The new offset of calipher 2 is:\n',...
    '\t %7.5f,\n'],offset_cal2_new);
disp(s)



