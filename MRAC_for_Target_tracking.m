
%% main.m - Physics-Based 2-DOF Gimbal Simulation with MRAC (Final Version)
% Author: Iram Zubi
% Description: Realistic gimbal simulation with separate MRAC controllers for pitch and yaw.
%              Includes actuator limits, sensor noise, and well-organized plots.

clc; clear; close all;

%% Simulation Parameters
T = 30; dt = 0.01; time = 0:dt:T;
N = length(time);

%% Gimbal Physical Parameters
J_pitch = 0.02; J_yaw = 0.025;
b_pitch = 0.01; b_yaw = 0.015;

tau_max = 1;  % actuator torque limit
sensor_noise_std = deg2rad(0.1);  % simulated IMU noise

%% MRAC Parameters
gamma = 30;
theta_pitch = zeros(2,N); theta_yaw = zeros(2,N);

%% State Initialization
x_pitch = zeros(2,N); x_yaw = zeros(2,N);
x_ref_pitch = zeros(2,N); x_ref_yaw = zeros(2,N);
e_pitch = zeros(2,N); e_yaw = zeros(2,N);
u_pitch = zeros(1,N); u_yaw = zeros(1,N);

% Reference Inputs
r_pitch = deg2rad(5)*sin(0.5*time);
r_yaw = deg2rad(7)*cos(0.3*time);

% Reference Model
omega_n = 4; zeta = 0.9;
A_ref = [0 1; -omega_n^2 -2*zeta*omega_n];
B_ref = [0; omega_n^2];

%% Simulation
for k = 1:N-1
    measured_pitch = x_pitch(1,k) + sensor_noise_std*randn;
    measured_yaw = x_yaw(1,k) + sensor_noise_std*randn;

    phi_pitch = [measured_pitch; x_pitch(2,k)];
    phi_yaw = [measured_yaw; x_yaw(2,k)];

    u_pitch(k) = max(min(theta_pitch(:,k)' * phi_pitch, tau_max), -tau_max);
    u_yaw(k) = max(min(theta_yaw(:,k)' * phi_yaw, tau_max), -tau_max);

    dx_pitch = [x_pitch(2,k); (u_pitch(k) - b_pitch*x_pitch(2,k)) / J_pitch];
    x_pitch(:,k+1) = x_pitch(:,k) + dt * dx_pitch;

    dx_yaw = [x_yaw(2,k); (u_yaw(k) - b_yaw*x_yaw(2,k)) / J_yaw];
    x_yaw(:,k+1) = x_yaw(:,k) + dt * dx_yaw;

    dxr_pitch = A_ref * x_ref_pitch(:,k) + B_ref * r_pitch(k);
    x_ref_pitch(:,k+1) = x_ref_pitch(:,k) + dt * dxr_pitch;

    dxr_yaw = A_ref * x_ref_yaw(:,k) + B_ref * r_yaw(k);
    x_ref_yaw(:,k+1) = x_ref_yaw(:,k) + dt * dxr_yaw;

    e_pitch(:,k) = x_pitch(:,k) - x_ref_pitch(:,k);
    e_yaw(:,k)   = x_yaw(:,k)   - x_ref_yaw(:,k);

    theta_pitch(:,k+1) = theta_pitch(:,k) - dt * gamma * phi_pitch * e_pitch(1,k);
    theta_yaw(:,k+1)   = theta_yaw(:,k)   - dt * gamma * phi_yaw   * e_yaw(1,k);
end

e_pitch(:,N) = x_pitch(:,N) - x_ref_pitch(:,N);
e_yaw(:,N)   = x_yaw(:,N)   - x_ref_yaw(:,N);

%% Plotting (Improved)
figure('Name','Pitch Control');
subplot(3,1,1); plot(time, rad2deg(r_pitch), 'r--', time, rad2deg(x_pitch(1,:)), 'b', time, rad2deg(x_ref_pitch(1,:)), 'g-.');
ylabel('Angle [deg]'); title('Pitch Tracking'); legend('Reference','Plant','Model'); grid on;

subplot(3,1,2); plot(time, rad2deg(e_pitch(1,:)), 'k'); ylabel('Error [deg]');
title('Pitch Tracking Error'); legend('Error'); grid on;

subplot(3,1,3); plot(time, theta_pitch(1,:), 'b', time, theta_pitch(2,:), 'm');
legend('\theta_1','\theta_2'); xlabel('Time [s]'); ylabel('Gain');
title('Pitch Adaptive Gains'); grid on;

figure('Name','Yaw Control');
subplot(3,1,1); plot(time, rad2deg(r_yaw), 'r--', time, rad2deg(x_yaw(1,:)), 'b', time, rad2deg(x_ref_yaw(1,:)), 'g-.');
ylabel('Angle [deg]'); title('Yaw Tracking'); legend('Reference','Plant','Model'); grid on;

subplot(3,1,2); plot(time, rad2deg(e_yaw(1,:)), 'k'); ylabel('Error [deg]');
title('Yaw Tracking Error'); legend('Error'); grid on;

subplot(3,1,3); plot(time, theta_yaw(1,:), 'b', time, theta_yaw(2,:), 'm');
legend('\theta_1','\theta_2'); xlabel('Time [s]'); ylabel('Gain');
title('Yaw Adaptive Gains'); grid on;
