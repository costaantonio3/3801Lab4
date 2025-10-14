clear;
clc;
close all;

%% ========================================================================
%% PLOTTING FUNCTION
%% ========================================================================
function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
% PlotAircraftSim: Plots the results of a full simulation
% Inputs:
%   time - n x 1 vector of time values
%   aircraft_state_array - 12 x n array of aircraft states
%   control_input_array - 4 x n array of control inputs [Zc; Lc; Mc; Nc]
%   fig - 6 x 1 vector of figure numbers to plot
%   col - plotting color/style (e.g., 'b-', 'r--')

% Figure 1: Inertial Position (x, y, z)
figure(fig(1));
subplot(3,1,1);
plot(time, aircraft_state_array(1,:), col); hold on;
xlabel('Time (s)'); ylabel('x (m)'); title('X Position'); grid on;
subplot(3,1,2);
plot(time, aircraft_state_array(2,:), col); hold on;
xlabel('Time (s)'); ylabel('y (m)'); title('Y Position'); grid on;
subplot(3,1,3);
plot(time, aircraft_state_array(3,:), col); hold on;
xlabel('Time (s)'); ylabel('z (m)'); title('Z Position'); grid on;

% Figure 2: Euler Angles (phi, theta, psi)
figure(fig(2));
subplot(3,1,1);
plot(time, rad2deg(aircraft_state_array(7,:)), col); hold on;
xlabel('Time (s)'); ylabel('\phi (deg)'); title('Roll Angle'); grid on;
subplot(3,1,2);
plot(time, rad2deg(aircraft_state_array(8,:)), col); hold on;
xlabel('Time (s)'); ylabel('\theta (deg)'); title('Pitch Angle'); grid on;
subplot(3,1,3);
plot(time, rad2deg(aircraft_state_array(9,:)), col); hold on;
xlabel('Time (s)'); ylabel('\psi (deg)'); title('Yaw Angle'); grid on;

% Figure 3: Inertial Velocity in Body Frame (u, v, w)
figure(fig(3));
subplot(3,1,1);
plot(time, aircraft_state_array(4,:), col); hold on;
xlabel('Time (s)'); ylabel('u (m/s)'); title('Body X Velocity'); grid on;
subplot(3,1,2);
plot(time, aircraft_state_array(5,:), col); hold on;
xlabel('Time (s)'); ylabel('v (m/s)'); title('Body Y Velocity'); grid on;
subplot(3,1,3);
plot(time, aircraft_state_array(6,:), col); hold on;
xlabel('Time (s)'); ylabel('w (m/s)'); title('Body Z Velocity'); grid on;

% Figure 4: Angular Velocity (p, q, r)
figure(fig(4));
subplot(3,1,1);
plot(time, rad2deg(aircraft_state_array(10,:)), col); hold on;
xlabel('Time (s)'); ylabel('p (deg/s)'); title('Roll Rate'); grid on;
subplot(3,1,2);
plot(time, rad2deg(aircraft_state_array(11,:)), col); hold on;
xlabel('Time (s)'); ylabel('q (deg/s)'); title('Pitch Rate'); grid on;
subplot(3,1,3);
plot(time, rad2deg(aircraft_state_array(12,:)), col); hold on;
xlabel('Time (s)'); ylabel('r (deg/s)'); title('Yaw Rate'); grid on;

% Figure 5: Control Inputs (Zc, Lc, Mc, Nc)
figure(fig(5));
subplot(4,1,1);
plot(time, control_input_array(1,:), col); hold on;
xlabel('Time (s)'); ylabel('Z_c (N)'); title('Control Force Z'); grid on;
subplot(4,1,2);
plot(time, control_input_array(2,:), col); hold on;
xlabel('Time (s)'); ylabel('L_c (N*m)'); title('Control Moment L (Roll)'); grid on;
subplot(4,1,3);
plot(time, control_input_array(3,:), col); hold on;
xlabel('Time (s)'); ylabel('M_c (N*m)'); title('Control Moment M (Pitch)'); grid on;
subplot(4,1,4);
plot(time, control_input_array(4,:), col); hold on;
xlabel('Time (s)'); ylabel('N_c (N*m)'); title('Control Moment N (Yaw)'); grid on;

% Figure 6: 3D Path
figure(fig(6));
plot3(aircraft_state_array(1,:), aircraft_state_array(2,:), -aircraft_state_array(3,:), col); hold on;
% Mark start (green) and finish (red)
plot3(aircraft_state_array(1,1), aircraft_state_array(2,1), -aircraft_state_array(3,1), ...
      'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(aircraft_state_array(1,end), aircraft_state_array(2,end), -aircraft_state_array(3,end), ...
      'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Height (m)');
title('3D Aircraft Path'); grid on; axis equal;
view(3);
end

%% ========================================================================
%% DATA IMPORT AND PROCESSING
%% ========================================================================

% Load the data file
load('RSdata_nocontrol.mat');

% Extract time vector from estimated states
time = rt_estim.time(:);

% Extract state estimates
% Columns: X_E, Y_E, Z_E, ψ, θ, φ, u_E, v_E, w_E, p, q, r
state_values = rt_estim.signals.values;

% Build aircraft_state_array (12 x n) to match plotting function format
% Expected order: [x; y; z; u; v; w; phi; theta; psi; p; q; r]
aircraft_state_array = zeros(12, length(time));

aircraft_state_array(1,:) = state_values(:,1)';  % X_E
aircraft_state_array(2,:) = state_values(:,2)';  % Y_E
aircraft_state_array(3,:) = state_values(:,3)';  % Z_E
aircraft_state_array(4,:) = state_values(:,7)';  % u_E
aircraft_state_array(5,:) = state_values(:,8)';  % v_E
aircraft_state_array(6,:) = state_values(:,9)';  % w_E
aircraft_state_array(7,:) = state_values(:,6)';  % phi (φ)
aircraft_state_array(8,:) = state_values(:,5)';  % theta (θ)
aircraft_state_array(9,:) = state_values(:,4)';  % psi (ψ)
aircraft_state_array(10,:) = state_values(:,10)'; % p
aircraft_state_array(11,:) = state_values(:,11)'; % q
aircraft_state_array(12,:) = state_values(:,12)'; % r

% Extract motor commands and convert to control inputs (optional)
% Columns: Motor 1, Motor 2, Motor 3, Motor 4
motor_values = rt_motor.signals.values;
motor_time = rt_motor.time(:);

% Convert motor commands to rotation rates (rad/s)
omega_motors = sqrt(motor_values) * 13840.4;

% Create a placeholder control_input_array (4 x n)
% Note: Actual control inputs [Zc; Lc; Mc; Nc] would require
% drone parameters and motor-to-control mapping
control_input_array = zeros(4, length(time));

% If you have the control command data, you can use rt_cmd
% cmd_values = rt_cmd.signals.values;
% Process based on your specific control law

%% ========================================================================
%% CALL PLOTTING FUNCTION
%% ========================================================================

% Define figure numbers
fig = [1, 2, 3, 4, 5, 6];

% Define plot style
col = 'b-';

% Call the plotting function
PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col);
