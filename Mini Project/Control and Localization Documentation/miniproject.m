%% RunMiniProjectSim.m
% Mini Project
%
% Modified by: Madeleine Houghton
%
% Date: 2/23/2024
%
% This script runs a simulation of a motor controller and plots the
% results. The results plot experimental and simulated voltage vs. time 
% and velocity vs. time data.
%
% Required file: miniproject_controller.slx
%
% Loads in the Arduino step data for the motor controller program.
%% Define motor parameters
Kp = 2.5; % Gain for motor controller
Kp_pos = 18.7;
Ki = 4.5;
K=1.35; % DC gain [rad/Vs]
sigma=9.7; % Time constant reciprocal [1/s]
%% Run a Simulation
% Opens the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function.
% In the diagram, the step function is set with a time step of 1 and final
% value of 1 (for 1 rad/sec at 1 second). The saturation interval is 
% between -7.5 V and 7.5 V and the band-limited white noise has a noise 
% power of 0.0001 and sample time 0.01. The block diagram runs for 3 seconds.
open_system('miniproject_controller');

% Runs the motor controller simulation.
out=sim('miniproject_controller');

% Plots simulation results.
% Plots the voltage output.
figure
subplot(3,1,1);
plot(out.Voltage,'linewidth',2)
xlabel('Time (s)')
ylabel('Voltage (V)')

% Plots the desired velocity.
subplot(3,1,2);
plot(out.DesiredVelocity, 'linewidth',2)
xlabel('Time (s)')
ylabel('Desired Velocity (rad/sec)')

% Plots the actual position versus the desired position.
subplot(3,1,3);
plot(out.Position,'linewidth',2)
hold on
plot(out.DesiredPosition,'linewidth',2)
xlabel('Time (s)')
ylabel('Position (rad)')
hold off
