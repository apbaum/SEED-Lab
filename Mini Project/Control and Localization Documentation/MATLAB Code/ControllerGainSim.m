%% RunControllerGainSim.m
% Mini Project
%
% Modified by: Madeleine Houghton
%
% Date: 2/14/2024
%
% This script runs a simulation of a motor controller and plots the
% results. The results show the outputs of the Simulink controller.
%
% Required file: ControllerDiagram.slx
%
%% Define motor parameters
Kp = 2.5; % Gain for motor controller
Kp_pos = 18.7;
Ki = 4.5;
K=1.35; % DC gain [rad/Vs]
sigma=9.7; % Time constant reciprocal [1/s]
%% Run a Simulation
% Opens the block diagram.
open_system('ControllerDiagram');

% Runs the motor controller simulation.
out=sim('ControllerDiagram');

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
