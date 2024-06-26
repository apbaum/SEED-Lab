%% RunControllerTesting.m
% Mini Project
%
% Modified by: Madeleine Houghton
%
% Date: 2/15/2024
%
% This script runs a simulation of a motor controller and plots the
% results. The results plot experimental and simulated voltage vs. time 
% and position vs. time data.
%
% Required file: ControllerDiagram.slx
%
% Loads in the Arduino step data for the motor controller program.
load('stepData.mat');
Kp = 2.5; % Gain for motor controller
Kp_pos = 18.7;
Ki = 4.5;
K=1.35; % DC gain [rad/Vs]
sigma=9.7; % Time constant reciprocal [1/s]
%% Run a Simulation
open_system('ControllerDiagram');

% Runs the motor controller simulation.
out=sim('ControllerDiagram');
%% Plot the results from simulation and experimentation data

% Plots voltages.
figure
subplot(2,1,1)
plot(out.Voltage,'--','linewidth',2)
hold on
plot(data(:,1),data(:,2),'linewidth',2)
legend('Simulated','Experimental','location','southeast')
hold off
xlabel('Time (s)')
ylabel('Voltage (V)')

% Plots positions.
subplot(2,1,2)
plot(out.Position,'--','linewidth',2)
hold on
plot(data(:,1),data(:,3),'linewidth',2)
hold off
legend('Simulated','Experimental','location','southeast')
xlabel('Time (s)')
ylabel('Position (rad)')
