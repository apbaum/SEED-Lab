% run_steeringsimulation
% Adapted by: Madeleine Houghton
%
% Runs a simulation of a two wheeled robot with a forward motion and anglual
% motion controller. This script sets up
% the parameters of the robot, and the input voltages as timeseries
% The simulink model 'steeringsimulation' is run, and the results plotted.


% Conversions from angle to counts and back
rad_to_counts = 3200/(2*pi);
counts_to_rad = 1/rad_to_counts;
r_measured_ft = 0.26365; % measured wheel radius;
b_measured_ft = 0.9479; % measured robot width;
r_actual_ft = .25; % actual wheel radius
b_actual_ft = 1; % actual robot width
Ts=.01; % sample time in seconds
%
% right wheel parameters
%
K_r=1.2;
sigma_r=10.7;

%
% left wheel parameters
%
K_l=1.4;
sigma_l=11.7;

% Sets inputs to the controller.
phi_d=timeseries([0 5*pi/2 5*pi/2 5*pi/2],[0 2.5 9 30]);
rho_d=timeseries([0 0 3 3],[0 6 15 30]);

% Plots the results
out=sim('steeringsimulation.slx')
figure(1)
clf
plot(phi_d)
hold on
plot(rho_d)
plot(out.phi)
plot(out.rho)
set(gca,'fontsize',14)
legend('phi setpoint','rho setpoint','phi','rho','location','northwest')
xlabel('Time (s)')
ylabel('magnitude')
title('Wheel position')
figure(2)
clf
plot(out.Pos)
set(gca,'fontsize',14)
xlabel('Time (s)')
ylabel('Position (ft)')
legend('X','Y','Phi')
title('Robot Position')
figure(3)
animate
