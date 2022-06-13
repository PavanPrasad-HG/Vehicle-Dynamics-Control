% B.Shyrokau
% Template for homework assignment #2
% RO47017 Vehicle Dynamics & Control, 2022
% Use and distribution of this material outside the RO47017 course 
% only with the permission of the course coordinator
clc; clear; clear mex;
set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');
%% Non-tunable parameters
par.g = 9.81;
par.Vinit   = 50 /3.6;              % initialization velocity, Don't TUNE
% Vehicle/Body (Camry)
par.mass     = 1380;                % vehicle mass, kg      
par.Izz      = 2634.5;              % body inertia around z-axis, kgm^2
par.L        = 2.79;                % wheelbase, m
par.l_f      = 1.384;               % distance from front axle to CoG, m
par.l_r      = par.L - par.l_f;     % distance from rear axle to CoG, m
% Steering
par.i_steer  = 15.4;                % steering ratio
% Additional
par.m_f      = par.mass * par.l_r / par.L;      % front sprung mass, kg
par.m_r      = par.mass * par.l_f / par.L;      % rear sprung mass, kg
par.mu       = 0.3;                   % friction coefficient
%% Tunable parameters
% Reference Generator
par.Calpha_front = 120000;          % front axle cornering stiffness
par.Calpha_rear  = 190000;          % rear axle cornering stiffness
par.Kus = par.m_f/par.Calpha_front - par.m_r/par.Calpha_rear; % understeer gradient
% second order TF identified from Sine Swept Test
par.wn      = 11;                   % yaw rate frequency
par.kseta   = 0.7;                  % yaw rate damping
par.tau     = 0.09;                 % yaw rate time constant
%% Change V_kmph as you prefer

% Maneuver settings

V_kmph = 100; % pre-maneuver speed, km/h

V_ref = V_kmph /3.6;                % pre-maneuver speed, m/s

% Reference Generator TF coefficients

par.w0 = par.wn*sqrt(1-par.kseta^2); %
coeff_num_s = par.mass*par.l_f*V_ref/(par.L*par.Calpha_rear) ; %coefficient of s in numerator
coeff_den_s = 2*par.kseta/par.w0; %coefficient of s in denominator
coeff_den_s2 = 1/par.w0^2; %coefficient of s^2 in denominator
%% AY Reference Generator TF coefficients

par.w0 = par.wn*sqrt(1-par.kseta^2); %
acoeff_num_s2 = par.Izz/(par.L*par.Calpha_rear) ; %coefficient of s^2 in numerator
acoeff_num_s = par.l_r/V_ref ; %coefficient of s in numerator
acoeff_den_s = 2*par.kseta; %coefficient of s in denominator
acoeff_den_s2 = 1/par.w0^2; %coefficient of s^2 in denominator
%% LQR Control Design - Using Yaw Rate

A11 = (par.Calpha_front + par.Calpha_rear);
A21 = -par.l_f*par.Calpha_front + par.l_r*par.Calpha_rear; 
A22 = par.l_f^2 * par.Calpha_front + par.l_r^2 * par.Calpha_rear;

n_points = 60; % number of data points

LQR_Vx_ref = linspace(10,V_kmph,n_points)/3.6; % longitudinal speed, km/h

% zero initialization
LQR_K = zeros(n_points,2);

% LQR gain calculation
for i = 1:length(LQR_Vx_ref)
Vx = LQR_Vx_ref(i);

%B_lqr = [par.Calpha_front/par.mass 0; par.l_f*par.Calpha_front/par.Izz 1/par.Izz];
B_lqr = [0;1/par.Izz];
A_lqr = [-A11/(par.mass * Vx), (A21/(par.mass * Vx))-Vx; (A21/(par.Izz* Vx)), -A22/(par.Izz* Vx)];

R = diag([Vx]);
Q = diag([10^0,5*10^13]);
%Q = diag([10^4,10^2]);
LQR_K(i,:) = lqr(A_lqr,B_lqr,Q,R); % LQR gain
end

LQR_K = LQR_K';

ay_max = par.mu*par.g; %limit on lateral acceleration to saturate beta
%% PD Control
% Running LQR
sim('GP_PD.slx')

%  Plots - Beta Sat

figure(1)
plot(ref_yaw,'b--')
hold on
plot(yaw_lqr,'r')
title(sprintf('Yaw Rate vs Reference - PD Control of Sideslip Angle - %0.0f km/h',V_kmph),'Interpreter','latex')
xlabel('Time(s)'), ylabel('Yaw Rate(rad/s)')
legend('Reference Yaw Rate','Actual Yaw Rate')

max(ref_yaw.Data)
min(ref_yaw.Data)

max(yaw_lqr.Data)
min(yaw_lqr.Data)

figure(2)
plot(long_vel)
title(sprintf('Longitudinal Velocity - PD Control of Sideslip Angle - Reference velocity %0.0f km/h',V_kmph),'Interpreter','latex')
xlabel('Time(s)'), ylabel('Longitudinal Velocity (km/h)')

figure(3)
plot(lat_acc_lqr)
title(sprintf('Lateral Acceleration - PD Control of Sideslip Angle - Reference velocity %0.0f km/h',V_kmph),'Interpreter','latex')
xlabel('Time(s)'), ylabel('Lateral Acceleration (m/s^2)')

figure(4)
plot(beta_lqr)
title(sprintf('Body Sideslip Angle - PD Control of Sideslip Angle - Reference velocity %0.0f km/h',V_kmph),'Interpreter','latex')
xlabel('Time(s)'), ylabel('Body Sideslip Angle (deg)')

max(abs(beta_lqr.Data))
%% Ref Correction
% Running LQR

sim('GP_ref_correction.slx')

% Plots - Ref Correction

figure(5)
plot(ref_yaw,'b--')
hold on
plot(yaw_lqr,'r')
title(sprintf('Yaw Rate vs Reference - Reference Correction - %0.0f km/h',V_kmph))
xlabel('Time(s)'), ylabel('Yaw Rate(rad/s)')
legend('Reference Yaw Rate','Actual Yaw Rate')

max(ref_yaw.Data)
min(ref_yaw.Data)

max(yaw_lqr.Data)
min(yaw_lqr.Data)

figure(6)
plot(long_vel)
title(sprintf('Longitudinal Velocity - Reference Correction - Reference velocity %0.0f km/h',V_kmph))
xlabel('Time(s)'), ylabel('Longitudinal Velocity (km/h)')

figure(7)
plot(lat_acc_lqr)
title(sprintf('Lateral Acceleration - Reference Correction - Reference velocity %0.0f km/h',V_kmph))
xlabel('Time(s)'), ylabel('Lateral Acceleration (m/s^2)')

figure(8)
plot(beta_lqr)
title(sprintf('Body Sideslip Angle - Reference Correction - Reference velocity %0.0f km/h',V_kmph))
xlabel('Time(s)'), ylabel('Body Sideslip Angle (deg)')

max(abs(beta_lqr.Data))

%% Beta Sat
% Running LQR
sim('GP_beta_saturation.slx')

%  Plots - Beta Sat

figure(9)
plot(ref_yaw,'b--')
hold on
plot(yaw_lqr,'r')
title(sprintf('Yaw Rate vs Reference - Saturation of Sideslip Angle - %0.0f km/h',V_kmph),'Interpreter','latex')
xlabel('Time(s)'), ylabel('Yaw Rate(rad/s)')
legend('Reference Yaw Rate','Actual Yaw Rate')

max(ref_yaw.Data)
min(ref_yaw.Data)

max(yaw_lqr.Data)
min(yaw_lqr.Data)

figure(10)
plot(long_vel)
title(sprintf('Longitudinal Velocity - Saturation of Sideslip Angle - Reference velocity %0.0f km/h',V_kmph),'Interpreter','latex')
xlabel('Time(s)'), ylabel('Longitudinal Velocity (km/h)')

figure(11)
plot(lat_acc_lqr)
title(sprintf('Lateral Acceleration - Saturation of Sideslip Angle - Reference velocity %0.0f km/h',V_kmph),'Interpreter','latex')
xlabel('Time(s)'), ylabel('Lateral Acceleration (m/s^2)')

figure(12)
plot(beta_lqr)
title(sprintf('Body Sideslip Angle - Saturation of Sideslip Angle - Reference velocity %0.0f km/h',V_kmph),'Interpreter','latex')
xlabel('Time(s)'), ylabel('Body Sideslip Angle (deg)')

max(abs(beta_sat.Data))

%%

[I1,J1]=find(ydist_pd.Time==11.07);
[I2,J2]=find(ydist_ref.Time==11.07);
[I3,J3]=find(ydist_sat.Time==11.07);

disp("[PD] Y distance after 1.07 sec is " + ydist_pd.Data(I1));
disp("[PD] Yaw Velocity Metric is " + yaw_velocity_metric_pd.Data(end));
disp("[PD] Max.Sideslip Angle is " + max(abs(beta_pd.Data)));

disp("[Reference Correction]] Y distance after 1.07 sec is " + ydist_ref.Data(I2));
disp("[Reference Correction]] Yaw Velocity Metric is " + yaw_velocity_metric_ref.Data(end));
disp("[Reference Correction]] Max.Sideslip Angle is " + max(abs(beta_ref.Data)));

disp("[Beta Saturation] Y distance after 1.07 sec is " + ydist_sat.Data(I3));
disp("[Beta Saturation] Yaw Velocity Metric is " + yaw_velocity_metric_sat.Data(end));
disp("[Beta Saturation] Max.Sideslip Angle is "+ max(abs(beta_sat.Data)));
%% Plots

% PD Control
figure(3)
plot(ref_yaw_pd,'b--')
hold on
plot(yaw_pd,'r')
title(sprintf('Yaw Rate vs Reference - PD Control - %0.0f km/h',V_kmph),'Interpreter','latex')
xlabel('Time(s)'), ylabel('Yaw Rate(rad/s)')
legend('Reference Yaw Rate','Actual Yaw Rate')

figure(4)
plot(beta_pd)
title(sprintf('Body Sideslip Angle - PD Control - Reference velocity %0.0f km/h',V_kmph))
xlabel('Time(s)'), ylabel('Body Sideslip Angle (deg)')

% Ref Corr
figure(3)
plot(ref_yaw_pd,'g--')
hold on
plot(ref_yaw_ref,'b--')
hold on
plot(yaw_ref,'r')
title(sprintf('Yaw Rate vs Reference - Reference Correction - %0.0f km/h',V_kmph),'Interpreter','latex')
xlabel('Time(s)'), ylabel('Yaw Rate(rad/s)')
legend('Reference Yaw Rate without Correction','Reference Yaw Rate','Actual Yaw Rate')

figure(4)
plot(beta_ref)
title(sprintf('Body Sideslip Angle - Reference Correction - Reference velocity %0.0f km/h',V_kmph))
xlabel('Time(s)'), ylabel('Body Sideslip Angle (deg)')

% Saturation
figure(5)
plot(ref_yaw_pd,'g--')
hold on
plot(ref_sat,'b--')
hold on
plot(yaw_sat,'r')
title(sprintf('Yaw Rate vs Reference - Saturation of Sideslip Angle - %0.0f km/h',V_kmph),'Interpreter','latex')
xlabel('Time(s)'), ylabel('Yaw Rate(rad/s)')
legend('Reference Yaw Rate without Correction','Reference Yaw Rate','Actual Yaw Rate')

figure(6)
plot(beta_sat)
title(sprintf('Body Sideslip Angle - Saturation of Sideslip Angle - Reference velocity %0.0f km/h',V_kmph),'Interpreter','latex')
xlabel('Time(s)'), ylabel('Body Sideslip Angle (deg)')