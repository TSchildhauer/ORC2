% #########################################################################
% TUHH :: Institute for Control Systems :: Control Lab
% #########################################################################
% Experiment ORC2: Robust Control of a Gyroscope
%
% Copyright Herbert Werner and Hamburg University of Technology, 2014
% #########################################################################
% This File is used to run the nonlinear simulation and plot the results
% This version is to be published using 
%   publish('orc2_simulation.m','pdf') 
% and submitted as a pdf-file at least one week prior to the scheduled date
% for the experiment to prove that your design works

%----------------------------
% v.0.9 - 26-03-2015
% by Julian Theis 
%----------------------------
% Last modified on 26-03-2015
% by Julian Theis
% ---------------------------
%%
% set operating point
q1dot_0 = 45;
q2_0 = 0;
q3_0 = 0;

% select which controller is used 
% (1 for single-degree-of-freedom)
% (2 for two-degrees-of-freedom)
% (3 for two-degrees-of-freedom with all measurements)
select_controller = 3;

%% run simulation
addpath('./simulink')
open('orc2_GyroSimulation2')
sim('orc2_GyroSimulation2')


%% Plots Simulation Results

% downsampling
ds = 50;

% q3
figure(101);clf
subplot(211)
plot(downsample(simdata.time,ds),downsample(simdata.signals(1).values(:,2),ds),'k:')
hold on;
plot(downsample(simdata.time,ds),downsample(simdata.signals(1).values(:,1),ds),'b-')
xlim([20 70])
ylim([-10 10])
ylabel('q3')
title('Tracking')
% q4
subplot(212)
plot(downsample(simdata.time,ds),downsample(simdata.signals(2).values(:,2),ds),'k:')
hold on;
plot(downsample(simdata.time,ds),downsample(simdata.signals(2).values(:,1),ds),'b-')
xlim([20 70])
ylim([-60 60])
xlabel('time')
ylabel('q4')


% controls
figure(102)
subplot(211)
plot(downsample(simdata.time,ds),downsample(simdata.signals(5).values(:,1),ds),'b-')
xlim([20 70])
ylim([-0.666 0.666])
ylabel('T1 (Nm)')
title('Control Inputs')
subplot(212)
plot(downsample(simdata.time,ds),downsample(simdata.signals(5).values(:,2),ds),'b-')
xlim([20 70])
ylim([-2.44 2.44])
xlabel('time')
ylabel('T2 (Nm)')


% angular velocities
figure(103);clf
subplot(311)
plot(downsample(simdata.time,ds),downsample(simdata.signals(3).values(:,1),ds),'b-')
xlim([20 70])
ylim([-2 2])
ylabel('q2dot')
title('Angular Velocities')
subplot(312)
plot(downsample(simdata.time,ds),downsample(simdata.signals(3).values(:,2),ds),'b-')
xlim([20 70])
ylim([-2 2])
ylabel('q3dot')
subplot(313)
plot(downsample(simdata.time,ds),downsample(simdata.signals(3).values(:,3),ds),'b-')
xlim([20 70])
ylim([-2 2])
xlabel('time')
ylabel('q4dot')

% operating point
figure(104);clf
subplot(311)
plot(downsample(simdata.time,ds),repmat(q1dot_0,1,numel(downsample(simdata.time,ds))),'k:')
hold on;
plot(downsample(simdata.time,ds),downsample(simdata.signals(4).values(:,1),ds),'b-')
xlim([20 70])
ylim([q1dot_0-15 q1dot_0+15])
ylabel('q1dot')
title('Operating Point')
subplot(312)
plot(downsample(simdata.time,ds),repmat(q2_0,1,numel(downsample(simdata.time,ds))),'k:')
hold on;
plot(downsample(simdata.time,ds),downsample(simdata.signals(4).values(:,2),ds),'b-')
xlim([20 70])
ylim([q2_0-20 q2_0+20])
ylabel('q2')
subplot(313)
plot(downsample(simdata.time,ds),repmat(q3_0,1,numel(downsample(simdata.time,ds))),'k:')
hold on;
plot(downsample(simdata.time,ds),downsample(simdata.signals(1).values(:,1),ds),'b-')
xlabel('time')
ylabel('q3')
xlim([20 70])
ylim([q3_0-20 q3_0+20])

ShowAllFig(101:104)