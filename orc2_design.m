% #########################################################################
% TUHH :: Institute for Control Systems :: Control Lab
% #########################################################################
% Experiment ORC2: Robust Control of a Gyroscope
%
% Copyright Herbert Werner and Hamburg University of Technology, 2014
% #########################################################################
% This file is to be completed by the student.
% The completed version is to be published using 
%   publish('orc2_design.m','pdf') 
% and submitted as a pdf-file at least one week prior to the scheduled date
% for the experiment
%
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% !!!  The gaps in the code are denoted by XXX !!!
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
%
% HINT 1:
% if you want to find out more about a certain command, just type 
% 'help command' into the matlab window
% HINT 2:
% use section evaluations (Ctrl+Enter) to run the code within a single 
% section

%----------------------------
% v.0.9 - 26-03-2015
% by Julian Theis 
%----------------------------
% Last modified on 26-03-2015
% by Julian Theis
% ---------------------------
%%
clearvars; clc; close all

XXX = 'YOU NEED TO CORRECT THIS';
%% 1.  PLANT AND SCALING
%
% we define the frequency range of interest for the design to be 0.01 rad/s
% to 1000 rad/s, mainly for convenience in plotting. Further some options
% for bodeplots are invoked.
w = {1e-2,1e3};
b = bodeoptions;
b.xlim = [1e-2 1e2];
b.ylim = [-1e2 1e2];
b.Grid = 'on';

%% 1.1 Linearized model
% We first evaluate the Jacobian of the nonlinear system at a single
% operating point, which gives an LTI state space model
% The first parameter is angular velocity of the flywheel, the second and
% third are the gimbal inclinations q2 and q3.
Gp = linearize_gyro(45,0,0);

figure(1)
bodemag(Gp,b,w)
figure(2)
sigma(Gp,w)

%% 1.2 Initial Scaling
%
% The following (very simple) scaling just normalizes
% the input and output values of the plant, such that 1 corresponds to both
% the maximum expected change in the reference signal and the maximum
% actuator capacity. 

% Define the largest allowed input signals (in Nm)
T1_max = 0.666; %[Nm] Thomas: 09.11.
T2_max = 2.44; %[Nm] Thomas: 09.11.
umax = [T1_max T2_max];
% and the largest expected change in reference (in rad)
q3_max = 10*pi./180; %[rad]
q4_max = 45*pi./180; %[rad]
rmax = [q3_max q4_max];
% Now, scale both input and output this values
Du = diag(umax);
Dr = diag(rmax);
G = Du*Gp/Dr; %Thomas: 09.11.

% G is now our synthesis model used to design a controller.
% When we apply this controller to the real plant, we need to remember to
% reverse the scalings, ie. 
% u_real = Du*u and y = Dr^-1*y_real and r = Dr^-1*r_real

%% 2.1 S/KS closed-loop shaping filter design
%
% We are now ready to start with the actual controller design.
% S defines our nominal performance while KS provides some basic
% robustness and of course limits our control effort.
% As you hopefully remember, S needs to be weighted with a lowpass while KS
% needs to be weighted with a high-pass in order to achieve the desired
% behavior. 

% While we could use the parameterization from the ORC lecture 
% notes and build the filters manually as tf objects, a build-in command
% that we can use is makeweight(dcgain, bandwidth, feedthroughgain).
% For W_S, the dcgain determines our inverse error constant, the bandwidth the speed
% of response and the feedthroughgain limits the peak of S
dcgain1 = 1000;
bandwidth1 = 1;
feedthroughgain1 = 0.5;
W_S1 = makeweight(dcgain1, bandwidth1, feedthroughgain1); % applied to v1
dcgain2 = 1000;
bandwidth2 = 1;
feedthroughgain2 = 0.5;
W_S2 = makeweight(dcgain2, bandwidth2, feedthroughgain2); % applied to v2
W_S  = mdiag(W_S1, W_S2);

% For W_K, the dcgain determines available control effort, the bandwidth 
% corresponds to the available actuator rates and the feedthroughgain limits 
% authority at high frequencies
dcgain_k1 = 0.9;
bandwidth_k1 = 2;
feedthroughgain_k1 = 100;
W_K1 = makeweight(dcgain_k1, bandwidth_k1, feedthroughgain_k1); % applied to u1
dcgain_k2 = 0.7;
bandwidth_k2 = 2;
feedthroughgain_k2 = 100;
W_K2 = makeweight(dcgain_k2, bandwidth_k2, feedthroughgain_k2); % applied to u2
W_K  = mdiag(W_K1, W_K2);

figure(3)
sigma(W_S^-1,W_K^-1,w)
% Note that all these values despite having "clear interpretation" are
% tuning values, and that's how the numbers in this example came up.

%% 2.2 Generalized Plant formulation
%
% For our synthesis tools to work, we need to first assemble a generalized 
% plant that includes performance inputs/outputs that represent S and KS.
systemnames = 'XXX';
inputvar = [XXX];
input_to_G = '[XXX]';
input_to_W_S = '[XXX]';
input_to_W_K = '[XXX]';
outputvar = [XXX];
cleanupsysic = 'yes'; %this just deletes the temporary variables afterwards
P = sysic

% We can also use the command P = AUGW(G,W_S,W_K) [augment with weights] 
% to get our generalized plant. 
P_verify = augw(G,W_S,W_K);
figure(200)
sigma(P,P_verify,w)   
% Make sure that your P and the P_verify look the same in the sigma plot
% shown in Figure 200

%% 2.3 Synthesis

% We first get the number of available measurements and controls 
[nmeas,ncont] = size(G); 

% We can now run the synthesis code, which takes our generalized plant, the
% number of measured / control signals and returns a
% dynamic controller K as well as the performance index gamma.
% We use the 'lmi' formulation here, which is what is taught in class.

% Further we apply a balancing transformation to the generalized plant to
% help preventing numerical issues and further perform a suboptimal
% synthesis. We can get a suboptimal controller with a specified 
% loss-of-performance  calling the hinfsyn command twice, where the second 
% time we can just ask for a controller  that achieves 1.1*gam, i.e.
% that is at most 10% worse in the H-Infinity sense.

% optimal synthesis to obtain the best possible gam
[~,~,gam,~] = hinfsyn(XXX);
% suboptimal synthesis in which a controller with 1.1*gam is sufficient
[K,CL,gam] = hinfsyn(XXX);
gam

% The first thing that we should always check is whether everything worked
% fine. Even if we do not get an error message, there could be mistakes
% made by us setting up the problem. So let's verify that the controller
% actually achieves stability and performance as it should.

% Let's now verify that the closed loop is stable
stab = isstable(CL)
% and that the Hinfnorm is less than gam
normOK = norm(CL,'inf')<gam

% What might also be of interest for implementation is the fastest 
% controller pole
fastestpole = max(abs(eig(K.a)))
% This is relatively fast, but since we have ~1100Hz available it should
% cause no serious problems


%% 3.1 Frequency Response Analysis
%
% A first step in evaluating our controller can be a frequency response
% analysis. We can calculate all the six different transfer functions of
% interest using the loopsens command. These are the sensitivity S / input
% sensitivity Si, complementary sensitivity T , complementary input sensitivty Ti,
% as well as the transfer functions K*S and S*G (which are for square
% plants the same as Si*K and G*Si, respectively)

loops = loopsens(XXX);
figure(4);
subplot(2,2,1)
sigma(loops.So,W_S^-1,w)
title('S')
subplot(2,2,2)
sigma(loops.PSi,w)
title('SG')
subplot(2,2,3)
sigma(loops.CSo,(W_K)^-1,w)
title('KS')
subplot(2,2,4)
sigma(loops.Ti,w)
title('Ti')

% We can identify quite a few relevant properties of our design by looking 
% at the frequency response of what is sometimes called "the gang of four".
% For example, we can see that we will achieve tracking of step responses 
% when the sensitivity function starts with a slope of +20db. Further, KS
% tells us, if the controller rolls off at high frequencies. 
%
% SG on the other hand shows the same peak as our open loop plant, which
% indicates poor damping an further the low frequency gain is large, which
% means that input disturbances are not supressed but even amplified! This
% is a consequence of the fact that the controller does not include 
% integral action but just makes use of the integral behavior of the plant.
% to satisfy the demand on the sensitivity. We can verify this by taking a 
% look at the frequency response of the controller.
figure(5); 
 bodemag(K,'b',b,w)
 title('Controller with S/KS mixed sensitivity')
%% 3.2 Time Domain Analysis 
% A way of visualizing the input disturbance problem in S/KS
% problems is to take a look at the pole zero map of the open-loop plant 
% and the controller.
figure(6); 
pzplot(G,K,'r')
xlim([-1.5, 1])
title('PZmap with S/KS mixed sensitivity')
%The pole zero map of the controller and the plant shows us, that the
%controller simply cancels the lightly damped poles of the plant. This is a
%common problem with the S/KS design, since it means that input
%disturbances are not adequately damped.
 
 
% Since what we are really interestet in is tracking of a signal in the 
% time domain, let's take a look at the step responses of T. 
% It shows us how the plant reacts to an output disturbance or a change
% in the reference signal.
figure(7); 
step(loops.To,'b',loops.CSo,'g',10)
legend('y','u')
title('Step response S/KS mixed sensitivity')
% Even though there is some oscillations visible in the second response,
% the overall tracking can be seen to be sufficient.

% If, on the other hand, we consider input disturbances, the deficits of
% the present design that we expected from the frequency response analysis 
% become very apparent:
figure(8); 
step(loops.PSi,'b',10)
title('Disturbance response S/KS mixed sensitivity')


%% 3.3 Robustness
% There is a variety of linear robustness margins. For simplicity we only
% consider the (rather conservative) multiloop disk margins. These are
% computed by using what is called structured singular values and which are
% far beyond the scope of what we are looking at. The important thing however
% is that they correspond to simultaneous gain and phase variations at ALL
% plant inputs and outputs and are therefore a relatively safe measure for
% robustness of MIMO systems
mm = loopmargin(XXX,'m');
minGM = db(mm.GainMargin(2))
minPM = min(mm.PhaseMargin(2))
% For the example, we get something like 9db and 50° Phase, which is
% an excellent result.


%% HOW TO PROCEED

% Repeat the design a few times while adjusting the weights
% and see what happens.  

% To give you a little orientation, try to come up with a design that 
% 1) achieves a settling time of less than 2 seconds on both channels
% 2) a steady state error of less than 1% on both channels
% 3) the control input magnitude should not exceed 1 at any time 
% 4) a controller with a fastest pole < 2000 rad/s 
% 5) 3db and 30° Phase Multiloop Disk Margin

% Once you're design is satisfactory, take a look at the following 
% Section 4 which discusses a more advanced design approach.

%% 4.  ALTERNATE DESIGN CONCEPTS

% The following section introduces extensions to S/KS mixed
% sensitivity control in order to circumvent its limitations. 
% The design cylce is exactly the same as before and 
% Comments are only given for the parts not already discussed above.

% HINT3: You can use the provided ShowAllFig command to easily compare
% plots of different designs by invoking the number of the plot
% e.g. ShowAllFig([1,2,3]) shows the Figures 1, 2, 3 next to each other.
%% 4.1 Four block design

% In what is called a four block design, we explicitely consider input 
% disturbances and therefore avoid the cancellation problems.
% The only difference in the setup is an additional input disturbance
% weight Wd, which can be initally set to identity.

% disturbance weight
Wd = XXX;

% S / SG
W_S1 = makeweight(XXX); 
W_S2 = makeweight(XXX);
W_S  = mdiag(XXX);

% KS / Ti
W_K1 = makeweight(XXX);
W_K2 = makeweight(XXX);
W_K  = mdiag(W_K1,W_K2);


% The generalized plant now has an additional external input at the input
% of the plant. There are thus not two but four transfer functions involved
% in the problem.
systemnames = 'G Wd W_S W_K';
inputvar = ['[XXX]'];
input_to_G = '[u+Wd]';
input_to_Wd = '[w2]';
input_to_W_S = ['[w1-G]'];
input_to_W_K = '[u]';
outputvar = ['[XXX]'];
cleanupsysic = 'yes'; %this just deletes the temporary variables afterwards
P = sysic;

% The synthesis is exactly the same as above
[~,~,gam,~] = hinfsyn(XXX);
[K1,CL,gam,INFO] = hinfsyn(XXX);
gam
stab = isstable(CL)
normOK = norm(CL,'inf')<gam
fastestpole = max(abs(eig(K1.a)))

% And also the analysis can be carried out as before
loops = loopsens(XXX);
figure(14);
subplot(2,2,1)
sigma(loops.So,W_S^-1,w)
title('S')
subplot(2,2,2)
sigma(loops.PSi,w)
title('SG')
subplot(2,2,3)
sigma(loops.CSo,(W_K)^-1,w)
title('KS')
subplot(2,2,4)
sigma(loops.Ti,w)
title('Ti')
% Compare figure 14 to figure 4. The main difference now is, that the
% transfer functions related to the input disturbance, i.e. SG is
% drastically decreased in mangitude. In the low frequency range it tends
% to zero, thus step disturbances are completely regulated in steady state.
% To achieve this, the controller now actually includes
% integral action, i.e., it has a high gain at low frequencies. We can
% verify this by comparing figure 15 with figure 5
figure(15); 
bodemag(K1,'b',b,w)
title('Controller with four-block mixed sensitivity')

% We further see from the pole-zero map (figure 16 vs figure 6) that there 
% are no pole-zero cancellations any more.
figure(16);
pzplot(G,K1,'r')
xlim([-1.5, 1])
title('PZmap with four-block mixed sensitivity')

% If we plot the responses to the reference step, the result looks much
% worse than with the S/KS controller (Figure 17 vs figure 7).
% The large overshoot is loosely speaking  aused by the additional loop 
% gain in the low frequency range. If you take a close look at the 
% sensitivity plot in figure 14, you can see that
% the sensitivity now has a +40dB per decade slope, which is 20dB more than
% we desired.This is however a consequence of the fact that we need 
% integral action to reject the disturbances and that in the sensitivity 
% function, both integral behavior from the plant and the controller will 
% add up.
figure(17);
step(loops.To,'b',loops.CSo,'g',10)
legend('y','u')
title('Step response four-block mixed sensitivity')


% The disturbance response however looks much better than before 
%(Figure 18 vs 8).
figure(18); 
step(loops.PSi,'b',10)
title('Disturbance response four-block mixed sensitivity')

% If we test for robustness, we note that it is decreased.
% Again, this is a consequence of the larger controller gains, or more
% precisely of the larger control bandwidth with the four block design. 
mm = loopmargin(XXX,'m');
minGM = db(mm.GainMargin(2))
minPM = min(mm.PhaseMargin(2))

% We see, that while we indeed are able to improve disturbance rejection,
% we pay a price for this in terms of trajectory following as well as
% robustness. Such trade-offs are inherent to feedback control design.
% Experiment with the four block setup to confirm this on your own. In 
% general, decreasing Wd brings the design closer to the S/KS design, i.e.
% It should improve trajectory tracking and robustness, while the
% disturbance rejection gets worse.

%% 4.2 Two-degrees-of-freedom design

% If you played with the tuning of the four block problem, you might have
% realized that it is very hard to achieve satisfactory tracking
% performance and disturbance rejection simultaneously. The main reason is, 
% that disturbance rejection and tracking are coupled. Since the controller 
% receives only the error signal, both unknown disturbances and known 
% set-point changes enter the control in the same way: as an error that 
% needs to be rejected. We can however take  advantage of the fact that 
% they are not the same. This is referred to as two-degrees-of-freedom 
% control and is usually advantageous for tracking  applications. 
% We consider a relatively simple modification of the four block 
% problem that yields a two-degrees-of-freedom controller. 

% The only difference is, that the controller now receives the reference
% signal and the measured output separately. The generalized plant 
% formulation thus has an additional input r that is used to form the 
% control error (input_to_W_S) but that is also directly provided to the 
% controller (outputvar).
systemnames = 'G Wd W_S W_K';
inputvar = ['[w1{2}; w2{2}; r{2}; u{2}]'];
input_to_G = '[XXX]';
input_to_Wd = '[XXX]';
input_to_W_S = ['[XXX]'];
input_to_W_K = '[XXX]';
outputvar = ['[W_S; W_K; w1-G; r]'];
cleanupsysic = 'yes'; 
P = sysic


% Again, we use the exact same synthesis except that we also have 
% the reference signals available, so nmeas is now 4.
nmeas = 2+2; 
ncont = 2;
[~,~,gam,~] = hinfsyn(XXX);
[K2,CL,gam,INFO] = hinfsyn(XXX);
gam
stab = isstable(CL)
normOK = norm(CL,'inf')<gam
fastestpole = max(abs(eig(K2.a)))

% The controller now has four inputs (Figure 25). The first two are meant
% to be connected to the measured outputs of the plants and are thus
% feedback paths. You can access the feedback paths as K2(:,1:2)
% The last two receive the reference signal and consequently are 
% feedforward paths. You can access them as K2(:,3:4)
% Since robustness as well as the  closed-loop transfer functions that we 
% are interested in depend only on  the feedback path, we have to use just 
% that part of the controller associated with the first two inputs for analysis.
loops = loopsens(XXX);
figure(24);
subplot(2,2,1)
sigma(loops.So,W_S^-1,w)
title('S')
subplot(2,2,2)
sigma(loops.PSi,w)
title('SG')
subplot(2,2,3)
sigma(loops.CSo,(W_K)^-1,w)
title('KS')
subplot(2,2,4)
sigma(loops.Ti,w)
title('Ti')

figure(25); 
bodemag(K2,'b',b,w)
title('Controller with two-degrees-of-freedom')

% We can build a closed loop (which is now a little more complicated
% since it involves the two-degree-of-freedom controller) using sysic
systemnames = 'G K2';
inputvar = ['[di{2}; r{2}]'];
input_to_G = '[K2+di]';
input_to_K2 = '[-G;r]';
outputvar = ['[G; K2]'];
cleanupsysic = 'yes';
CL = sysic;
% Again, the only real difference is, that the controller receives the
% measurement and the reference as two independent signals and the we are
% interested in both outputs and control signals as outputs.

% The step response to changes in the reference signal now looks
% exactly as we like it. Now overshoot, and almost no oscillations. 
% (Figure 27 vs Figure 17 vs Figure 7)
figure(27);
step(CL(1:2,3:4),'b',CL(3:4,3:4),'g',10)
legend('y','u')
 title('Step response two-degrees-of-freedom')
% The response to the input disturbance looks also quite nice and very
% similar to the one that we obtained from the four block design.
% Figure 28 vs Figure 18 vs Figure 8)
figure(28);
step(CL(1:2,1:2),'b',10)
title('Disturbance response two-degrees-of-freedom')

% Also, robustness of the feedback path should be pretty much the same as 
% with the four block design.
mm = loopmargin(XXX,'m');
minGM = db(mm.GainMargin(2))
minPM = min(mm.PhaseMargin(2))

% The two-degrees-of-freedom design thus helps to improve tracking
% performance while we still can get the benefits of integral disturbance
% rejections. It thus relieves the trade-off between tracking performance
% and disturbance rejection. The second inherent trade-off, namely
% disturbance rejection and robustness is however still very visible and we
% have to pay close attention which is more important to us in a certain
% design.




%% FINAL COMPARISON
% let's take a final look at the three different designs to appreciate the
% differences. 

% Closed-Loop transfer functions
ShowAllFig([4,14,24])
pause

% Controller
ShowAllFig([5,15,25])
pause

% Disturbance Responses
% Closed-Loop transfer functions
ShowAllFig([7,17,27])
pause

% Step Responses
% Closed-Loop transfer functions
ShowAllFig([8,18,28])



%% FOR THE EXPERIMENT: USE ALL AVAILABLE MEASUREMENTS
%
% All angles can be measured and the velocities can be estimated by first
% order differentiation filters. Use these additional information to
% improve the design!! 
%
% Note that for instance q2_dot is not visible for the controller, so it
% does not try to improve damping even though this is desireable on the
% actual plant!
%
% Further note that the model inaccuracy due to neglecting the
% differentiation filters in the model is addressed in the controller
% design, since we assume measurement noise on all the channels. This is an
% important difference to directly using state feedback, in which not even
% the slightest disturbance in the states would be allowed from a
% theoretical point of view.


% Plant model with two outputs:
Gp = linearize_gyro(45,0,0);
% Add the other three states as outputs (q2dot,q3dot,q4dot)
Gp = ss(Gp.a, Gp.b, [Gp.c; 0 0 1 0 0 ; 0 0 0 1 0 ; 0 0 0 0 1], 0) ;

% Note how the singular values change, i.e. the peak is even more pronounced
figure(31)
bodemag(Gp,b,w)
title('Plant')
figure(32)
sigma(Gp,w)
title('Plant')

% Scaling again
umax = [0.666,  2.44];
rmax = [10/180*pi, 45/180*pi, 1, 1, 1];

Du = diag(umax);
Dr = diag(rmax);
G = Dr\Gp*Du;


%% Two-degrees-of-freedom design with 5 outputs
%
% Design a two-degrees-of-freedom controller K3 that uses all 5 measurements.
%
% Hint: 
% We want to just track the first two inputs, but simultaneously 
% reduce disturbances on the three other outputs, i.e. increase damping.
% Thus, W_S now is 5 by 5 and the input to W_S  looks something like
% ['[w1(1:2)-G(1:2)+r ; w1(3:5)-G(3:5)]'];
% Make sure you understand that we CANNOT track all 5 outputs and hence are
% not allowed to weight all of them with integral behavior (This is called
% 'underactuated')


% \   /  \   /  \   / 
%  \ /    \ /    \ /  
%   X      X      X 
%  / \    / \    / \ 
% /   \  /   \  /   \


%% Evaluation of the controller in frequency and time domain
% The controller now has seven inputs (Figure 35). The first 5 are meant
% to be connected to the measured outputs of the plants and are thus
% feedback paths. The last two receive the reference signal and 
% consequently are feedforward paths. 

loops = loopsens(G,K3(:,1:5));
figure(34);
subplot(2,3,1)
sigma(loops.So,gam*W_S^-1,w)
title('S')
subplot(2,3,2)
sigma(loops.PSi,gam*W_S^-1,w)
title('SG')
subplot(2,3,4)
sigma(loops.CSo,gam*W_K^-1,w)
title('KS')
subplot(2,3,5)
sigma(loops.Ti,gam*W_K^-1,w)
title('Ti')

% feedforward properties
forwardS = loops.PSi*K3(:,6:7);
subplot(2,3,3)
sigma(eye(2)-forwardS(1:2,:),gam*W_S^-1,w)
title('I-SGKr')
subplot(2,3,6)
sigma(loops.Si*K3(:,6:7),gam*W_K^-1,w)
title('SiKr')

%  closed loop 
systemnames = 'G K3';
inputvar = ['[di{2}; r{2}]'];
outputvar = ['[G; K3]'];
input_to_G = '[K3+di]';
input_to_K3 = '[-G;r]';
cleanupsysic = 'yes';
CL = sysic;

% step responses 
figure(37); 
step(CL(1:2,3:4),'b',CL(3:4,3:4),'g',10)
legend('y','u')
ylim([-1.25,1.75])
title('Step response two-degrees-of-freedom with all measurements')
% The response to the input disturbance looks also quite nice and very
% similar to the one that we obtained from the four block design.
% Figure 28 vs Figure 18 vs Figure 8)
figure(38); 
step(CL(1:2,1:2),'b',10)
title('Disturbance response two-degrees-of-freedom with all measurements')


%% COMPARISON WHY INCLUDING VELOCITIES IS USEFUL
% (this time against the other 2dof only)
% To appreciate the difference to the controller without the additional
% measurements, let's build a closed loop for the plant with 5 outputs but
% using only the 2x2 controller K2.

loops = loopsens(G,K2(:,1:2)*eye(2,5));
figure(24);
subplot(2,3,1)
sigma(loops.So,w)
title('S')
subplot(2,3,2)
sigma(loops.PSi,w)
title('SG')
subplot(2,3,4)
sigma(loops.CSo,w)
title('KS')
subplot(2,3,5)
sigma(loops.Ti,w)
title('Ti')

% feedforward properties
forwardS = loops.PSi*K2(:,3:4);
subplot(2,3,3)
sigma(eye(2)-forwardS(1:2,:),w)
title('I-SGKr')
subplot(2,3,6)
sigma(loops.Si*K2(:,3:4),w)
title('SiKr')

% Closed-Loop transfer functions
ShowAllFig([24,34])
pause

% Disturbance Responses
% Closed-Loop transfer functions
ShowAllFig([27,37])
pause

% Step Responses
% Closed-Loop transfer functions
ShowAllFig([28,38])
