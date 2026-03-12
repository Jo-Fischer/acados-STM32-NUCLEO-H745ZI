%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File: Parameters_FurutaPendulum.m
%
% Authors:  Jörg Fischer
%           
% Created:  
%
% Description:  Assignment of values for physical quantities of the 
%               Furuta-Pendulum
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% General parameters
% Gravitation
    grav = 9.81;
% Smoothing value for tanh approxmation of sign()
    k_factor = 2;

%% Motor
% pwm unit voltage facotr
    kSteller = 1.0983;
% Torque constant
    kM = 0.0236;
% inductance constant of motor
    %kE = 0.0867;
% Motor resistance
    RM = 0.5200;

%% Actuator arm
% vicous friction in actuator arm joint
    b1vis = 6.0700e-04;
% coulomb frcition in actuator arm joint
    b1coul = 0.0104;
% inertia of pendulum (hanging down) and actuator arm with respect to motor axis
    hJ0 =0.0021;
% Length of actuator arm
    L1 = 0.1215;
    
%% Pendulum
% mass
    m2 = 0.0461;
% length from pendulum joint to center of mass of pendulum
    l2 = 0.0790;
% viscious frcition of pendulum
    b2vis = 2.6830e-05;
% coulomb frcition of pendulum
    b2coul = 7.5418e-04;
% inertial of pendulum mass with respect to pendulum joint
    hJ2 = 3.7054e-04;