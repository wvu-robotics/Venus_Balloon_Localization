function [Q] = Q_INS(ins_att,ins_pos,f_ib_b,sigma_rg,sigma_ra,sigma_bgd,sigma_bad,dt)

% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% EKF_Measurement_Update.m
% 
% Purpose: Localization Simulation using FGO with PF
%
% Primary Developer Contact Information:
% Heath Cottrill
% Student
% Statler College of Engineering & Mineral Resources
% Dept. Mechanical and Aerospace Engineering
% West Virginia University (WVU)
% hsc0009@mix.wvu.edu
%
% Development History
% Date              Developer        Comments
% ---------------   -------------    --------------------------------
% Feb. 27, 2023     H. Cottrill      Initial implemention
%

% Parameters of Venus
e = 0;
R_0 = 6051800.0;

% The radius of curvature for north-south motion (Eq. 2.65)
R_N = R_0*(1-e^2)/(1-e^2*sin(ins_pos(1))^2)^(3/2);

% The radius of curvature for east-west motion (Eq. 2.66)
R_E = R_0/(1-e^2*sin(ins_pos(1))^2)^(1/2);

% (Eq. 12.62)
C_n_b = eulr2dcm(ins_att);
vector = -(C_n_b*f_ib_b);
F21 = [0        -vector(3) vector(2);
       vector(3) 0        -vector(1);
      -vector(2) vector(1) 0];  

% Power Spectral Density
S_rg = sigma_rg^2*dt;
S_ra = sigma_ra^2*dt;
S_bad = sigma_bad^2*dt;
S_bgd = sigma_bgd^2*dt;

% Cartesian-to-Curvilinear Positin Change Matrix
T = [1/(R_N+ins_pos(3)) 0                                     0;
     0                  1/((R_E+ins_pos(3))*cos(ins_pos(1)))  0;
     0                  0                                    -1];

% Row 1
Q11 = (S_rg*dt+1/3*S_bgd*dt^3)*eye(3);
Q12 = ((1/2*S_rg*dt^2+1/4*S_bgd*dt^4)*F21)';
Q13 = ((1/3*S_rg*dt^3+1/5*S_bgd*dt^5)*T*F21)';

% Row 2
Q21 = Q12';
Q22 = (S_ra*dt+1/3*S_bad*dt^3)*eye(3)+(1/3*S_rg*dt^3+1/5*S_bgd*dt^5)*(F21*F21');
Q23 = ((1/2*S_ra*dt^2+1/4*S_bad*dt^4)*T+(1/4*S_rg*dt^4+1/6*S_bgd*dt^6)*T*F21*F21*T)';

% Row 3
Q31 = Q13';
Q32 = Q23';
Q33 = (1/3*S_ra*dt^3+1/5*S_bad*dt^5)*T*T+(1/5*S_rg*dt^5+1/7*S_bgd*dt^7)*T*F21*F21*T;


% System Noise Covariance Matrix
Q = [Q11, Q12, Q13;
     Q21, Q22, Q23;
     Q31, Q32, Q33];

end