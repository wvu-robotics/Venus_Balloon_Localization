function [x,P] = EKF(ins_att,ins_vel,ins_pos,f_ib_b,Q,dt,P,R,delta_z,H)

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

% The first term is due to the rotation of the Earth with respect to an inertial frame

% So the skew-symmetric matrix is the following noting it is a function of latitude (Eq. 5.34)
omega_ie = 2.9924e-7; % rad/s
omega_n_ie = [omega_ie*cos(ins_pos(1));
              0;
             -omega_ie*sin(ins_pos(1))];

% The second term arises from the rotation of the local-navigation frame with respect to Earth

% Parameters of Venus
e = 0;
g = gravity(ins_pos(3));
R_0 = 6051800.0;

% The radius of curvature for north-south motion (Eq. 2.65)
R_N = R_0*(1-e^2)/(1-e^2*sin(ins_pos(1))^2)^(3/2);

% The radius of curvature for east-west motion (Eq. 2.66)
R_E = R_0/(1-e^2*sin(ins_pos(1))^2)^(1/2);
omega_n_en = [ins_vel(2)/(R_E+ins_pos(3));
             -ins_vel(1)/(R_N+ins_pos(3));
            (-ins_vel(2)*tan(ins_pos(1)))/(R_E+ins_pos(3))];

% The total rotation
omega_n_in = omega_n_en+omega_n_ie;

% (Eq. 12.59)
F11 = -[0            -omega_n_in(3) omega_n_in(2);
        omega_n_in(3) 0            -omega_n_in(1);
       -omega_n_in(2) omega_n_in(1) 0];   

% (Eq. 12.60)
F12 = [0,                  -1/(R_E+ins_pos(3)),               0;
       1/(R_N+ins_pos(3)),  0,                                0;
       0,                   tan(ins_pos(1))/(R_E+ins_pos(3)), 0];

% (Eq. 12.61)
F13 = [omega_ie*sin(ins_pos(1)),                                                 0,  ins_vel(2)/(R_E+ins_pos(3))^2;
       0,                                                                        0, -ins_vel(1)/(R_N+ins_pos(3))^2;
       omega_ie*cos(ins_pos(1))+ins_vel(2)/((R_E+ins_pos(3))*cos(ins_pos(1))^2), 0, -ins_vel(2)*tan(ins_pos(1))/(R_E+ins_pos(3))^2];

% (Eq. 12.62)
C_n_b = eulr2dcm(ins_att);
vector = -(C_n_b*f_ib_b);
F21 = [0        -vector(3) vector(2);
       vector(3) 0        -vector(1);
      -vector(2) vector(1) 0];  

% (Eq. 12.63)
F22 = [ins_vel(3)/(R_N+ins_pos(3)),                                            -(2*ins_vel(2)*tan(ins_pos(1))/(R_E+ins_pos(3)))-2*omega_ie*sin(ins_pos(1)), ins_vel(1)/(R_N+ins_pos(3));
      (ins_vel(2)*tan(ins_pos(1))/(R_E+ins_pos(3)))+2*omega_ie*sin(ins_pos(1)), (ins_vel(1)*tan(ins_pos(1))+ins_vel(3))/(R_E+ins_pos(3)),                  (ins_vel(2)/(R_E+ins_pos(3)))+2*omega_ie*cos(ins_pos(1));
      -2*ins_vel(1)/(R_N+ins_pos(3)),                                           -2*(ins_vel(2)/(R_E+ins_pos(3)))-2*omega_ie*cos(ins_pos(1)),                0];

% (Eq. 12.64)
F23 = [-(((ins_vel(2)^2)*sec(ins_pos(1))^2)/(R_E+ins_pos(3)))-(2*ins_vel(2)*omega_ie*cos(ins_pos(1))),                                          0, (((ins_vel(2)^2)*(tan(ins_pos(1))))/((R_E+ins_pos(3))^2))-((ins_vel(1)*ins_vel(3))/(R_N+ins_pos(3))^2);
      ((ins_vel(1)*ins_vel(2)*sec(ins_pos(1))^2)/(R_E+ins_pos(3)))+2*ins_vel(1)*omega_ie*cos(ins_pos(1))-2*ins_vel(3)*omega_ie*sin(ins_pos(1)), 0, -((ins_vel(1)*ins_vel(2)*tan(ins_pos(1))+ins_pos(2)*ins_pos(3))/(R_E+ins_pos(3))^2);
       2*ins_vel(2)*omega_ie*sin(ins_pos(1)),                                                                                                   0, (((ins_vel(2)^2)/(R_E+ins_pos(3))^2)+((ins_vel(1)^2)/(R_N+ins_pos(3))^2)-((2*g)/(R_0)))];

% (Eq. 12.65)
F32 = [1/(R_N+ins_pos(3)),  0,                                    0;
       0,                   1/((R_E+ins_pos(3))*cos(ins_pos(1))), 0;
       0,                   0,                                   -1]; 

% (Eq. 12.66)
F33 = [0,                                                                 0, -ins_vel(1)/(R_N+ins_pos(3))^2;
       (ins_vel(2)*sin(ins_pos(1)))/((R_E+ins_pos(3))*cos(ins_pos(1))^2), 0, -(ins_vel(2))/(((R_E+ins_pos(3))^2)*cos(ins_pos(1)));
       0,                                                                 0,  0]; 

% Row 1
Phi11 = eye(3)+F11*dt;
Phi12 = F12*dt;
Phi13 = F13*dt;

% Row 2
Phi21 = F21*dt;
Phi22 = eye(3)+F22*dt;
Phi23 = F23*dt;

% Row 3
Phi31 = zeros(3);
Phi32 = F32*dt;
Phi33 = eye(3)+F33*dt;

% Transition Matrix
Phi = [Phi11, Phi12, Phi13;
       Phi21, Phi22, Phi23;
       Phi31, Phi32, Phi33];

% Error Covariance Matrix
P = real(Phi*P*Phi'+Q);

% Kalman Gain Matrix
K = (P*H')*pinv(H*P*H'+R);

% State Vector (w/ Measurement)
delta_x = zeros(9,1);
delta_x = K*(delta_z-H*delta_x);

% Attitude
ins_att = dcm2eulr(eulr2dcm(delta_x(1:3))'*C_n_b);

% Velocity
ins_vel = ins_vel-delta_x(4:6);

% Position
ins_pos = ins_pos-delta_x(7:9);

% Total State Vector
x = real([ins_att;ins_vel;ins_pos]);
if anynan(x)
    x = inf(9,1);
end

end