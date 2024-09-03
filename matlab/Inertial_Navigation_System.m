function [ins_att_plus,ins_vel_plus,ins_pos_plus] = Inertial_Navigation_System(ins_att_minus,ins_vel_minus,ins_pos_minus,omega_b_ib,v_ib_b,dt)

% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% Inertial_Navigation_System.m
% 
% Purpose: Function to propogate pose based on previous pose and time step
% 
% Input [ins_att_minus,ins_vel_minus,ins_pos_minus,omega_b_ib,v_ib_b,dt]:
%       ins_att_minus: Euler angles
%       ins_vel_minus: Linear velocities
%       ins_pos_minus: Latitude, Longitude, Height 
%       omega_b_ib: Angular rate (gyroscope data)
%       v_ib_b: Acceleration (accelerometer data)
%		dt: Sampling time
%
% Output [ins_att_plus,ins_vel_plus,ins_pos_plus,ins_pos_plus_xyz]: pose at timestep k+1
%       ins_att_plus: Euler angles
%       ins_vel_plus: Linear velocities
%       ins_pos_plus: Latitude, Longitude, Height 
%       ins_pos_plus: x, y, z
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
% Dec. 4, 2023      H. Cottrill      Initial implemention
%

%%% Attitude update %%%

% The original attitude in DCM from Euler representation
C_n_b_minus = eulr2dcm(ins_att_minus);

% The first term is due to the inertially referenced angular rate, measured by the gyros

% The angular-rate measurement from the IMU (Eq 2.26)
Omega_b_ib= [0            -omega_b_ib(3) omega_b_ib(2);
             omega_b_ib(3) 0            -omega_b_ib(1);
            -omega_b_ib(2) omega_b_ib(1) 0];

% The second term is due to the rotation of the Earth with respect to an inertial frame

% So the skew-symmetric matrix is the following noting it is a function of latitude (Eq. 5.34)
omega_ie = 2.9924e-7; % rad/s
Omega_n_ie = omega_ie * [0                     sin(ins_pos_minus(1)) 0;
                        -sin(ins_pos_minus(1)) 0                    -cos(ins_pos_minus(1));
                         0                     cos(ins_pos_minus(1)) 0];

% The third term arises from the rotation of the local-navigation-frame axes as the frame center moves with respect to the Earth.

% Parameters of Venus
e = 0;
R_0 = 6051800.0;

% The radius of curvature for north-south motion (Eq. 2.65)
R_N = R_0*(1-e^2)/(1-e^2*sin(ins_pos_minus(1))^2)^(3/2);

% The radius of curvature for east-west motion (Eq. 2.66)
R_E = R_0/(1-e^2*sin(ins_pos_minus(1))^2)^(1/2);

% The rotation rate vector (Eq. 5.37)
omega_n_en = [ins_vel_minus(2)/(R_E+ins_pos_minus(3));
             -ins_vel_minus(1)/(R_N+ins_pos_minus(3));
            (-ins_vel_minus(2)*tan(ins_pos_minus(1)))/(R_E+ins_pos_minus(3))];

% The skew-symetric matrix alternative
Omega_n_en = [0            -omega_n_en(3) omega_n_en(2);
              omega_n_en(3) 0            -omega_n_en(1);
             -omega_n_en(2) omega_n_en(1) 0];

% The update of the coordinate transformation matrix
C_n_b_plus_dot = C_n_b_minus*Omega_b_ib-(Omega_n_ie+Omega_n_en)*C_n_b_minus;
C_n_b_plus = C_n_b_minus+C_n_b_plus_dot*dt;

% The resulting attitude in Euler from DCM representation
ins_att_plus = dcm2eulr(C_n_b_plus);


%%% Velocity Update %%%

% Specific-Force Frame Transformation (Eq. 5.41)
V_n_ib = 1/2*(C_n_b_minus+C_n_b_plus)*v_ib_b;

% The rotation rate vector (Eq. 5.37)
omega_n_en = [ins_vel_minus(2)/(R_E+ins_pos_minus(3));
             -ins_vel_minus(1)/(R_N+ins_pos_minus(3));
            (-ins_vel_minus(2)*tan(ins_pos_minus(1)))/(R_E+ins_pos_minus(3))];

% The skew-symetric matrix alternative
Omega_n_en = [0            -omega_n_en(3) omega_n_en(2);
              omega_n_en(3) 0            -omega_n_en(1);
             -omega_n_en(2) omega_n_en(1) 0];

% The update of the velocity considering the specific force, gravity, and centrifugal acceleration (Eq. 5.47)
ins_vel_plus = ins_vel_minus+V_n_ib+([0;0;gravity(ins_pos_minus(3))]-(Omega_n_en+2*Omega_n_ie)*ins_vel_minus)*dt;

%%% Position Update %%%

% The height update (Eq. 5.49)
ins_pos_plus(3,:) = ins_pos_minus(3)-dt/2*(ins_vel_minus(3)+ins_vel_plus(3));

% The latitude update (Eq. 5.49)
ins_pos_plus(1,:) = ins_pos_minus(1)+dt/2*(ins_vel_minus(1)/(R_N+ins_pos_minus(3))+ins_vel_plus(1)/(R_N+ins_pos_plus(3)));

% The new radius of curvature for east-west motion
R_E_plus= R_0/(1-e^2*sin(ins_pos_plus(1))^2)^(1/2);

% The longitude update (Eq. 5.49)
ins_pos_plus(2,:)=ins_pos_minus(2)+dt/2*(ins_vel_minus(2)/((R_E+ins_pos_minus(3))*cos(ins_pos_minus(1)))+ins_vel_plus(2)/((R_E_plus+ins_pos_plus(3))*cos(ins_pos_plus(1))));

end