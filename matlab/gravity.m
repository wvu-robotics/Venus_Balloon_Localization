function g = gravity(z)

% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% gravity.m
% 
% Purpose: Determine gravitational acceleration based on altitude
%
% Input: z = altitude in meters
%
% Output: g = gravitational acceleration in meters per second squared
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
% July 10, 2023     H. Cottrill      Initial implemention
%

% Gravitational Constant
G = 6.67408E-11; % m^3/(Kg*s^2)

% Mass of Venus 
M = 4.8673E24; % kg

% Radius of Venus
r = 6051.8E3;    % m

% Gravity of Venus at altitude h
g = (G*M)./((r+z).^2);

end
   