function [particles,weights,P] = MSVResampling(particles,weights,P)

% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% MSVResampling.m
% 
% Purpose: Optimally resample distribution to minimizes the sampling variance
%
% Input: particles
%        weights
%
% Output: particles
%         weights
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

% Perform MSV resampling
cumulativeWeights = cumsum(weights); % Calculate cumulative sum of weights
numParticles = numel(weights);
indices = zeros(1, numParticles);

% MSV Optimization
targetWeights = linspace(0,1,numParticles)+rand(1,numParticles)/numParticles; % Generate target weights
for i = 1:numParticles
    [~,indices(i)] = min(abs(cumulativeWeights-targetWeights(i))); % Find the closest cumulative weight index
end

% Update particles and weights based on MSV resampling
particles = particles(:,indices); % Resample particles based on indices
P = P(:,:,indices);
weights = ones(size(weights)) / numParticles;

end