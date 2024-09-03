% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% Simulation_FGO.m
% 
% Purpose: Balloon Localization Simulation using FGO
%          ** Requires GTSAM Matlab Toolbox**
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
% August 2, 2024    H. Cottrill      Initial implemention
%

%% Cleaning

% Clean
clear
close
clc

% Display Progress
disp('Cleaning...')

% Start timer
tic

%% Initialization

% Display Progress
disp('Initialization...')

% Load Data
load('Test_1.mat')
data = cell(length(trials),1); 

% Time
dt = 1; % Time step [s]
dt_fgo = 15;  % Factor Graph Time Step [s]
time = 0:dt:1*60*60; % Simulation Duration
time_no_gps = 1*60*60; % Time w/o GPS [s]
time_gps = 5*60; % Time w/ GPS [s]
L = round(time(end)/dt)+1; 
gps_time = (time >= 1*time_no_gps-time_gps & time <= 1*time_no_gps) | (time <= time_gps); % GPS availability
fgo_time = (time == time(end)); % FGO update
 
% Accelerometer
bias_static_accel = 1e-3; % Static bias of acceleration [m/s^2]
bias_dynamic_accel = 1e-4; % Dynamic bias of acceleration [m/s^2]
random_noise_accel = 1e-5; % Random noise of acceleration [m/s^2]

% Gyroscope
bias_static_gyro = 5e-8; % Static bias of angular velocity [rad/s]
bias_dynamic_gyro = 5e-9; % Dynamic bias of angular velocity [rad/s]
random_noise_gyro = 5e-10; % Random noise of angular velocity [rad/s]

% Measurement Noise
sigma_topo = 1; % Measurement noise of height measure [m]
sigma_baro = 10; % Measurement noise of pressure measure [Pa]
sigma_gps_pos = 100; % Measurement noise of GPS position measure [m]
sigma_gps_vel = 1; % Measurement noise of GPS velocity measure [m/s]

% Topography
load('topography_4641m.mat')

% Barometry
load('barometry.mat')

%% Factor Graph

% Display Progress
disp('Simulation...')

for i = 1:length(trials)

    % Extract data
    trial = trials{i};

    % Import GTSAM
    addpath('/usr/local/gtsam_toolbox')
    import gtsam.*
    import example.*
    
    % Create a Levenberg-Marquardt Optimizer Object
    params = LevenbergMarquardtParams();
    params.setVerbosityLM('Summary');
    
    % Create the environment
    newFactors = NonlinearFactorGraph;
    newValues = Values;
    
    % Include gravity
    imuParams = PreintegrationCombinedParams.MakeSharedD(gravity(trial.truth.pos_llh(3,1)));
    
    % Define noise parameters
    imuParams.setAccelerometerCovariance(random_noise_accel^2*eye(3))
    imuParams.setGyroscopeCovariance(random_noise_gyro^2*eye(3))
    imuParams.setIntegrationCovariance((1e-3)^2*eye(3))
    
    % Establish bias parameters
    imuParams.setBiasAccCovariance(bias_dynamic_accel^2*eye(3))
    imuParams.setBiasOmegaCovariance(bias_dynamic_gyro^2*eye(3))
    imuParams.setBiasAccOmegaInit(diag([bias_static_accel^2*ones(3,1);bias_static_gyro^2*ones(3,1)]))
    
    % Define the PreintegratedImuMeasurements object here
    pim = PreintegratedCombinedMeasurements(imuParams,imuBias.ConstantBias([0;0;0],[0;0;0]));
    
    % Noise Models
    sigma_X = noiseModel.Isotropic.Sigmas([deg2rad(1e-3)*ones(3,1);(1e-3)*ones(3,1)]);
    sigma_V = noiseModel.Isotropic.Sigma(3,1e-3);
    sigma_B = noiseModel.Isotropic.Sigmas([bias_static_accel*ones(3,1);bias_static_gyro*ones(3,1)]);
    sigma_G = noiseModel.Isotropic.Sigma(2,bias_static_gyro);
    sigma_GPS = noiseModel.Isotropic.Sigma(3,sigma_gps_pos);
    sigma_Topo = noiseModel.Isotropic.Sigma(1,sigma_topo);
    sigma_Baro = noiseModel.Isotropic.Sigma(1,sigma_baro);
    
    % Initial Values
    newValues.insert(symbol('X',0),Pose3(Rot3(eulr2dcm(trial.truth.att(:,1))),Point3(trial.truth.pos_ned(:,1))))
    newValues.insert(symbol('V',0),trial.truth.vel(:,1))
    newValues.insert(symbol('B',0),imuBias.ConstantBias([0;0;0],[0;0;0]))
    
    % Initial Factors
    newFactors.add(PriorFactorPose3(symbol('X',0),Pose3(Rot3(eulr2dcm(trial.truth.att(:,1))),Point3(trial.truth.pos_ned(:,1))),sigma_X))
    newFactors.add(PriorFactorVector(symbol('V',0),trial.truth.vel(:,1),sigma_V))
    newFactors.add(PriorFactorConstantBias(symbol('B',0),imuBias.ConstantBias([0;0;0],[0;0;0]),sigma_B))
    
    % Store Data
    trial.fgo.att(:,1) = trial.truth.att(:,1);
    trial.fgo.vel(:,1) = trial.truth.vel(:,1);
    trial.fgo.pos_ned(:,1) = trial.truth.pos_ned(:,1);
    trial.fgo.pos_llh(:,1) = trial.truth.pos_llh(:,1);
    trial.fgo.bias_f_ib_b(:,1) = [0;0;0];
    trial.fgo.bias_omega_b_ib(:,1) = [0;0;0];
    trial.fgo.state(:,1) = [trial.fgo.att(:,1);trial.fgo.vel(:,1);trial.fgo.pos_ned(:,1);trial.fgo.bias_f_ib_b(:,1);trial.fgo.bias_omega_b_ib(:,1)];
    
    % Initialize Factor Graph
    j = 0;
    
    % Iterate for all time steps less the first
    for k = 2:L
    
        % Integrated Measurement
        pim.integrateMeasurement(trial.truth.f_ib_b(:,k)+trial.truth.bias_f_ib_b(:,k),trial.truth.omega_b_ib(:,k)+trial.truth.bias_omega_b_ib(:,k),dt);
    
        if rem(k-1,dt_fgo) == 0
    
            % Update counter
            j = j+1;
    
            % Create IMU factor
            newFactors.add(CombinedImuFactor(symbol('X',j-1),symbol('V',j-1),symbol('X',j),symbol('V',j),symbol('B',j-1),symbol('B',j),pim));
    
            % Create Barometry Factor
            baroIn = GetBarometryData(trial.truth.pos_llh(:,k),barometry)+sigma_baro*randn(1);
            newFactors.add(BarometryFactor(symbol('X',j),baroIn,trial.truth.pos_llh(:,1),sigma_Baro));
    
            % Create Gravity Factor
            nZ = Unit3(trial.truth.f_ib_b(:,k)/norm(trial.truth.f_ib_b(:,k)));
            newFactors.add(Pose3AttitudeFactor(symbol('X',j),nZ,sigma_G,Unit3([0,0,-1]')))
            
            % If GPS available
            if gps_time(k)
    
                % Create GPS Factor
                newFactors.add(GPSFactor(symbol('X',j),trial.truth.pos_ned(:,k)+sigma_gps_pos.*randn(3,1),sigma_GPS));
    
            end
    
            % Initial State Estimate
            newValues.insert(symbol('X',j),Pose3(Rot3(eulr2dcm(trial.rbpf.att(:,k)')),Point3(trial.rbpf.pos_ned(:,k))))
            newValues.insert(symbol('V',j),Point3(trial.rbpf.vel(:,k)))
            newValues.insert(symbol('B',j),imuBias.ConstantBias([0;0;0],[0;0;0]))
    
            % FGO Update
            if fgo_time(k)

                % Display Progress
                fprintf('Trial #%.0f\n',i)
    
                % Create a Levenberg-Marquardt Optimizer Object
                optimizer = LevenbergMarquardtOptimizer(newFactors,newValues,params);
    
                % Solve Optimization Problem
                result = optimizer.optimize();
    
                % Counter
                count = 1;
    
                % Iterate for all nodes
                for m = 1:j+1
    
                    % Attitude
                    trial.fgo.att(:,m) = result.atPose3(symbol('X',m-1)).rotation.rpy;

                    % Position
                    trial.fgo.pos_ned(:,m) = result.atPose3(symbol('X',m-1)).translation;
                    trial.fgo.pos_llh(:,m) = NED2LLH(trial.fgo.pos_ned(:,m),trial.truth.pos_llh(:,1));
    
                    % Velocity
                    if count == 1
                        trial.fgo.vel(:,m) = trial.rbpf.vel(:,k);
                    else
                        trial.fgo.vel(3,m) = (-2/dt_fgo)*(trial.fgo.pos_llh(3,m)-trial.fgo.pos_llh(3,m-1))-trial.fgo.vel(3,m-1);
                        trial.fgo.vel(1,m) = ((2/dt_fgo)*(trial.fgo.pos_llh(1,m)-trial.fgo.pos_llh(1,m-1))-trial.fgo.vel(1,m-1)/(6051800+trial.fgo.pos_llh(3,m-1)))*(6051800+trial.fgo.pos_llh(3,m));
                        trial.fgo.vel(2,m) = ((2/dt_fgo)*(trial.fgo.pos_llh(2,m)-trial.fgo.pos_llh(2,m-1))-trial.fgo.vel(2,m-1)/((6051800+trial.fgo.pos_llh(3,m-1))*cos(trial.fgo.pos_llh(1,m-1))))*((6051800+trial.fgo.pos_llh(3,m))*cos(trial.fgo.pos_llh(1,m)));
                    end
    
                    % Bias
                    trial.fgo.bias_f_ib_b(:,m) = result.atConstantBias(symbol('B',m-1)).accelerometer;
                    trial.fgo.bias_omega_b_ib(:,m) = result.atConstantBias(symbol('B',m-1)).gyroscope;

                    % Counter
                    count = count+1;
    
                end

                % Update Cell
                data{i} = trial;
              
            end
    
            % Clean out preintegration value
            pim.resetIntegration()
    
        end
    
        % Display Progress
        if(mod(k,0.01*(L-1))==0)
            progress = k/(L-1)*100;
            fprintf('%.0f%% \n',progress);
        end
    end
end

% Stop timer
fgo_duration = toc;
disp(fgo_duration/60)

% Display Progress
disp('Saving Data...')

% Save data
save('Test_1_FGO.mat', 'data', '-v7.3')