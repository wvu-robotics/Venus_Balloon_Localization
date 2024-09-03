% Development Information
% WVU Interactive Robotics Laboratory
% Venus Localization Simulator
% Simulation_Single.m
%
% Purpose: Balloon Localization Simulation
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

% Start Timer
tic

% Anemology
load('anemology.mat','anemology','longitudes','latitudes','heights')
e_wind = griddedInterpolant({longitudes,latitudes,heights},permute(anemology{1},[2 1 3]));
n_wind = griddedInterpolant({longitudes,latitudes,heights},permute(anemology{2},[2 1 3]));
u_wind = griddedInterpolant({longitudes,latitudes,heights},permute(anemology{3},[2 1 3]));

% Barometry
load('barometry.mat')

% Topography
load('topography_4641m.mat')

%%% Parameters

% Time Step
dt = 1; % Time step [s]

% Time
time = 0:dt:1*60*60; % Simulation Duration

% Accelerometer
bias_static_accel = 1e-4; % Static bias of acceleration [mg]
bias_dynamic_accel = 1e-5; % Dynamic bias of acceleration [mg]
random_noise_accel = 1e-6; % Random noise of acceleration [m/sec/sqrt(hr)]

% Gyroscope
bias_static_gyro = 5e-9; % Static bias of angular velocity [deg/hr]
bias_dynamic_gyro = 5e-10; % Dynamic bias of angular velocity [deg/hr]
random_noise_gyro = 5e-11; % Random noise of angular velocity [deg/sqrt(hr)]

% Measurement Noise
sigma_topo = 1; % Measurement noise of height measure [m]
sigma_baro = 10; % Measurement noise of pressure measure [Pa]
sigma_gps_pos = 100; % Measurement noise of GPS position [m]
sigma_gps_vel = 1; % Measurement noise of GPS velocity measure [m/s]

% Particle Filter
N = 200; % Number of Particles
gap = 10; % Number of iterations required between resampling
k_j = 3; % Jitter Factor
sigma_add = deg2rad(1e-3); % Minimium Additional Noise [deg] 

% ES-EKF
sigma_att = deg2rad(1e-3); % Attitude [deg]
sigma_vel = 1e-3; % Velocity [m/s]
sigma_pos = 50; % Position [m]

% Complementay Filter
alpha = 0.98; %  Gain

%%% Simulation

% Time
L = round(time(end)/dt)+1;

% Preallocation (States)
ins_att = zeros(3,L);
ins_vel = zeros(3,L);
ins_pos_llh = zeros(3,L);
ins_pos_ned = zeros(3,L);
omega_b_ib = zeros(3,L);
f_ib_b = zeros(3,L);
E = zeros(9,L);
C = zeros(9,L);
J = zeros(2,2,L);

% Preallocation (Structures)
truth = struct;
est = struct;
rbpf = struct;

% Preallocation (Measurements)
wind_vel = zeros(3,L);
ext_meas = zeros(2,L);

% Coordinate Transformation
enu2ned = [0 1  0;
           1 0  0;
           0 0 -1];

% Pose
ins_pos_llh(:,1) = [deg2rad(-45+90*rand(1));deg2rad(-180+360*rand(1));50000+20000*rand(1)]; % Initial position (Latitude,Longitude,Height)
ins_pos_ned(:,1) = [0;0;0]; % Initial position (North,East,Down)
ins_vel(:,1) = enu2ned*[e_wind(ins_pos_llh(:,1)');n_wind(ins_pos_llh(:,1)');u_wind(ins_pos_llh(:,1)')]; % Initial velocity (North,East,Down)
ins_att(:,1) = dcm2eulr(vrrotvec2mat(vrrotvec([1;0;0],ins_vel(:,1)))); % Initial orientation (Roll, Pitch, Yaw)

% Accelerometer Error
b_as = normrnd(0,bias_static_accel,[3,1]).*ones(3,L);
b_ad = zeros(3,L);
for k = 2:L
    b_ad(:,k) = b_ad(:,k-1)+normrnd(0,bias_dynamic_accel*sqrt(dt),[3,1]);
end
w_a = normrnd(0,random_noise_accel,[3,L]);
error_accel = b_as+b_ad+w_a;

% Gyroscope Error
b_gs = normrnd(0,bias_static_gyro,[3,1]).*ones(3,L);
b_gd = zeros(3,L);
for k = 2:L
    b_gd(:,k) = b_gd(:,k-1)+normrnd(0,bias_dynamic_gyro*sqrt(dt),[3,1]);
end
w_g = normrnd(0,random_noise_gyro,[3,L]);
error_gyro = b_gs+b_gd+w_g;

% Initial Belief/Distribution
P0 = diag([sigma_att^2*ones(1,3) (sigma_vel)^2*ones(1,3) (sigma_pos*(1/(6051800+ins_pos_llh(3,1))))^2 (sigma_gps_pos*(1/((6051800+ins_pos_llh(3,1))*cos(ins_pos_llh(1,1)))))^2 (sigma_gps_pos)^2]);
E(:,1) = [ins_att(:,1);ins_vel(:,1);ins_pos_llh(:,1)]; % State Vector

% Initial Rao-Blackwellized Particle Filter
x = repmat([ins_att(:,1);ins_vel(:,1);ins_pos_llh(:,1)],1,N);
x(7,:) = x(7,:)+rand(1,N)*sigma_pos*(1/(6051800+ins_pos_llh(3,1)));
x(8,:) = x(8,:)+rand(1,N)*sigma_pos*(1/((6051800+ins_pos_llh(3,1))*cos(ins_pos_llh(1,1))));
w = ones(N,1)/N; % Particle Weight
P = repmat(P0,1,1,N); % Error Covariance Matrix
prev = 0;

% Measurements
altIn = ins_pos_llh(3,1)-GetTopographyData(ins_pos_llh(:,1),topography)+sigma_topo*randn(1);
baroIn = GetBarometryData(ins_pos_llh(:,1),barometry)+sigma_baro*randn(1);
ext_meas(:,1) = [altIn; baroIn];

% Iterate for [1] Truth
%             [2] INS
%             [3] RBPF
for j = [1,2,3]

    % Display Progress
    if j == 1
        disp('Truth Simulation...')
    elseif j == 2
        disp('Noisy Simulation...')
    else
        disp('RBPF Simulation...')
    end

    % Iterate for all time steps less the first
    for k = 2:L

        % If truth
        if j == 1

            % Anemology
            wind_vel(:,k) = enu2ned*[e_wind(ins_pos_llh(:,k-1)');n_wind(ins_pos_llh(:,k-1)');u_wind(ins_pos_llh(:,k-1)')];

            % Flight Model
            [omega_b_ib(:,k),f_ib_b(:,k)] = Flight_Model(ins_att(:,k-1),ins_vel(:,k-1),wind_vel(:,k),ins_pos_llh(:,k-1),dt);

            % Inertial Navigation System
            [ins_att_plus,ins_vel_plus,ins_pos_plus] = Inertial_Navigation_System(ins_att(:,k-1),ins_vel(:,k-1),ins_pos_llh(:,k-1),omega_b_ib(:,k),f_ib_b(:,k)*dt,dt);

            % Update States
            ins_att(:,k) = ins_att_plus;
            ins_vel(:,k) = ins_vel_plus;
            ins_pos_llh(:,k) = ins_pos_plus;
            ins_pos_ned(:,k) = LLH2NED(ins_pos_plus,ins_pos_llh(:,1));

            % Display Progress
            if (mod(k,0.01*(L-1))==0)
                progress = k/(L-1)*100;
                fprintf('%.0f%%\n',progress)
            end

            % If estimate
        elseif j == 2

            % Add error
            f_ib_b(:,k) = f_ib_b(:,k) + error_accel(:,k);
            omega_b_ib(:,k) = omega_b_ib(:,k) + error_gyro(:,k);

            % Inertial Navigation System
            [ins_att_plus,ins_vel_plus,ins_pos_plus] = Inertial_Navigation_System(ins_att(:,k-1),ins_vel(:,k-1),ins_pos_llh(:,k-1),omega_b_ib(:,k),f_ib_b(:,k)*dt,dt);

            % INS System Noise
            Q = diag(Q_INS(ins_att_plus,ins_pos_plus,f_ib_b(:,k),random_noise_gyro,random_noise_accel,bias_dynamic_gyro,bias_dynamic_accel,dt));

            % Update States
            ins_att(:,k) = ins_att_plus+normrnd([0 0 0]',Q(1:3));
            ins_vel(:,k) = ins_vel_plus+normrnd([0 0 0]',Q(4:6));
            ins_pos_llh(:,k) = ins_pos_plus+normrnd([0 0 0]',Q(7:9));
            ins_pos_ned(:,k) = LLH2NED(ins_pos_plus,ins_pos_llh(:,1));

            % If near gimbal lock
            if ~isreal(ins_att(:,k))

                % Add error to remaining states
                f_ib_b(:,k+1:end) = f_ib_b(:,k+1:end) + error_accel(:,k+1:end);
                omega_b_ib(:,k+1:end) = omega_b_ib(:,k+1:end) + error_gyro(:,k+1:end);

                % Update States
                ins_att(:,k:end) = nan(size(ins_att(:,k:end)));
                ins_vel(:,k:end) = nan(size(ins_vel(:,k:end)));
                ins_pos_llh(:,k:end) = nan(size(ins_pos_llh(:,k:end)));
                ins_pos_ned(:,k:end) = nan(size(ins_pos_ned(:,k:end)));

                % Break loop
                break

            end

            % Display Progress
            if (mod(k,0.01*(L-1))==0)
                progress = k/(L-1)*100;
                fprintf('%.0f%%\n',progress)
            end

            % If RBPF
        elseif j == 3

            % Inputs
            accel = f_ib_b(:,k);
            gyro = omega_b_ib(:,k);

            % States
            orient = x(1:3,:);
            vel = x(4:6,:);
            pos = x(7:9,:);

            % Measurements
            altIn = truth.pos_llh(3,k)-GetTopographyData(truth.pos_llh(:,k),topography)+sigma_topo*randn(1);
            baroIn = GetBarometryData(truth.pos_llh(:,k),barometry)+sigma_baro*randn(1);
            ext_meas(:,k) = [altIn; baroIn];

            % Measurement Noise
            sigma_baro_now = (GetBarometryHeight(baroIn+sigma_baro,barometry)-GetBarometryHeight(baroIn-sigma_baro,barometry))/(2*sigma_baro);

            % Iterate for each particle
            for l = 1:N

                %%% Inertial Navigation

                % Prediction Step
                [ins_att_plus,ins_vel_plus,ins_pos_plus] = Inertial_Navigation_System(orient(:,l),vel(:,l),pos(:,l),gyro,accel*dt,dt);

                % INS System Noise
                Q = diag(Q_INS(ins_att_plus,ins_pos_plus,accel,random_noise_gyro,random_noise_accel,bias_dynamic_gyro,bias_dynamic_accel,dt));

                % Update States
                ins_att_plus = ins_att_plus+normrnd([0 0 0]',Q(1:3));
                ins_vel_plus = ins_vel_plus+normrnd([0 0 0]',Q(4:6));
                ins_pos_plus = ins_pos_plus+normrnd([0 0 0]',Q(7:9));

                %%% Complementary Filter
                ins_att_plus = [alpha*ins_att_plus(1)+(1-alpha)*atan(accel(2)/accel(3));
                                alpha*ins_att_plus(2)+(1-alpha)*atan(accel(1)/accel(3));
                                ins_att_plus(3)];

                %%% Error State Extended Kalman Filter

                % Residual of measurements
                delta_z = GetBarometryHeight(baroIn,barometry)-ins_pos_plus(3);
                
                % Measurement Matrix
                H = [0 0 0 0 0 0 0 0 -1];

                % Measurement Noise
                R = sigma_baro_now^2;

                % Error State Extended Kalman Filter
                [x_new,P_new] = EKF(ins_att_plus,ins_vel_plus,ins_pos_plus,accel,Q,dt,P(:,:,l),R,delta_z,H);
                x_new = [x_new(1:6);ins_pos_plus(1:2);x_new(9)];

                % Update State
                x(:,l) = x_new;
                P(:,:,l) = P_new;

            end

            %%% Particle Filter

            % Actual measurement
            yNow = repmat(GetBarometryHeight(ext_meas(2,k),barometry)-ext_meas(1,k),1,N);

            % Expected measurement of each particle
            yhat = GetTopographyData(x(7:9,:),topography);

            % Likelihood of each particle
            q = normpdf(yhat,yNow,sqrt((k_j*sigma_topo)^2+sigma_baro^2))';

            % Remove invalid weights and states
            index = find(isnan(q));
            if ~isempty(index)
                q(index) = 0;
                x(:,index) = zeros(9,length(index));
            end

            % Multiply previous weight
            q = q.*w;

            % Effective particle count
            if sum(q) == 0
                break
            end

            % Normalize the weights
            q = q./sum(q);

            % Copy the weights
            w = q;

            % Mean
            E(:,k) = sum(x.*reshape(w,[1,N]),2);

            % Covariance
            C(:,k) = sum((x-E(:,k)).^2.*reshape(w,[1,N]),2); 

            % Figure 1: Particle Visualization
           if 0 == 1 %(rem(k-1,10*60/dt)) == 0

                figure(1)
                hold on
                grid on

                % Plots
                a = plot3(rad2deg(truth.pos_llh(1,k-1)),rad2deg(truth.pos_llh(2,k-1)),1,'bx','MarkerSize',20,'LineWidth',6);
                b = plot3(rad2deg(E(7,k-1)),rad2deg(E(8,k-1)),1,'gx','MarkerSize',20,'LineWidth',6);
                c = plot3(rad2deg(x(7,:)),rad2deg(x(8,:)),w'/max(w),'ko');

                % Topography Data
                info = axis;
                lat = linspace(info(1),info(2),1000);
                lon = linspace(info(3),info(4),1000);
                topo = zeros(1000);

                for n=1:1000
                    for o=1:1000
                        topo(n,o) = GetTopographyData([deg2rad(lat(n));deg2rad(lon(o))],topography);
                    end
                end

                % Topography Contour
                contourf(lat,lon,topo');

                % Layer Order
                uistack(c,'top')
                uistack(b,'top')
                uistack(a,'top')

                % Labels
                xlabel('Latitude [deg]')
                ylabel('Longitude [deg]')
                key = legend({'','Particles','Estimate','Truth'},Location='southoutside');
                key.NumColumns = 3;
                colorbar;
                pause()

                % Clear
                clf(1)

            end

            % If effective particle count below threshold and
            % If enough time has elapsed since last resampling
            if 1/sum(w.^2) <= 2/3*N && k-prev >= gap

                % Resample
                [x,w,P] = MSVResampling(x,w,P);

                % Add roughening
                x(7,:) = x(7,:)+normrnd(0,sigma_add,[1,N]);
                x(8,:) = x(8,:)+normrnd(0,sigma_add,[1,N]);

                % Save time step
                prev = k;

            end

            % Display Progress
            progress = (k-1)/(60/dt);
            fprintf('%.3f min\n',progress)

        end
    end

    % If truth
    if j == 1

        % Store data
        truth.att = ins_att;
        truth.vel = ins_vel;
        truth.pos_llh = ins_pos_llh;
        truth.pos_ned = ins_pos_ned;
        truth.bias_f_ib_b = error_accel;
        truth.bias_omega_b_ib = error_gyro;
        truth.f_ib_b = f_ib_b;
        truth.omega_b_ib = omega_b_ib;
        truth.J = J;
        trial.truth = truth;

        % Clear data
        ins_att(:,2:end) = zeros(size(ins_att(:,2:end)));
        ins_vel(:,2:end) = zeros(size(ins_vel(:,2:end)));
        ins_pos_ned(:,2:end) = zeros(size(ins_pos_ned(:,2:end)));
        ins_pos_llh(:,2:end) = zeros(size(ins_pos_llh(:,2:end)));

        % If estimate
    elseif j == 2

        % Store Data
        est.att = ins_att;
        est.vel = ins_vel;
        est.pos_llh = ins_pos_llh;
        est.pos_ned = ins_pos_ned;
        trial.est = est;

        % If RBPF
    elseif j == 3

        % Store Data
        rbpf.att = E(1:3,:);
        rbpf.vel = E(4:6,:);
        rbpf.pos_llh = E(7:9,:);
        rbpf.pos_ned = LLH2NED(E(7:9,:),truth.pos_llh(:,1));
        rbpf.ext_meas = ext_meas;
        trial.rbpf = rbpf;

    end
end

% Stop timer
duration = toc;
disp(duration)

% Save data
save('Test_1.mat', 'trial', '-v7.3')

% Shutdown Cluster
delete(gcp('nocreate'))

%% Plots

% Figure 1: Trajectory (NED Coordinates)
figure(1)
hold on
grid on

% Plots
plot3(rad2deg(trial.truth.pos_llh(1,1:k-1)),rad2deg(trial.truth.pos_llh(2,1:k-1)),trial.truth.pos_llh(3,1:k-1)./1000,'k')
plot3(rad2deg(trial.est.pos_llh(1,1:k-1)),rad2deg(trial.est.pos_llh(2,1:k-1)),trial.est.pos_llh(3,1:k-1)./1000,'r')
plot3(rad2deg(trial.rbpf.pos_llh(1,1:k-1)),rad2deg(trial.rbpf.pos_llh(2,1:k-1)),trial.rbpf.pos_llh(3,1:k-1)./1000,'bo')
view(3)

% Topography Data
info = axis;
lat = linspace(info(1),info(2),1000);
lon = linspace(info(3),info(4),1000);
h = info(5);
topo = zeros(1000);

for n=1:1000
    for o=1:1000
        topo(n,o) = GetTopographyData([deg2rad(lat(n));deg2rad(lon(o));1000*h],topography);
    end
end

% Topography Contour
[~,p] = contourf(lat,lon,topo);
p.ContourZLevel = h;

% Labels
xlabel('Latitude [deg]')
ylabel('Longitude [deg]')
zlabel('Height [km]')
key = legend({'Truth','INS','RBPF'},Location='southoutside');
key.NumColumns = 3;
c = colorbar;

%% Figure 2: RMSE (Attitude)
figure(2)
tiledlayout(3,1)

% Northern Direction
nexttile
hold on
grid on

% Plot
plot(time(1:1:k-1)/3600,rad2deg(rmse(trial.est.att(1,1:k-1),trial.truth.att(1,1:k-1),1)),'k')
plot(time(1:1:k-1)/3600,rad2deg(rmse(trial.rbpf.att(1,1:k-1),trial.truth.att(1,1:k-1),1)),'b')

% Labels
xlabel('Time [hr]')
ylabel('Attitude RMSE [deg]')
title('Roll')
legend({'INS','RBPF'},Location='best');
xlim([0 time(end)/3600])

% Eastern Direction
nexttile
hold on
grid on

% Plot
plot(time(1:1:k-1)/3600,rad2deg(rmse(trial.est.att(2,1:k-1),trial.truth.att(2,1:k-1),1)),'k')
plot(time(1:1:k-1)/3600,rad2deg(rmse(trial.rbpf.att(2,1:k-1),trial.truth.att(2,1:k-1),1)),'b')

% Labels
xlabel('Time [hr]')
ylabel('Attitude RMSE [deg]')
title('Pitch')
legend({'INS','RBPF'},Location='best');
xlim([0 time(end)/3600])

% Downward Direction
nexttile
hold on
grid on

% Plot
plot(time(1:1:k-1)/3600,rad2deg(rmse(trial.est.att(3,1:k-1),trial.truth.att(3,1:k-1),1)),'k')
plot(time(1:1:k-1)/3600,rad2deg(rmse(trial.rbpf.att(3,1:k-1),trial.truth.att(3,1:k-1),1)),'b')

% Labels
xlabel('Time [hr]')
ylabel('Attitude RMSE [deg]')
title('Yaw')
legend({'INS','RBPF'},Location='best');
xlim([0 time(end)/3600])

%% Figure 3: RMSE (Velocity)
figure(3)
tiledlayout(3,1)

% Northern Direction
nexttile
hold on
grid on

% Plot
plot(time(1:1:k-1)/3600,rmse(trial.est.vel(1,1:k-1),trial.truth.vel(1,1:k-1),1),'k')
plot(time(1:1:k-1)/3600,rmse(trial.rbpf.vel(1,1:k-1),trial.truth.vel(1,1:k-1),1),'b')

% Labels
xlabel('Time [hr]')
ylabel('Velocity RMSE [m/s]')
title('Northern Direction')
legend({'INS','RBPF'},Location='best');
xlim([0 time(end)/3600])

% Eastern Direction
nexttile
hold on
grid on

% Plot
plot(time(1:1:k-1)/3600,rmse(trial.est.vel(2,1:k-1),trial.truth.vel(2,1:k-1),1),'k')
plot(time(1:1:k-1)/3600,rmse(trial.rbpf.vel(2,1:k-1),trial.truth.vel(2,1:k-1),1),'b')

% Labels
xlabel('Time [hr]')
ylabel('Velocity RMSE [m/s]')
title('Eastern Direction')
legend({'INS','RBPF'},Location='best');
xlim([0 time(end)/3600])

% Downward Direction
nexttile
hold on
grid on

% Plot
plot(time(1:1:k-1)/3600,rmse(trial.est.vel(3,1:k-1),trial.truth.vel(3,1:k-1),1),'k')
plot(time(1:1:k-1)/3600,rmse(trial.rbpf.vel(3,1:k-1),trial.truth.vel(3,1:k-1),1),'b')

% Labels
xlabel('Time [hr]')
ylabel('Velocity RMSE [m/s]')
title('Downward Direction')
legend({'INS','RBPF'},Location='best');
xlim([0 time(end)/3600])

%% Figure 4: RMSE (Position LLH)
figure(4)
tiledlayout(3,1)

% Latitude Direction
nexttile
hold on
grid on

% Plot
plot(time(1:1:k-1)/3600,rad2deg(rmse(trial.est.pos_llh(1,1:k-1),trial.truth.pos_llh(1,1:k-1),1)),'k')
plot(time(1:1:k-1)/3600,rad2deg(rmse(trial.rbpf.pos_llh(1,1:k-1),trial.truth.pos_llh(1,1:k-1),1)),'b')

% Labels
xlabel('Time [hr]')
ylabel('RMSE Position [°]')
title('Latitude')
legend({'INS','RBPF'},Location='best');
xlim([0 time(end)/3600])

% Longitude Direction
nexttile
hold on
grid on

% Plot
plot(time(1:1:k-1)/3600,rad2deg(rmse(trial.est.pos_llh(2,1:k-1),trial.truth.pos_llh(2,1:k-1),1)),'k')
plot(time(1:1:k-1)/3600,rad2deg(rmse(trial.rbpf.pos_llh(2,1:k-1),trial.truth.pos_llh(2,1:k-1),1)),'b')

% Labels
xlabel('Time [hr]')
ylabel('RMSE Position [°]')
title('Longitude')
legend({'INS','RBPF'},Location='best');
xlim([0 time(end)/3600])

% Height Direction
nexttile
hold on
grid on

% Plot
plot(time(1:1:k-1)/3600,rmse(trial.est.pos_llh(3,1:k-1),trial.truth.pos_llh(3,1:k-1),1),'k')
plot(time(1:1:k-1)/3600,rmse(trial.rbpf.pos_llh(3,1:k-1),trial.truth.pos_llh(3,1:k-1),1),'b')

% Labels
xlabel('Time [hr]')
ylabel('RMSE Position [m]')
title('Height')
legend({'INS','RBPF'},Location='best');
xlim([0 time(end)/3600])

%% Figure 5: RMSE (Position NED)
figure(5)
tiledlayout(3,1)

% Northern Direction
nexttile
hold on
grid on

% Plot
plot(time(1:1:k-1)/3600,rmse(trial.est.pos_ned(1,1:k-1),trial.truth.pos_ned(1,1:k-1),1)/1000,'k')
plot(time(1:1:k-1)/3600,rmse(trial.rbpf.pos_ned(1,1:k-1),trial.truth.pos_ned(1,1:k-1),1)/1000,'b')

% Labels
xlabel('Time [hr]')
ylabel('RMSE Position [km]')
title('North')
legend({'INS','RBPF'},Location='best');
xlim([0 time(end)/3600])

% Eastern Direction
nexttile
hold on
grid on

% Plot
plot(time(1:1:k-1)/3600,rmse(trial.est.pos_ned(2,1:k-1),trial.truth.pos_ned(2,1:k-1),1)/1000,'k')
plot(time(1:1:k-1)/3600,rmse(trial.rbpf.pos_ned(2,1:k-1),trial.truth.pos_ned(2,1:k-1),1)/1000,'b')

% Labels
xlabel('Time [hr]')
ylabel('RMSE Position [km]')
title('East')
legend({'INS','RBPF'},Location='best');
xlim([0 time(end)/3600])

% Downward Direction
nexttile
hold on
grid on

% Plot
plot(time(1:1:k-1)/3600,rmse(trial.est.pos_ned(3,1:k-1),trial.truth.pos_ned(3,1:k-1),1)/1000,'k')
plot(time(1:1:k-1)/3600,rmse(trial.rbpf.pos_ned(3,1:k-1),trial.truth.pos_ned(3,1:k-1),1)/1000,'b')

% Labels
xlabel('Time [hr]')
ylabel('RMSE Position [km]')
title('Down')
legend({'INS','RBPF'},Location='best');
xlim([0 time(end)/3600])
