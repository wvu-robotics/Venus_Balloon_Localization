% Development Information
% WVU Interactive Robotics Laboratory
% Venus Localization Simulator
% Simulation_RBPF.m
%
% Purpose: Balloon Localization Simulation using RBPF
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

%% Clean

% Clean
clear
close
clc

% Display Progress
disp('Simulating...')

% Check if there is an existing parallel pool
currentPool = gcp('nocreate'); % Get the current pool without creating a new one

% If there is an existing pool, delete its
if ~isempty(currentPool)
    delete(currentPool);
end

% Start a new parallel pool for thread-based parallelism
myCluster = parcluster('local');
myCluster.NumWorkers = 40;

% Iterate for all cases
for i = 1:4

    % Start Timer
    tic

    %%% Parameters

    % Options (MAKE CHANGES HERE!!!)
    opt = [i;  % Duration [1 hr, 2 hr, 3 hr, 4 hr]
           4;  % Positions [1-40, 2-80, 3-120, 4-160]
           1;  % Height [1-50km, 2-60km, 3-70km]
           2;  % Particle Count [1-400, 2-200, 3-100]
           2;  % Map Quality [1-Moon (412m), 2-Moon (2063m), 3-Moon (4123m)]
           2;  % Accelerometer Quality [1-Space Grade, 2-Navigation Grade, 3-Tactical Grade]
           2;  % Gyroscope Quality [1-Space Grade, 2-Navigation Grade, 3-Tactical Grade]
           1;  % Altimeter Quality [1-1, 2-5, 3-10]
           1]; % Barometer Quality [1-10, 2-50, 3-100]

    % Conversion
    val = [1 2 3 4;  % Duration [1 hr, 2 hr, 3 hr, 4 hr]
           40 80 120 160;  % Positions [1-40, 2-80, 3-120, 4-160]
           50000 60000 70000 NaN;  % Height [1-50km, 2-60km, 3-70km]
           400 200 100 NaN;  % Particle Count [1-400, 2-200, 3-100]
           1 2 3 NaN;  % Map Quality [1-Moon (412m), 2-Moon (2063m), 3-Moon (4123m)]
           1e-4 1e-3 1e-2 NaN;  % Accelerometer Quality [1-Space Grade, 2-Navigation Grade, 3-Tactical Grade]
           5e-9 5e-8 5e-7 NaN;% Gyroscope Quality [1-Space Grade, 2-Navigation Grade, 3-Tactical Grade]
           1 5 10 NaN;  % Altimeter Quality [1-0.1, 2-1, 3-10, 4-100]
           10 50 100 NaN]; % Barometer Quality [1-1, 2-10, 3-100, 4-1000]

    %%% Initialization

    % Time Step
    dt = 1; % Time step [s]

    % Time
    time = 0:dt:val(1,opt(1))*60*60; % Simulation Duration
    L = round(time(end)/dt)+1; % Number of Steps

    % Positions
    pos = [0.0399150568104372	-0.0202919730618894	-0.210792870123913	-0.0970668866136607	-0.267046300385983	-0.403803488725766	-0.211075428492535	-0.173843325129569	-0.0862097869742737	-0.0460880224064992	0.0668936171435906	0.173771441180391	-0.141562715397140	0.0213002542685484	-0.203945855833341	-0.0265201604677678	-0.218533050543210	-0.380429052749904	-0.0595573103802422	-0.335347146057862	0.0504734523533892	0.270221453151011	-0.236968026851187	-0.258889028269309	0.205180261540010	0.125396976016864	0.106500129217020	-0.201253180620553	-0.00936475901432366	0.0434531178112515	0.143531027499235	0.121421378782942	-0.232814777327180	-0.383548802907519	0.122016312912420	-0.249870034536997	-0.0687700101997445	0.0346531367960040	0.0879951649500586	0.311191796737168	-0.193663220591077	-0.115551274123640	0.401072615960494	0.0943824956689235	-0.0688338760591157	-0.199492912197206	-0.140678608576581	-0.141883421013487	0.294331814075306	-0.0540839595387212	-0.214975460970055	-0.230110608805457	-0.377594591738847	-0.230031882226150	0.363828982885356	-0.243102653145424	0.0134688695358414	-0.245677711727343	-0.192298161830623	-0.00560069468234015	-0.293840311011867	-0.231948055806327	-0.000856014877387898	-0.0440736565055796	-0.0583432228370823	0.274218186597889	-0.0846124642465125	-0.107129987567016	0.210468584187595	-0.0231589971879382	-0.210592718158187	-0.386117277902827	-0.208979680429053	0.228998952490524	-0.195820566135917	-0.213724705233845	-0.140148661802487	0.120834229539463	0.200356288101762	-0.159212133076228	0.0923254320417501	0.0622539866196100	-0.372149109211464	0.00747638447291871	0.0899395613685132	-0.246469221515713	-0.175225982351795	-0.178217395541278	-0.285828102656678	0.112822063211447	-0.460088670940512	-0.242505154953395	-0.229157313613550	-0.242407537988077	-0.207394477300569	0.143498356974095	0.0375326462749329	-0.346315018098448	-0.212847426156784	-0.279963240527713	-0.0662652032609503	0.104681993578800	0.428630720511194	0.441239114346543	-0.257478561025645	-0.143698252884903	-0.413422248962255	-0.109187064433446	-0.300561294088335	-0.178271428120354	0.540073976863164	-0.0902140780197867	0.0840714327519362	0.299830660076538	0.267671654652502	-0.215216823741159	-0.247280914622199	0.00584394077323917	0.211684267299508	-0.198744735951064	0.322245778634361	-0.145450239315594	-0.0318202945457668	-0.217396715884762	-0.236643590631038	-0.148651193803544	-0.492985531080519	-0.0464568518837887	-0.262099940281311	-0.170234548044510	0.0996017657679157	0.0927088008734695	-0.328322353384460	-0.278586421675698	0.135042452660806	0.250099331969122	-0.0712100127941262	-0.315517544579064	-0.144963943597297	0.0388116290714901	0.274227416871575	0.426365379387987	-0.257510753003749	0.0877919000521371	0.216335430540279	-0.149799633527441	-0.0847475680168927	0.155350926850750	-0.414275827135582	-0.494954112042139	-0.0815880044707175	-0.280350610750996	-0.364472346375736	-0.201409992462780	0.230434923054382	-0.247279469681558	-0.421633276898733	-0.264818377340270	0.100573473936801	-0.227165718793330;
           0.546323625491525	0.614286403582561	0.771649379959598	0.201254467624621	0.433392019289355	-0.123090791099029	0.750529662228486	0.167677228560353	0.542886528296308	0.0221026486340373	0.625961022248051	0.551780241926534	0.0206093044516519	0.752280003401233	0.762403664894040	0.411766579535368	0.330602796587010	-0.245951778222699	0.380023212392870	0.531243213875930	0.144289398970904	0.508776068573200	-0.489136524201639	0.628854510351820	0.674714169532243	0.769329810231975	0.686002557957511	-0.00109747150062080	-0.0877980860473480	0.404630072455479	0.411574592253645	0.626076515150662	0.492083218853671	-0.0827877330046515	0.672993898310744	-0.470565131168743	0.535629340635040	0.242587009155436	0.616200205748340	0.599753521796865	0.159286170199256	-0.536540942264853	-0.0509621841844378	0.670851308177436	0.212294286242668	0.710990874843620	0.630858262396985	0.650468972356205	0.722922136287164	0.112386056101994	0.244011043901009	-0.746960552999283	-0.0509425919277141	0.334014895192237	0.0301888854963379	0.703351330573382	0.751021864926081	-0.529925202733817	0.159881557376335	-0.110946975443286	-0.633510612173514	0.606522327292235	0.204423280728147	0.00111611159485255	0.357887736512422	0.504219709709663	-0.0276616202634865	0.661943589470180	0.266845268099302	0.256466587049449	0.667248902086514	-0.0884983215383522	0.545155654463400	0.375806702454227	0.148463822564686	0.441796888395155	0.474960192387849	0.0857130146112807	0.618582044408592	-0.479867583651746	0.689655760072929	0.529741514000387	0.310992007467753	0.221775131158509	0.0890660140160368	-0.698745832010015	-0.706706824668480	0.0217742900188230	0.716161393134980	0.408304903467827	0.493934157427328	0.501479145413483	0.352344080795076	-0.696249071377630	-0.736086999139966	0.441685477958440	-0.105620963761859	0.407266880007918	0.139922028273502	0.483048440746617	0.184437663071164	0.641676566373546	0.00240233275575774	0.0346541354133876	0.610754125415658	0.505715473157314	-0.276335273166965	0.526896403300698	0.461718049670816	0.254756392515522	0.776892559413943	-0.0249532945499079	-0.0369213624561406	0.134321259431372	0.319023675190001	0.782821736441712	0.473163000620465	0.584311880173190	0.0490516604028058	0.368966676492802	0.197849351568528	0.0419157246006050	0.346706371238441	0.741673275864102	-0.511045443648949	-0.586277864191532	-0.478427865227339	0.261620301186650	-0.506116092051768	-0.672269269022940	0.693681097921767	0.287776256976266	-0.0981283740325795	0.417337307071149	0.595248865784020	0.529603881719460	0.578978467836628	0.511398523719792	0.0517162206397352	-0.0365037678132461	0.511929851980167	0.671668102090229	-0.532180977587899	-0.503595871543565	0.0528434226802252	-0.0810402599811712	-0.553582224420743	0.360308999267313	-0.577035619667412	0.491847084070877	0.432593543911962	-0.666642827472223	-0.467044042189538	-0.502187768117986	-0.498989597619867	0.442822518778699	-0.558729690348311	0.738157577878879	0.500708008446812	-0.128122428068645;
           val(3,opt(3))*ones(1,160)];
    pos = pos(:,1:val(2,opt(2)));

    % Preallocation
    trials = cell(1,length(pos));

    % PF
    N = val(4,opt(4)); % Number of Particles
    sigma_add = [5e-4 7.5e-4 5e-3]; % Additional Process Noise [deg]
    sigma_add = deg2rad(sigma_add(i));

    % ES-EKF
    sigma_att = deg2rad(1e-3); % Attitude [deg]
    sigma_vel = 1e-3; % Velocity [m/s]
    sigma_pos = 5*10^opt(6); % Position [m]

    % Accelerometer
    bias_static_accel = val(6,opt(6)); % Static bias of acceleration [mg]
    bias_dynamic_accel = val(6,opt(6))/10; % Dynamic bias of acceleration [mg]
    random_noise_accel = val(6,opt(6))/100; % Random noise of acceleration [m/sec/sqrt(hr)]

    % Gyroscope
    bias_static_gyro = val(7,opt(7)); % Static bias of angular velocity [deg/hr]
    bias_dynamic_gyro = val(7,opt(7))/10; % Dynamic bias of angular velocity [deg/hr]
    random_noise_gyro = val(7,opt(7))/100; % Random noise of angular velocity [deg/sqrt(hr)]

    % Measurement Noise
    sigma_topo = val(8,opt(8)); % Measurement noise of height measure [m]
    sigma_baro = val(9,opt(9)); % Measurement noise of pressure measure [Pa]

    % Anemology
    load('anemology.mat','anemology','longitudes','latitudes','heights')
    e_wind = griddedInterpolant({longitudes,latitudes,heights},permute(anemology{1},[2 1 3]));
    n_wind = griddedInterpolant({longitudes,latitudes,heights},permute(anemology{2},[2 1 3]));
    u_wind = griddedInterpolant({longitudes,latitudes,heights},permute(anemology{3},[2 1 3]));

    % Barometry
    load('barometry.mat')

    % Topography
    if opt(5) == 1
        load('topography_412m.mat')
    elseif opt(5) == 2
        load('topography_2063m.mat')
    elseif opt(5) == 3
        % load('topography_4641m.mat')
        load('topography_4123m.mat')
    end

    %%% Errors

    % Seed
    rng(0)

    % Preallocation
    error_accel = zeros(3,L,length(trials));
    error_gyro = zeros(3,L,length(trials));
    error_topo = zeros(1,L,length(trials));
    error_baro = zeros(1,L,length(trials));

    for j = 1:length(trials)

        % Accelerometer Error
        b_as = normrnd(0,bias_static_accel,[3,1]).*ones(3,L);
        b_ad = zeros(3,L);
        for k = 2:L
            b_ad(:,k) = b_ad(:,k-1)+normrnd(0,bias_dynamic_accel*sqrt(dt),[3,1]);
        end
        w_a = normrnd(0,random_noise_accel,[3,L]);
        error_accel(:,:,j) = b_as+b_ad+w_a;

        % Gyroscope Error
        b_gs = normrnd(0,bias_static_gyro,[3,1]).*ones(3,L);
        b_gd = zeros(3,L);
        for k = 2:L
            b_gd(:,k) = b_gd(:,k-1)+normrnd(0,bias_dynamic_gyro*sqrt(dt),[3,1]);
        end
        w_g = normrnd(0,random_noise_gyro,[3,L]);
        error_gyro(:,:,j) = b_gs+b_gd+w_g;

        % Measurement Error
        error_topo(:,:,j) = normrnd(0,sigma_topo,[1,L]);
        error_baro(:,:,j) = normrnd(0,sigma_baro,[1,L]);

    end

    %%% Simulation

    % Display current progress
    parfor n = 1:length(trials)
        trials{n} = Venus_Simulation(dt,time,pos(:,n),N,sigma_add,error_accel(:,:,n),error_gyro(:,:,n),error_topo(:,:,n),error_baro(:,:,n),sigma_topo,sigma_baro,sigma_att,sigma_vel,sigma_pos,random_noise_gyro,random_noise_accel,bias_dynamic_gyro,bias_dynamic_accel,e_wind,n_wind,u_wind,topography,barometry);
    end

    % Stop timer
    duration = toc;
    fprintf('Case #%.0f: %.1f min\n',i,duration/60)

    %%% Save

    % Save data
    if i == 1
        save('Test_1.mat', 'trials', '-v7.3')
    elseif i == 2
        save('Test_2.mat', 'trials', '-v7.3')
    elseif i == 3
        save('Test_3.mat', 'trials', '-v7.3')
    elseif i == 4
        save('Test_4.mat', 'trials', '-v7.3')
    end
end

% Shutdown Cluster
delete(gcp('nocreate'))

%% Function

function [trial] = Venus_Simulation(dt,time,pos,N,sigma_add,error_accel,error_gyro,error_topo,error_baro,sigma_topo,sigma_baro,sigma_att,sigma_vel,sigma_pos,random_noise_gyro,random_noise_accel,bias_dynamic_gyro,bias_dynamic_accel,e_wind,n_wind,u_wind,topography,barometry)
    
    %%% Parameters
    
    % Particle Filter
    k_j = 3; % Jitter Factor
    gap = 10; % Number of iterations required between resampling
    
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
    enu2ned = [0 1 0;
               1 0 0;
               0 0 -1];
    
    % Pose
    ins_pos_llh(:,1) = pos; % Initial position (Latitude,Longitude,Height)
    ins_pos_ned(:,1) = [0;0;0]; % Initial position (North,East,Down)
    ins_vel(:,1) = enu2ned*[e_wind(ins_pos_llh(:,1)');n_wind(ins_pos_llh(:,1)');u_wind(ins_pos_llh(:,1)')]; % Initial velocity (North,East,Down)
    ins_att(:,1) = dcm2eulr(vrrotvec2mat(vrrotvec([1;0;0],ins_vel(:,1)))); % Initial orientation (Roll, Pitch, Yaw)
    
    % Initial Belief/Distribution
    P0 = diag([(sigma_att)^2*ones(1,3) (sigma_vel)^2*ones(1,3) (sigma_pos*(1/(6051800+ins_pos_llh(3,1))))^2 (sigma_pos*(1/((6051800+ins_pos_llh(3,1))*cos(ins_pos_llh(1,1)))))^2 (sigma_pos)^2]);
    E(:,1) = [ins_att(:,1);ins_vel(:,1);ins_pos_llh(:,1)]; % State Vector
    
    % Initial Rao-Blackwellized Particle Filter
    x = repmat([ins_att(:,1);ins_vel(:,1);ins_pos_llh(:,1)],1,N);
    x(7,:) = x(7,:)+rand(1,N)*sigma_pos*(1/(6051800+ins_pos_llh(3,1)));
    x(8,:) = x(8,:)+rand(1,N)*sigma_pos*(1/((6051800+ins_pos_llh(3,1))*cos(ins_pos_llh(1,1))));
    w = ones(N,1)/N; % Particle Weight
    P = repmat(P0,1,1,N); % Error Covariance Matrix
    prev = 0;
    
    % Measurements
    altIn = ins_pos_llh(3,1)-GetTopographyData(ins_pos_llh(:,1),topography)+error_topo(1);
    baroIn = GetBarometryData(ins_pos_llh(:,1),barometry)+error_baro(1);
    ext_meas(:,1) = [altIn; baroIn];
    
    % Iterate for [1] Truth
    %             [2] INS
    %             [3] RBPF
    for j = [1,2,3]

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
                altIn = truth.pos_llh(3,k)-GetTopographyData(truth.pos_llh(:,k),topography)+error_topo(k);
                baroIn = GetBarometryData(truth.pos_llh(:,k),barometry)+error_baro(k);
                ext_meas(:,k) = [altIn; baroIn];

                % Measurement Noise
                sigma_baro_now = (GetBarometryHeight(baroIn+sigma_baro,barometry)-GetBarometryHeight(baroIn-sigma_baro,barometry))/(-2*sigma_baro);

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
                    ins_att_plus = [alpha*ins_att_plus(1)+(1-alpha)*atan2(-accel(2),-accel(3));
                                    alpha*ins_att_plus(2)+(1-alpha)*atan(-accel(1)/sqrt(accel(2)^2+accel(3)^2));
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
                q = normpdf(yhat,yNow,k_j*sqrt(sigma_topo^2+sigma_baro^2))';
    
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
            rbpf.E = E;
            rbpf.C = C;
            trial.rbpf = rbpf;
    
        end
    end
end
