%% graph based slam
% Description rule:
% It is easy to forget, so describe it
% (1) Underscore for two or more subscripts
% (2) However, Aeq for cplex is an exception.
% (3) Separate matrix elements with',' as much as possible
% (4) When the space after',' is appropriately long
% (5) The matrix size of the comment is (m, n)

% When I changed the range and moved it, it didn't work, so I need to fix it.
% 2018/11/18
clear all
close all
clc

%% options
opt_anime = 1;  % 1 for plotting

%% Simulation parameters
Ts = 1;   % sampling time[s]
T_gslam = 10;   % graph slam sampling

global MAX_ITR
MAX_ITR = 20;

% Landmarks as x y yaw
RFID = load('landmarks.txt')
    
%% Noise distribution setting
randn('state',1);% Fixed random number generation
Qsim = diag([0.2,1.0*2*pi/360])^2;% Observation noise
Rsim = diag([0.1,10*2*pi/360])^2;% Input noise

%% Covariance parameter of Graph based SLAM
C_sigma1 = 0.1;
C_sigma2 = 0.1;
C_sigma3 = 1.0*2*pi/360;

global sigma
sigma = diag([C_sigma1^2, C_sigma2^2, C_sigma3^2]);
%% 1-step behavior calculation
% x = [x coordinate, y coordinate, yaw]'
% u = [translational speed, turning speed]'
% Probabilistic robotics speed control model (excluding ?)
% calc_motion
EYESIGHT = 30.0;    %Robot visibility
global nx
nx = 3;
nu = 2;
path=load('truth.txt');
est=load('est.txt');
est=est.';
path=path.';
T_fin = size(path,2)-1;
x0 = zeros(nx,1);
u0 = [1; 0.1];  

x = x0;
x_true = path(:,1);    % True orbit
x_dr = est(:,1);      % dead reckoning (estimated by model only)
u = u0;
% Initial observation
z = [];
for j = 1:size(RFID,1) % Do as many as the number of landmarks
    % Calculate the distance to the landmark (robot coordinate system)
    dx_lm = RFID(j,1) - x_true(1);
    dy_lm = RFID(j,2) - x_true(2);
    d_RFID = sqrt(dx_lm^2 + dy_lm^2);
    phi = pi2pi(atan2(dy_lm,dx_lm)); % World coordinate system value (basically not used)
    angle = phi - x_true(3);

    if d_RFID <= EYESIGHT
        % Observe & save when in sight
        d_RFIDn = d_RFID;% + Qsim(1,1)*randn;
        anglen = angle;% + Qsim(2,2)*randn;
        z = [z;d_RFIDn, anglen, phi, j];
    end      
end

%% Data Warehouse (history)
hx_true = x_true;
hx_dr = x_dr;
hz = {z};

%% Main loop
for t = 0:Ts:T_fin - Ts
    % Behavior calculation (changed handling of input noise)
    t_next = t + Ts;
    u_true = u;% + Rsim*randn(nu,1); % Input noise added
    x_true=path(:,t+2);
    %x_true = calc_motion(x_true, u, Ts);   % Orbit simulation
    
    % dead reckoning
    x_dr = est(:,t+2);    % Close to state prediction
    
    % Landmark Search
%     z = zeros(1,4); % ?????? []???????H
    z = [];
    for j = 1:size(RFID,1)  % Do as many as the number of landmarks
        % Calculate the distance to the landmark (robot coordinate
        dx_lm = RFID(j,1) - x_true(1);
        dy_lm = RFID(j,2) - x_true(2);
        d_RFID = sqrt(dx_lm^2 + dy_lm^2);
        phi = pi2pi(atan2(dy_lm,dx_lm)); % World coordinate system value (basically not used)
        angle = phi - x_true(3);
        
        if d_RFID <= EYESIGHT
            % Observe & save when in sight
            d_RFIDn = d_RFID;% + Qsim(1,1)*randn;
            anglen = angle;% + Qsim(2,2)*randn;
            z = [z;d_RFIDn, anglen, phi, j];
        end      
    end
    
    % Data storage
    hx_true = [hx_true, x_true];
    hx_dr = [hx_dr, x_dr]; %array with estimate positions
    hz = [hz z]; %array with state vectors
    
    % Graph slam calculation for each specific cycle
    if mod(t_next, T_gslam) == 0
        t_next; % For cycle confirmation
        x_opt = calc_gslam(hx_dr, hz);
        
        % Arbitrarily check the estimation
        if opt_anime
            figure
            hold on
            grid on
            % Landmark
            plot(RFID(:,1), RFID(:,2), '*k', 'Displayname','Landmarks')   
            
            % True orbit
            plot(hx_true(1,:), hx_true(2,:),'--','Color', '#000000', 'Displayname','Groundtruth','LineWidth',2)
            % dead reckoning
            plot(hx_dr(1,:), hx_dr(2,:), 'r', 'Displayname','Estimation from frontend','LineWidth',2)
            % Optimal estimation
            plot(x_opt(1,:), x_opt(2,:), 'b', 'Displayname','Optimized Trajectory','LineWidth',2)
            axis equal
            legend
        end
    end
end