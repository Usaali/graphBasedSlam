clear all
close all
clc
fid = fopen('landmark_vec.txt');
hz = {};
while ~feof(fid)
    tline = fgets(fid);
    hz = [hz sscanf(tline, "%f %f %f %f;",[4,inf]).'];
end
fclose(fid);

global MAX_ITR
MAX_ITR = 30;
global nx
nx = 3;
C_sigma1 = 0.1;
C_sigma2 = 0.1;
C_sigma3 = 1.0*2*pi/360;
global sigma
sigma = diag([C_sigma1^2, C_sigma2^2, C_sigma3^2]);
path=load('truth.txt');
est=load('est.txt');
est=est.';
path=path.';
RFID = load('landmarks.txt');

x_opt = calc_gslam(est(:,1:50), hz(1:50));
figure
hold on
grid on
% Landmark
plot(RFID(:,1), RFID(:,2), '*k')   
            
% True orbit
plot(path(1,:), path(2,:), 'b')
% dead reckoning
plot(est(1,:), est(2,:), 'k')
% Optimal estimation
plot(x_opt(1,:), x_opt(2,:), 'r')
axis equal