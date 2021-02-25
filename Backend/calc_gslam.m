%% graph based slam calculation
%     argument：
%         dead reckoning matrix：hx_dr
%         Landmark observation matrix：hz
%     Return value：
%         Optimal estimation matrix：x_opt
function x_opt = calc_gslam(hx_dr, hz)
global MAX_ITR nx
disp('start graph based slam')
zlist = hz;   

x_opt = hx_dr;      % Initialize
nt = size(x_opt,2); % Check the number of elements
N = nx*nt;     

% Iteration from here
for j = 1:MAX_ITR
    %Edge cost calculation (check the sum of costs for all combinations)
    edges = calc_edges(x_opt, zlist);
    
    % Information matrix H and information vector b
    H = zeros(N,N);
    b = zeros(N,1);
    
    % Generate information matrix / vector from all edges
    for k = 1:size(edges,2)
        [H, b] = make_partHb(H, b, edges(k));
    end
    
    H(1:nx, 1:nx) = H(1:nx, 1:nx) + eye(nx);
    
    % Correction value calculation 
    dx = -inv(H)*b;
    
    % Estimated value correction, correction value added to each column of history
    for l = 1:nt
        x_opt(:,l) = x_opt(:,l) + dx(nx*(l-1)+1:nx*l,1);
    end
    
    % Iteration end condition
    diff = dx'*dx;  % Correction amount norm squared
%     fprintf('iteration: %d, diff: %f\n', j, diff)
    if diff < 1e-5
        diff
        break
    end    
end