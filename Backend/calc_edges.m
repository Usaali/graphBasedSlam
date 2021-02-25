%% Edge cost calculation
%     argument?F
%         Self-estimated?Fxlist
%         Landmark?Fzlist
%     Return value?F
%         Edge structure set?Fedges
function edges = calc_edges(xlist, zlist)
edges = [];
cost = 0.0;
nt_z = size(zlist,2);  
z_IDs = nchoosek([1:nt_z],2);    % Combination array of data numbers (corresponds to time + 1)
%z_IDs = [[1:nt_z-1]',[2:nt_z]'];

for j = 1:size(z_IDs,1)  % For all combinations
    % Data number extraction
    t1 = z_IDs(j,1);    % Discrete time+1 t1 
    t2 = z_IDs(j,2);    % Discrete time+1 t2
    
    zlist_temp1 = zlist{t1};
    zlist_temp2 = zlist{t2};
    
    if isempty(zlist_temp1) || isempty(zlist_temp2)
        continue
    end
    % For all observed landmark information
    for k = 1:size(zlist_temp1,1)
        for l = 1:size(zlist_temp2,1)
            % Compare the same landmark information
            if zlist_temp1(k,4) == zlist_temp2(l,4)                
                % Edge generation & cost addition (because it takes the sum)
                edge = make_edge(xlist(:, t1), xlist(:, t2), zlist{t1}(k,:), zlist{t2}(l,:), t1, t2);
                cost = cost + edge.e'*edge.OMEGA*edge.e;    % Weighted by information matrix ?? (inv of covariance)
                edges = [edges, edge];                               
            end
        end
    end
end
fprintf('cost:%f, nedge:%f \n',cost, length(edges))
