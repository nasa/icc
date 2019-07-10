function priority = get_coverage_reward(observed_points, sc_obs_type )
%GET_COVERAGE_REWARD Defines the value of the observed points 
%   priority(i,j) is the value per bit/second acciciated with the
%   observed_points(i,j)
%
% This coverage reward function reduces the value of observing a point by
% 50 percent each time it is observed. 

N = size(observed_points,1);
K = size(observed_points,2); 

priority = zeros(N,K); 

%% Reward Function: Diminishing reward each time a point is observed
for i_time = 1:K
    for i_sc = 1:N
        %% Determine Priority of the Observed Point 
        
        % If a point is observed by i_sc at i_time, set Priority to 1/(number times observed by any spacecraft so far)
        if observed_points(i_sc, i_time) ~= 0
            n_times_observed = sum(sum( observed_points(i_sc, i_time)==observed_points(1:i_sc, 1:i_time ) )); 
            priority(i_sc, i_time) = 1/(n_times_observed); 
        end 
    end
end



end