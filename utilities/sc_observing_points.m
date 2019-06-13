function [point_index, SC_memory_use] = sc_observing_points(SC_memory_use, SC_max_memory, SC_current_pos, point_index, this_pos_points, ErosParameters, flag_SC_observing_points, bits_per_point )
%F_SC_OBSERVING_POINTS Summary of this function goes here
%   Detailed explanation goes here

% Infer Sizes
num_SC = size(SC_memory_use,1);
% num_points = size(this_pos_points,1);

% SC observing Points
for ns = 1:1:num_SC
    this_SC_pos = SC_current_pos(ns,1:3);
    not_observed_index = logical(point_index == 0);
    max_observation_distance = min(2*ErosParameters.radius, norm(this_SC_pos));
    distance_index = logical(vecnorm(this_pos_points - this_SC_pos,2,2) < max_observation_distance);
    this_SC_observes = (not_observed_index & distance_index);
    if sum(this_SC_observes) > 0
        switch flag_SC_observing_points
            case 0
                if SC_memory_use(ns,1) + sum(this_SC_observes)*bits_per_point < SC_max_memory
                    point_index(this_SC_observes) = ns;
                    SC_memory_use(ns,1) = SC_memory_use(ns,1) + sum(this_SC_observes)*bits_per_point;
                elseif SC_memory_use(ns,1) + bits_per_point > SC_max_memory
                    % Do nothing
                else
                    this_indices = find(this_SC_observes)';
                    for this_index_i = this_indices
                        if SC_memory_use(ns,1) + bits_per_point < SC_max_memory
                            point_index(this_index_i) = ns;
                            SC_memory_use(ns,1) = SC_memory_use(ns,1) + bits_per_point;
                        end
                    end
                end
            case 1
                % observe only 1 point per time step
                if SC_memory_use(ns,1) + bits_per_point <= SC_max_memory
                    this_distance_vector = [vecnorm(this_pos_points(this_SC_observes,:) - this_SC_pos,2,2),  find(this_SC_observes)];
                    [~,I_tdv] = sort(this_distance_vector(:,1));
                    this_index_i = round(this_distance_vector(I_tdv(1),2));
                    point_index(this_index_i) = ns;
                    SC_memory_use(ns,1) = SC_memory_use(ns,1) + bits_per_point;
                end
%             case 2
%                 % observe only 1 point per obs_time_threshold
%                 if (t - SC_last_obs_time(ns,1) >= obs_time_threshold)
%                     if SC_memory_use(ns,1) + bits_per_point <= SC_max_memory
%                         this_distance_vector = [vecnorm(this_pos_points(this_SC_observes,:) - this_SC_pos,2,2),  find(this_SC_observes)];
%                         [B_tdv,I_tdv] = sort(this_distance_vector(:,1));
%                         this_index_i = round(this_distance_vector(I_tdv(1),2));
%                         point_index(this_index_i) = ns;
%                         SC_memory_use(ns,1) = SC_memory_use(ns,1) + bits_per_point;
%                         SC_last_obs_time(ns,1) = t;
%                     end
%                 end
            otherwise
                disp('Should not reach here!')
        end
    end
end