function [ns_comm, SC_memory_use] = sc_communication_single_hop(old_SC_current_pos, SC_current_pos, carrier_SC_pos, AsteroidParameters, standard_comm_data_rate, SC_memory_use, SC_max_memory, delta_t)
%F_SC_COMMUNICATION_TEMP Summary of this function goes here
%   Detailed explanation goes here

num_SC = size(SC_memory_use,1); % number of (instrument) spacecraft

SC_visible_to_carrier = zeros(num_SC,1); % ratio of memory used by spacecraft to total memory available

% Calculate SC_visible_to_carrier
for ns = 1:1:num_SC
    
    old_this_SC_pos = old_SC_current_pos(ns,1:3);
    
    old_this_visible = sc_visible_to_carrier(old_this_SC_pos, carrier_SC_pos, AsteroidParameters.radius);
    
    this_SC_pos = SC_current_pos(ns,1:3);
    
    this_visible = sc_visible_to_carrier(this_SC_pos, carrier_SC_pos, AsteroidParameters.radius);
    
    if (old_this_visible==1) && (this_visible==1)
        
        SC_visible_to_carrier(ns,1) = (SC_memory_use(ns,1))/SC_max_memory; % priority by memory usage
        
    end
    
end

[~,I_SC] = sort(SC_visible_to_carrier,'descend');

ns_comm = I_SC(1); % ??? 

if SC_visible_to_carrier(ns_comm,1) > 0
    
    this_SC_pos = SC_current_pos(ns_comm,1:3);
    
    comm_data_rate = (1e4/(norm(this_SC_pos-carrier_SC_pos))^2) *standard_comm_data_rate;
    
    if SC_memory_use(ns_comm,1) > comm_data_rate*delta_t
        SC_memory_use(ns_comm,1) = SC_memory_use(ns_comm,1) - comm_data_rate*delta_t;
    else
        SC_memory_use(ns_comm,1) = 0;
    end
    
else
    ns_comm = 0;
end


end

