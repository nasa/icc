function [isc_comm, sc_memory_use] = simulate_single_hop_communication(sc_previous_pos, sc_current_pos, carrier_current_pos, asteroid_radius, standard_comm_data_rate, sc_memory_use, sc_max_memory, delta_t)
%SC_COMMUNICATION_SINGLE_HOP Simulates single-hop relay of data to the
%carrier, using a heuristic to decide which spacecraft sends its data
%   Syntax: [isc_comm, sc_memory_use] = sc_communication_single_hop(sc_previous_pos,sc_current_pos,
%             carrier_current_pos, asteroid_radius, standard_comm_data_rate, sc_memory_use, sc_max_memory, delta_t)
%
%   Inputs:
%    - sc_previous_pos [m]: [N_SPACECRAFT x 3] Array of sc positions at
%   	previous iteration
%    - sc_current_pos [m]: [N_SPACECRAFT x 3] Array of sc positions at
%    	current iteration
%    - carrier_current_pos [m]: [N_SPACECRAFT x 3] Array of carrier
%    	position at current iteration
%    - asteroid_radius [m]: Equatorial radius of asteroid
%    - standard_comm_data_rate [bits/s]: Data flow rate between spacecraft
%    	and carrier
%    - sc_memory_use [bits]: Vector containing the current memory usage of
%    	each spacecraft
%    - sc_max_memory [bits]: Scalar indicating the maximum memory capacity
%       of the spacecraft
%    - delta_t [s]: Discrete timestep used for the update
%
%   Outputs:
%    - isc_comm: Index of the spacecraft chosen to relay its data
%    - sc_memory_use [bits]: Updated memory usage vector

%% Setup
n_spacecraft = size(sc_memory_use,1); % number of spacecraft (not including carrier)

SC_visible_to_carrier = zeros(n_spacecraft,1); % ratio of memory used by spacecraft to total memory available

%% Choose a spacecraft to relay its data
for ns = 1:1:n_spacecraft
    
    old_this_SC_pos = sc_previous_pos(ns,1:3);
    
    old_this_visible = sc_visible_to_carrier(old_this_SC_pos, carrier_current_pos, asteroid_radius);
    
    this_SC_pos = sc_current_pos(ns,1:3);
    
    this_visible = sc_visible_to_carrier(this_SC_pos, carrier_current_pos, asteroid_radius);
    
    if (old_this_visible==1) && (this_visible==1)
        
        SC_visible_to_carrier(ns,1) = (sc_memory_use(ns,1))/sc_max_memory; % priority by memory usage
        
    end
    
end

[~,I_SC] = sort(SC_visible_to_carrier,'descend');

isc_comm = I_SC(1); % This is the spacecraft chosen to communicate with the carrier

%% Simulate the Data Transfer
if SC_visible_to_carrier(isc_comm,1) > 0
    
    this_SC_pos = sc_current_pos(isc_comm,1:3);
    
    comm_data_rate = (1e4/(norm(this_SC_pos-carrier_current_pos))^2) *standard_comm_data_rate;
    
    if sc_memory_use(isc_comm,1) > comm_data_rate*delta_t
        sc_memory_use(isc_comm,1) = sc_memory_use(isc_comm,1) - comm_data_rate*delta_t;
    else
        sc_memory_use(isc_comm,1) = 0;
    end
    
else
    isc_comm = 0;
end


end

