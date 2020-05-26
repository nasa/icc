% Stolen from relay_optimization_driver

%% Test overall gradient
%% Setup
clear all; close all; clc;

% Load things
addpath(genpath('../utilities/'))
addpath(genpath('../small_body_dynamics/'))
addpath(genpath('../network_flow_communication_optimizer/'))
addpath(genpath('../relay_orbit_optimizer/'))
addpath('../')

constants = initialize_SBDT();

eros_sbdt = loadEros( constants, 1, 1, 3, 3 );
ErosGravity = SphericalHarmonicsGravityIntegrator_SBDT(eros_sbdt);
GM = eros_sbdt.gravity.gm * 1e9;  % Convert to m from km

% Create a swarm object 
n_spacecraft = 4;
time_bounds = [0:300:86400];
sc_types = cell(n_spacecraft,1);
sc_types{1} = 1;
sc_types{2} = 1;
sc_types{3} = 1;
sc_types{4} = 0;
max_memory = ones(n_spacecraft,1)*1e10; %1TB

swarm = SpacecraftSwarm(time_bounds, sc_types, max_memory);

% Orbits
sc_initial_state_array = zeros(n_spacecraft,6);
% SC 1
sc_ix = 1;
sc_location = [30*1e3;0;0];
sc_orbital_vel = sqrt(GM/norm(sc_location));
sc_vel = [0; sc_orbital_vel*sqrt(2)/2; sc_orbital_vel*sqrt(2)/2];
sc_initial_state_array(sc_ix,:) = [sc_location; sc_vel];
sc_ix = sc_ix+1;
% SC 2
sc_location = [30*1e3;0;0];
sc_orbital_vel = sqrt(GM/norm(sc_location));
sc_vel = [0; sc_orbital_vel; 0];
sc_initial_state_array(sc_ix,:) = [sc_location; sc_vel];
sc_ix = sc_ix+1;
% Relay
sc_location = [50*1e3;0;0];
sc_orbital_vel = sqrt(GM/norm(sc_location));
sc_vel = [0; sc_orbital_vel; 0];
sc_initial_state_array(sc_ix,:) = [sc_location; sc_vel];
sc_ix = sc_ix+1;
% Carrier
sc_location = [100*1e3;0;0];
sc_orbital_vel = sqrt(GM/norm(sc_location));
sc_vel = [0; sc_orbital_vel; 0];
sc_initial_state_array(sc_ix,:) = [sc_location; sc_vel];
sc_ix = sc_ix +1;
% Integrate
swarm.integrate_trajectories(ErosGravity, sc_initial_state_array);

% Science parameters
swarm.Observation.flow = zeros(n_spacecraft,length(time_bounds));
swarm.Observation.flow(1:2,:) = 1e9; %1Gb/time step
swarm.Observation.priority = zeros(n_spacecraft,length(time_bounds));
swarm.Observation.priority(1:2,:) = 1;

% Bandwidth params
bandwidth_parameters.reference_bandwidth = 250000;
bandwidth_parameters.reference_distance = 100000;
bandwidth_parameters.max_bandwidth = 100*1e6;

% Try optimizing comms right now for sanity - maybe with wrong comms model
[swarm] = observation_and_communication_optimizer(ErosGravity, swarm);
% plot_communications(swarm, ErosGravity,true)addpath(genpath('../utilities/'))
initialize_SBDT();

% This scales the variables size to attempt to make the problem better
% conditioned. 
optvar_scaling_factor = [1e-4; 1e-4; 1e-4; 1; 1; 1];

N = swarm.get_num_spacecraft();
% Create initial conditions from Swarm object


initial_conditions = zeros(6*N,1);
for sc =1:N
    offset = 6*(sc-1);
    assert(all(size(swarm.abs_trajectory_array(1,:,sc)') == size(optvar_scaling_factor)));
    initial_conditions(1+offset:6+offset) = swarm.abs_trajectory_array(1,:,sc)'.*optvar_scaling_factor;
end

% One test call to the cost function
%[goal, gradient] = integrated_optimization_cost_function(swarm,initial_conditions, optvar_scaling_factor, gravity_model, bandwidth_parameters);
[goal] = integrated_optimization_cost_function(swarm,initial_conditions, optvar_scaling_factor, ErosGravity, bandwidth_parameters);
% if (isnan(goal) || any(isnan(gradient)))
if (isnan(goal))
    error('ERROR: initial location is infeasible. fmincon will crash.')
end

% Proper cost function
fun = @(params) integrated_optimization_cost_function(swarm, params, optvar_scaling_factor, ErosGravity, bandwidth_parameters);

% Try calling the "proper cost function"
% [goal, gradient] = fun(initial_conditions);
[goal] = fun(initial_conditions);
disp("Ready");

 %% Compute numerical gradients
% gradient_steps = [1e3, 1e2, 1e1, 1, 1e-2, 1e-4, 1e-6, 1e-8];
gradient_steps = [1, 0.1, 0.01,0.001, 0.0001];
% gradient_perturbation = [.05; .05; .05; 10.; 10.; 10.;];
gradient_perturbation = [100; 100; 100; 1.; 1.; 1.;];
gradient_perturbation = repmat(gradient_perturbation,N,1);

num_gradient = cell(size(gradient_steps));
legend_labels = cell(length(gradient_steps)+1, 1);
for step_index = 1:length(gradient_steps)
    num_gradient{step_index} = numerical_gradient(fun, initial_conditions, gradient_steps(step_index).*gradient_perturbation);
    legend_labels{step_index} = sprintf("Numerical, %d", gradient_steps(step_index));
end
legend_labels{end} = "Analytical";


% Plotting
figure()
line_type = cell(length(gradient_steps)+1, 1);
line_available_strokes = {'-', ':', '-.', '--'};
line_available_marks = {'.', 'o', 'x', '+', '*', 's', 'd', 'v', '^', '<', '>', 'p', 'h'};

for step_index = 1:length(gradient_steps)
    stroke_idx = mod(step_index, length(line_available_strokes))+1;
    mark_idx = mod(step_index, length(line_available_marks))+1;
    semilogy(abs(num_gradient{step_index}), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end 

step_index = length(gradient_steps)+1;
stroke_idx = mod(step_index, length(line_available_strokes))+1;
mark_idx = mod(step_index, length(line_available_marks))+1;
% semilogy(abs(semi_analytical_gradient), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))
legend(legend_labels, 'location', 'best')
title("Full gradient of reward wrt initial location")

%% Test just communications on targeted links
% communication_optimizer(swarm, bandwidth_model). Need to reach in and
% change bandwidths.

N = swarm.get_num_spacecraft;
K = swarm.get_num_timesteps;

bandwidth_model = @(x1,x2) min(bandwidth_parameters.reference_bandwidth * (bandwidth_parameters.reference_distance/norm(x2-x1,2))^2, bandwidth_parameters.max_bandwidth); 
% bandwidths_and_memories = zeros(K,N,N);

bandwidths_and_memories = swarm.Communication.bandwidths_and_memories;

% Choose a few bandwidths to perturb
num_bandwidths_to_perturb = 10;
k_ix = randi(K, num_bandwidths_to_perturb,1);
n1_ix = randi(N, num_bandwidths_to_perturb,1);
n2_ix = randi(N, num_bandwidths_to_perturb,1);

% gradient_steps = [1e3, 1e2, 1e1, 1, 1e-2, 1e-4, 1e-6, 1e-8];
gradient_steps = [1e2, 1e1, 1, 1e-2];


 % Compute gradients
num_gradient_bw = cell(size(gradient_steps));
legend_labels = cell(length(gradient_steps)+2, 1);

data_scaling_factor = 1e6;

swarmb = copy(swarm);

% [swarm] = observation_and_communication_optimizer(ErosGravity, swarmb, params);

fun_b = @(params) observation_and_communication_optimizer(ErosGravity, swarmb, params, data_scaling_factor);

for step_index = 1:length(gradient_steps)
    gradient_step = gradient_steps(step_index);
    fprintf("Gradient step %d / %d (%d)\n", step_index, length(gradient_steps), gradient_step)
    semi_analytical_gradient = zeros(num_bandwidths_to_perturb,1);
    parfor entry = 1:num_bandwidths_to_perturb
        fprintf("Entry %d/%d\n", entry, num_bandwidths_to_perturb)
        perturbation = zeros(size(bandwidths_and_memories));
        perturbation(k_ix(entry), n1_ix(entry), n2_ix(entry)) = gradient_step;
        try
            [~, goalplus] = fun_b(max(0,bandwidths_and_memories+perturbation));
            [~, goalminus] = fun_b(max(0,bandwidths_and_memories-perturbation));
            semi_analytical_gradient(entry) = (goalplus - goalminus)/(2*gradient_step);
        catch e
           semi_analytical_gradient(entry)= NaN;
           warning("No gradient could be computed.")  % at point "+ sprintf(" %d", bandwidths_and_memories+perturbation))
           fprintf(1,'The identifier was:\n%s',e.identifier);
           fprintf(1,'There was an error! The message was:\n%s',e.message);
        end
    end
    num_gradient_bw{step_index} = semi_analytical_gradient;
    legend_labels{step_index} = sprintf("Numerical, %d", gradient_steps(step_index));
end

% observation_and_communication_optimizer(ErosGravity, swarmb, swarmb.Communication.bandwidths_and_memories);

[swarmc, goal] = observation_and_communication_optimizer(ErosGravity, swarmb, swarmb.Communication.bandwidths_and_memories);
analytical_gradient_bw = zeros(num_bandwidths_to_perturb,1);
for entry=1:num_bandwidths_to_perturb
    analytical_gradient_bw(entry) = swarmc.Communication.dual_bandwidths_and_memories(k_ix(entry), n1_ix(entry), n2_ix(entry));
end
% Another way
% Reuse the gradient we had computed before
% [dk_dic, dk_dbandwidth, dbandwidth_dlocation, dlocation_dic] = compute_gradient(swarmc, bandwidth_parameters.reference_distance, bandwidth_parameters.reference_bandwidth, bandwidth_parameters.max_bandwidth);
% analytical_gradient_bw_c = zeros(num_bandwidths_to_perturb,1);
% for entry=1:num_bandwidths_to_perturb
%     index = N*K*(n2_ix(entry)-1) + K*(n1_ix(entry)-1) + k_ix(entry);
%     analytical_gradient_bw_c(entry) = dk_dbandwidth(index);
% end


legend_labels{end-1} = "Analytical";
legend_labels{end} = "Analytical (fun)";


% Plotting


figure()
line_type = cell(length(gradient_steps)+1, 1);
line_available_strokes = {'-', ':', '-.', '--'};
line_available_marks = {'.', 'o', 'x', '+', '*', 's', 'd', 'v', '^', '<', '>', 'p', 'h'};

for step_index = 1:length(gradient_steps)
    stroke_idx = mod(step_index, length(line_available_strokes))+1;
    mark_idx = mod(step_index, length(line_available_marks))+1;
    semilogy(abs(num_gradient_bw{step_index}), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end


step_index = length(gradient_steps)+1;
stroke_idx = mod(step_index, length(line_available_strokes))+1;

mark_idx = mod(step_index, length(line_available_marks))+1;
semilogy(abs(analytical_gradient_bw), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))
step_index = length(gradient_steps)+2;
stroke_idx = mod(step_index, length(line_available_strokes))+2;
mark_idx = mod(step_index, length(line_available_marks))+2;
% semilogy(abs(analytical_gradient_bw_c), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))

legend(legend_labels, 'location', 'best')
title("Gradient of reward wrt bandwidth")

%% Test communications and all positions on targeted links
% communication_optimizer(swarm, bandwidth_model). Need to reach in and
% change bandwidths. after modifying swarm.abs_trajectory_array(k, 1:3, i).

swarmb = copy(swarm);
N = swarmb.get_num_spacecraft;
K = swarmb.get_num_timesteps;

bandwidth_model = @(x1,x2) min(bandwidth_parameters.reference_bandwidth * (bandwidth_parameters.reference_distance/norm(x2-x1,2))^2, bandwidth_parameters.max_bandwidth); 
% bandwidths_and_memories = zeros(K,N,N);

data_scaling_factor = 1e6;

num_locations_to_perturb = 20;
k_ix = randi(K, num_locations_to_perturb,1);
dir_ix=randi(3, num_locations_to_perturb,1);
n_ix = randi(N, num_locations_to_perturb,1);

% We perturb the relay (at 50, 0, 0) in all three directions.
% Recall that s1 and s2 are at (30, 0, 0) and s4 is at (100, 0, 0)
% k_ix = [1, 1, 1];
% dir_ix = [1, 2, 3]; 
% n_ix = [2, 2, 2];

gradient_steps= [50, 5, .5];

semi_analytical_gradient = zeros(num_locations_to_perturb, length(gradient_steps)+1);
gradient_bw_wrt_pos = cell(num_locations_to_perturb, length(gradient_steps)+1);

% Perturb locations

abs_trajectory_array = swarmb.abs_trajectory_array;
sample_times = swarmb.sample_times;

legend_labels = cell(length(gradient_steps)+1,1);

for delta_pose_ix = 1:length(gradient_steps)
    fprintf("Delta pose %d/%d\n", delta_pose_ix, length(gradient_steps))
    delta_pos = gradient_steps(delta_pose_ix);
    legend_labels{delta_pose_ix} = sprintf("Numerical, %d", delta_pos);
    parfor loc =1:num_locations_to_perturb
        fprintf("Location %d/%d\n", loc, num_locations_to_perturb)
        abs_trajectory_array_plus = swarmb.abs_trajectory_array;
        abs_trajectory_array_minus = swarmb.abs_trajectory_array;
        % sample_times = swarm.sample_times;
        abs_trajectory_array_plus(k_ix(loc), dir_ix(loc), n_ix(loc))  = abs_trajectory_array(k_ix(loc), dir_ix(loc), n_ix(loc)) + delta_pos; 
        abs_trajectory_array_minus(k_ix(loc), dir_ix(loc), n_ix(loc)) = abs_trajectory_array(k_ix(loc), dir_ix(loc), n_ix(loc)) - delta_pos; 

        % Compute perturbed bandwidths 
        bandwidths_and_memories       = zeros(K,N,N);
        bandwidths_and_memories_plus  = zeros(K,N,N);
        bandwidths_and_memories_minus = zeros(K,N,N);

        for k=1:K-1
            for i=1:N
                for j=1:N
                    if j~=i
                        bandwidths_and_memories(k,i,j)       = bandwidth_model(abs_trajectory_array(k,1:3,i),       abs_trajectory_array(k,1:3,j))* (sample_times(k+1)-sample_times(k));
                        bandwidths_and_memories_plus(k,i,j)  = bandwidth_model(abs_trajectory_array_plus(k,1:3,i),  abs_trajectory_array_plus(k,1:3,j))* (sample_times(k+1)-sample_times(k));
                        bandwidths_and_memories_minus(k,i,j) = bandwidth_model(abs_trajectory_array_minus(k,1:3,i), abs_trajectory_array_minus(k,1:3,j))* (sample_times(k+1)-sample_times(k));

                    elseif ~isnan(swarm.Parameters.available_memory(i)) && ~isinf(swarm.Parameters.available_memory(i))
                        bandwidths_and_memories(k,i,j)       = swarm.Parameters.available_memory(i);
                        bandwidths_and_memories_plus(k,i,j)  = swarm.Parameters.available_memory(i);
                        bandwidths_and_memories_minus(k,i,j) = swarm.Parameters.available_memory(i);

                    else
                        bandwidths_and_memories(k,i,j)       = max_memory;
                        bandwidths_and_memories_plus(k,i,j)  = max_memory;
                        bandwidths_and_memories_minus(k,i,j) = max_memory;
                    end
                end
            end
        end

        % Compute the perturbed bw numerically
        try
            [~, goalplus]   = communication_optimizer(swarmb, bandwidths_and_memories_plus, data_scaling_factor);
            [~, goalminus]  = communication_optimizer(swarmb, bandwidths_and_memories_minus, data_scaling_factor);
            local_gradient = (goalplus - goalminus)/(2*abs(delta_pos));
        catch e
           local_gradient= NaN;
           warning("No gradient could be computed.")  % at point "+ sprintf(" %d", bandwidths_and_memories+perturbation))
           fprintf(1,'The identifier was:\n%s',e.identifier);
           fprintf(1,'There was an error! The message was:\n%s',e.message);
        end
        semi_analytical_gradient(loc,delta_pose_ix) = local_gradient;
        temp_gradient_bw_wrt_pos = (bandwidths_and_memories_plus-bandwidths_and_memories_minus)./(2*abs(delta_pos));
        gradient_bw_wrt_pos{loc, delta_pose_ix} = temp_gradient_bw_wrt_pos(k_ix(loc),:,:);
    end
end

% Compute the perturbed bw analytically
% Reuse dk_dbandwidth from above

% [swarmb, goal] = communication_optimizer(swarmb, bandwidth_model, data_scaling_factor);
% [dk_dic, dk_dbandwidth, dbandwidth_dlocation, dlocation_dic] = compute_gradient(swarmb, bandwidth_parameters.reference_distance, bandwidth_parameters.reference_bandwidth, bandwidth_parameters.max_bandwidth);

dk_dlocation = dk_dbandwidth*dbandwidth_dlocation;

for loc=1:num_locations_to_perturb
    % THis indexing is iffy - and indeed the problem may lie here?
    index = K*N*(dir_ix(loc)-1) + K*( n_ix(loc)-1) + k_ix(loc);
    semi_analytical_gradient(loc,end) = dk_dlocation(index);
            temp_gradient_bw_wrt_pos = reshape(dbandwidth_dlocation(:,index),K,N,N);

    gradient_bw_wrt_pos{loc,end} = temp_gradient_bw_wrt_pos(k_ix(loc),:,:);
end
legend_labels{end} = sprintf("Analytical");

% Plotting
figure()
line_type = cell(length(gradient_steps)+1, 1);
line_available_strokes = {'-', ':', '-.', '--'};
line_available_marks = {'.', 'o', 'x', '+', '*', 's', 'd', 'v', '^', '<', '>', 'p', 'h'};

for step_index = 1:length(gradient_steps)
    stroke_idx = mod(step_index, length(line_available_strokes))+1;
    mark_idx = mod(step_index, length(line_available_marks))+1;
    semilogy(abs(semi_analytical_gradient(:,step_index)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end 

step_index = length(gradient_steps)+1;
stroke_idx = mod(step_index, length(line_available_strokes))+1;
mark_idx = mod(step_index, length(line_available_marks))+1;

semilogy(abs(semi_analytical_gradient(:,end)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))
legend(legend_labels, 'location', 'best')
title("Gradient of reward wrt location")


% AHA. The error seems to be here. Specifically, in how we compute the
% numerical gradient wrt the bandwidth model. All the numerical gradients
% agree remarkably well, and the analytical gradient has similar shape but
% wildly different values.

gradient_bw_wrt_loc_norm = zeros(num_locations_to_perturb,length(gradient_steps)+1);
fprintf(strcat("Location: ",mat2str(gradient_steps),"\n"))
for loc =1:num_locations_to_perturb
    fprintf("%d: ",loc)
    for delta_pose_ix = 1:length(gradient_steps)
        gradient_bw_wrt_loc_norm(loc,delta_pose_ix) = sum(sum(sum(abs(gradient_bw_wrt_pos{loc,delta_pose_ix}))));
    
        fprintf("%f (%f X),",sum(sum(sum(abs(gradient_bw_wrt_pos{loc,delta_pose_ix}-gradient_bw_wrt_pos{loc,end})))), sum(sum(sum(abs(gradient_bw_wrt_pos{loc,delta_pose_ix}-gradient_bw_wrt_pos{loc,end}))))/sum(sum(sum(abs(gradient_bw_wrt_pos{loc,end})))));
    end
    fprintf("\n")
    gradient_bw_wrt_loc_norm(loc,end) = sum(sum(sum(abs(gradient_bw_wrt_pos{loc,end}))));

end

% figure()
% spy(abs(gradient_bw_wrt_pos{loc,delta_pose_ix}-gradient_bw_wrt_pos{loc,end}))

figure()
for delta_pose_ix = 1:length(gradient_steps)+1
    stroke_idx = mod(delta_pose_ix, length(line_available_strokes))+1;
    mark_idx = mod(delta_pose_ix, length(line_available_marks))+1;
    plot(gradient_bw_wrt_loc_norm(:,delta_pose_ix),strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end
legend(legend_labels, 'location', 'best')
title("Gradient of bandwidth wrt location")

figure()
for delta_pose_ix = 1:length(gradient_steps)+1
    stroke_idx = mod(delta_pose_ix, length(line_available_strokes))+1;
    mark_idx = mod(delta_pose_ix, length(line_available_marks))+1;
    semilogy(gradient_bw_wrt_loc_norm(:,delta_pose_ix),strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end
legend(legend_labels, 'location', 'best')
title("Gradient of bandwidth wrt location (log)")

%% I feel I am going crazy.
% 
% 
% bandwidth_parameters.reference_bandwidth = 250000;
% bandwidth_parameters.reference_distance = 100000;
% bandwidth_parameters.max_bandwidth = 100*1e6;
% scaling_factor = bandwidth_parameters.reference_distance;
% 
% bandwidth_model = @(x1,x2) quadratic_comm_model(x1,x2,bandwidth_parameters.reference_distance, bandwidth_parameters.reference_bandwidth,bandwidth_parameters.max_bandwidth,scaling_factor);
% 
% test_len=1e4;
% x1=linspace(40000,60000,test_len);
% x2=30000*ones(size(x1));
% % linspace(0,30000,test_len);
% 
% bw = zeros(test_len,1);
% dbw_dx1 = zeros(size(bw));
% dbw_dx2 = zeros(size(bw));
% dbw_dx1_num = zeros(size(bw));
% dbw_dx2_num = zeros(size(bw));
% 
% for i = 1:length(bw)
%     bw(i) = bandwidth_model(x1(i),x2(i));
%     dbw_dx1(i) = diff_quadratic_comm_model_x1(x1(i),x2(i),1,bandwidth_parameters.reference_distance,bandwidth_parameters.reference_bandwidth,bandwidth_parameters.max_bandwidth,scaling_factor);
%     dbw_dx2(i) = diff_quadratic_comm_model_x2(x1(i),x2(i),1,bandwidth_parameters.reference_distance,bandwidth_parameters.reference_bandwidth,bandwidth_parameters.max_bandwidth,scaling_factor);
% end
% for i = 2:length(bw)-1
%     dbw_dx1_num(i) = (bw(i+1)-bw(i-1))./(x1(i+1)-x1(i-1));
%     dbw_dx2_num(i) = (bw(i+1)-bw(i-1))./(x2(i+1)-x2(i-1));        
% end
% 
% 
% 
% figure()
% % plot(x1,bw)
% hold all
% plot(x1,dbw_dx1,':')
% plot(x1,dbw_dx1_num,'-.')
% plot(x1,dbw_dx1-dbw_dx1_num,'r')
% legend("Analytical", "Numerical","Difference")
% figure()
% % plot(x2,bw)
% hold all
% plot(x2,dbw_dx2)
% plot(x2,dbw_dx2_num)
% legend("Analytical", "Numerical")
% 
% 
% function [bandwidth] = quadratic_comm_model(x1, x2, reference_distance,reference_bandwidth,max_bandwidth, scaling_factor)
%     if nargin<6
%         scaling_factor=reference_distance;
%     end
%     x1=x1./scaling_factor;
%     x2=x2./scaling_factor;
%     reference_distance=reference_distance./scaling_factor;
%     bandwidth = min(max_bandwidth, reference_bandwidth*(reference_distance/norm(x2-x1,2))^2);
% end
% 
% function [db_dx1] = diff_quadratic_comm_model_x1(x1, x2, dir, reference_distance,reference_bandwidth,max_bandwidth,scaling_factor)
%     if nargin<7
%         scaling_factor=reference_distance;
%     end
% 
%     if quadratic_comm_model(x1, x2, reference_distance,reference_bandwidth,max_bandwidth,scaling_factor)>=max_bandwidth
%         db_dx1 = 0; %zeros(size(x1));
%     else
%         x1=x1./scaling_factor;
%         x2=x2./scaling_factor;
%         reference_distance=reference_distance./scaling_factor;        
% %          db_dx1 = -2*reference_bandwidth*(reference_distance/norm(x2-x1,2)^2)^2 * (x1(dir)-x2(dir));
%         % Numerical conditioning - let's try and get something that looks
%         % like a distance before squaring
%          db_dx1 = -2*reference_bandwidth.* ( reference_distance/norm(x2-x1,2)^2 ).^2 .* (x1(dir)-x2(dir));
%          db_dx1 = db_dx1./scaling_factor;
%     end
%     
% end
% 
% function [db_dx2] = diff_quadratic_comm_model_x2(x1, x2, dir, reference_distance,reference_bandwidth,max_bandwidth, scaling_factor)
%     if nargin<7
%         scaling_factor=reference_distance;
%     end
%     db_dx2 = - diff_quadratic_comm_model_x1(x1, x2, dir, reference_distance,reference_bandwidth,max_bandwidth, scaling_factor);
% end

%% Gradient of location with respect to initial position

swarmb = copy(swarm);
N = swarmb.get_num_spacecraft;
K = swarmb.get_num_timesteps;

% Numerical gradient of initial location wrt location
num_ics_to_perturb = 20;
perturbation_magnitudes = [10, 1, 0.1, 0.01,0.001];

sc_id =  randi(N, num_ics_to_perturb,1);
%sc_id = 3*ones(num_ics_to_perturb,1);
direction_id =  randi(6, num_ics_to_perturb,1);
% time_id =  randi(K, num_ics_to_perturb,1);

% swarmb.integrate_trajectories(ErosGravity, sc_initial_state_array);

numerical_state_transition_matrices=cell(num_ics_to_perturb,length(perturbation_magnitudes)+1);

for perturb_id =1:num_ics_to_perturb
    parfor perturb_mag_id = 1:length(perturbation_magnitudes)
        fprintf("Perturbation %d/%d, gradient %d/%d\n",perturb_id, num_ics_to_perturb, perturb_mag_id,length(perturbation_magnitudes))
        if direction_id(perturb_id)<4
            perturbation = 100*perturbation_magnitudes(perturb_mag_id); %100m
        else
            perturbation = 1*perturbation_magnitudes(perturb_mag_id); %1m/s
        end
        temp_swarm = copy(swarmb);
        temp_sc_initial_state_array = sc_initial_state_array;

        temp_sc_initial_state_array(sc_id(perturb_id), direction_id(perturb_id)) = temp_sc_initial_state_array(sc_id(perturb_id), direction_id(perturb_id)) + perturbation;
        temp_swarm.integrate_trajectories(ErosGravity, temp_sc_initial_state_array);
        nominal_trajectory = swarmb.abs_trajectory_array(:,1:3,sc_id(perturb_id));
        temp_trajectory = temp_swarm.abs_trajectory_array(:,1:3,sc_id(perturb_id));
        temp_stm = ((temp_trajectory-nominal_trajectory)./perturbation)';

        numerical_state_transition_matrices{perturb_id,perturb_mag_id} = temp_stm;
    end
    sbdt_stm = swarmb.state_transition_matrix(:,direction_id(perturb_id),:,sc_id(perturb_id));
    sbdt_stm = reshape(sbdt_stm,[6,K]);
    numerical_state_transition_matrices{perturb_id,end} = sbdt_stm(1:3,:);
end

% figure()
% plot3(temp_trajectory(:,1),temp_trajectory(:,2),temp_trajectory(:,3))
% hold all
% plot3(nominal_trajectory(:,1),nominal_trajectory(:,2),nominal_trajectory(:,3))
% axis equal
% figure()
% plot3(temp_trajectory(:,1)-nominal_trajectory(:,1),temp_trajectory(:,2)-nominal_trajectory(:,2),temp_trajectory(:,3)-nominal_trajectory(:,3))
% temp_sc_initial_state_array-sc_initial_state_array


stm_norm = zeros(num_ics_to_perturb,length(perturbation_magnitudes)+1);
labels = cell(length(perturbation_magnitudes)+1,1);
fprintf(strcat("PID: ",mat2str(perturbation_magnitudes),"\n"))
for perturb_id =1:num_ics_to_perturb
    fprintf("%d: ",perturb_id)
    sbdt_stm = numerical_state_transition_matrices{perturb_id,end};
    for perturb_mag_id = 1:length(perturbation_magnitudes)
        num_stm  = numerical_state_transition_matrices{perturb_id,perturb_mag_id};
        diff_stm = num_stm - sbdt_stm;
        fprintf("%f (%f X error) ", norm(diff_stm), norm(diff_stm)/norm(sbdt_stm));
        stm_norm(perturb_id,perturb_mag_id) = norm(num_stm);
        labels{perturb_mag_id} = sprintf("%d",perturbation_magnitudes(perturb_mag_id));
    end
    stm_norm(perturb_id,length(perturbation_magnitudes)+1) = norm(sbdt_stm);
    labels{end} = "Analytical";
    fprintf("\n")
end

% Plotting
line_available_strokes = {'-', ':', '-.', '--'};
line_available_marks = {'.', 'o', 'x', '+', '*', 's', 'd', 'v', '^', '<', '>', 'p', 'h'};

figure()
for perturb_mag_id = 1:length(perturbation_magnitudes)+1
    stroke_idx = mod(perturb_mag_id, length(line_available_strokes))+1;
    mark_idx = mod(perturb_mag_id, length(line_available_marks))+1;
    semilogy(stm_norm(:,perturb_mag_id),strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end
title("Gradient of location wrt initial position (norm)")
legend(labels,'location','best');

save(strcat('gradient_experiments', datestr(datetime,"yyyymmdd_HHMMSS")))