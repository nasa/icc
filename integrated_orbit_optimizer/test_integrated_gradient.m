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
sc_location = [24*1e3;0;0];
sc_orbital_vel = sqrt(GM/norm(sc_location));
sc_vel = [0; sc_orbital_vel*sqrt(2)/2; sc_orbital_vel*sqrt(2)/2];
sc_initial_state_array(sc_ix,:) = [sc_location; sc_vel];
sc_ix = sc_ix+1;
% SC 2
sc_location = [26*1e3;0;0];
sc_orbital_vel = sqrt(GM/norm(sc_location));
sc_vel = [0; sc_orbital_vel; 0];
sc_initial_state_array(sc_ix,:) = [sc_location; sc_vel];
sc_ix = sc_ix+1;
% Relay
sc_location = [32*1e3;0;0];
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
[goal, gradient] = integrated_optimization_cost_function(swarm,initial_conditions, optvar_scaling_factor, ErosGravity, bandwidth_parameters);
% [goal] = integrated_optimization_cost_function(swarm,initial_conditions, optvar_scaling_factor, ErosGravity, bandwidth_parameters);
% if (isnan(goal) || any(isnan(gradient)))
if (isnan(goal))
    error('ERROR: initial location is infeasible. fmincon will crash.')
end

% Proper cost function
fun = @(params) integrated_optimization_cost_function(swarm, params, optvar_scaling_factor, ErosGravity, bandwidth_parameters);

% Try calling the "proper cost function"
[goal, semi_analytical_gradient] = fun(initial_conditions);
% [goal] = fun(initial_conditions);
disp("Ready");

 %% Compute numerical gradients
% gradient_steps = [1e3, 1e2, 1e1, 1, 1e-2, 1e-4, 1e-6, 1e-8];
gradient_steps = [0.01,0.001, 0.0001, 0.00001];
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

for step_index = 2:length(gradient_steps)
    stroke_idx = mod(step_index, length(line_available_strokes))+1;
    mark_idx = mod(step_index, length(line_available_marks))+1;
    semilogy(abs(num_gradient{step_index}), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end 

step_index = length(gradient_steps)+1;
stroke_idx = mod(step_index, length(line_available_strokes))+1;
mark_idx = mod(step_index, length(line_available_marks))+1;
semilogy(abs(semi_analytical_gradient), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))
legend(legend_labels(2:length(gradient_steps)+1), 'location', 'best')
title("Full gradient of reward wrt initial location")

%% Test just communications on targeted links
% communication_optimizer(swarm, bandwidth_model). Need to reach in and
% change bandwidths.
 num_bandwidths_to_perturb = 15;
% gradient_steps = [1e3, 1e2, 1e1, 1, 1e-2, 1e-4, 1e-6, 1e-8];
gradient_steps = [1e7, 1e5, 1e3];

N = swarm.get_num_spacecraft;
K = swarm.get_num_timesteps;

swarmb = copy(swarm);
bandwidths_and_memories = swarm.Communication.bandwidths_and_memories;

data_scaling_factor = 1e6;

% Analytical gradient
[swarmb, goal] = observation_and_communication_optimizer(ErosGravity, swarmb, swarmb.Communication.bandwidths_and_memories);

% Choose a few nonzero bandwidths to perturb

% k_ix = randi(K, num_bandwidths_to_perturb,1);
% n1_ix = randi(N, num_bandwidths_to_perturb,1);
% n2_ix = randi(N, num_bandwidths_to_perturb,1);

% Indices of the random locations, picked to be nonzero
k_ix = zeros(num_bandwidths_to_perturb,1);
n1_ix = zeros(num_bandwidths_to_perturb,1);
n2_ix = zeros(num_bandwidths_to_perturb,1);

randix = 0;
%randoffset = randi(len(dk_dlocation_obs)/2,1);
for k=1:K
    for n1=1:N
        for n2=1:N
            if abs(swarmb.Communication.dual_bandwidths_and_memories(k, n1, n2))>100*eps
                randix = randix+1;
                k_ix(randix) = k;
                n1_ix(randix) = n1;
                n2_ix(randix) = n2;
            end
            if randix == num_bandwidths_to_perturb
                break
            end
        end
        if randix == num_bandwidths_to_perturb
            break
        end
    end
    if randix == num_bandwidths_to_perturb
        break
    end
end

if randix<num_bandwidths_to_perturb
    disp("I could not find enough non-zero-gradient locations!")
    k_ix(randix+1:num_bandwidths_to_perturb)  = randi(K, num_bandwidths_to_perturb-randix,1);
    n1_ix(randix+1:num_bandwidths_to_perturb) = randi(N, num_bandwidths_to_perturb-randix,1);
    n2_ix(randix+1:num_bandwidths_to_perturb) = randi(N, num_bandwidths_to_perturb-randix,1);
end

% Extract analytical gradients
analytical_gradient_bw = zeros(num_bandwidths_to_perturb,1);
for entry=1:num_bandwidths_to_perturb
    analytical_gradient_bw(entry) = swarmb.Communication.dual_bandwidths_and_memories(k_ix(entry), n1_ix(entry), n2_ix(entry));
end
% Another way
% Reuse the gradient we had computed before
[~, dk_dbandwidth] = compute_integrated_gradient(swarmb, bandwidth_parameters);
analytical_gradient_bw_c = zeros(num_bandwidths_to_perturb,1);
for entry=1:num_bandwidths_to_perturb
    index = N*K*(n2_ix(entry)-1) + K*(n1_ix(entry)-1) + k_ix(entry);
    analytical_gradient_bw_c(entry) = dk_dbandwidth(index);
end


 % Compute numerical gradients
num_gradient_bw = cell(size(gradient_steps));
legend_labels = cell(length(gradient_steps)+2, 1);

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

legend_labels{end-1} = "Analytical";
legend_labels{end} = "Analytical (fun)";


% Plotting


figure()
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
stroke_idx = mod(step_index, length(line_available_strokes))+1;
mark_idx = mod(step_index, length(line_available_marks))+1;
semilogy(abs(analytical_gradient_bw_c), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))

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

num_locations_to_perturb = 10;
k_ix = randi(K, num_locations_to_perturb,1);
dir_ix=randi(3, num_locations_to_perturb,1);
n_ix = randi(N, num_locations_to_perturb,1);

% We perturb the relay (at 50, 0, 0) in all three directions.
% Recall that s1 and s2 are at (30, 0, 0) and s4 is at (100, 0, 0)
% k_ix = [1, 1, 1];
% dir_ix = [1, 2, 3]; 
% n_ix = [2, 2, 2];

gradient_steps= [5, .5, .05, .005];

semi_analytical_gradient = zeros(num_locations_to_perturb, length(gradient_steps)+3);
gradient_bw_wrt_pos = cell(num_locations_to_perturb, length(gradient_steps)+1);
legend_labels = cell(length(gradient_steps)+3,1);


% Perturb locations
abs_trajectory_array = swarmb.abs_trajectory_array;
sample_times = swarmb.sample_times;


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
            [~, goalplus]   = observation_and_communication_optimizer(ErosGravity, swarmb, bandwidths_and_memories_plus, data_scaling_factor);
            [~, goalminus]  = observation_and_communication_optimizer(ErosGravity, swarmb, bandwidths_and_memories_minus, data_scaling_factor);
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

% We want to test how the reward changes if we perturb the orbits but NOT
% the bandwidths, which will tell us about dk_dposition_obs
% To do this, we call o_a_c_o with unchanged bws, but perturbed orbits.

% Compute the perturbed bw analytically
% Reuse dk_dbandwidth from above

[swarmb, goal] =  observation_and_communication_optimizer(ErosGravity, swarmb, bandwidth_model, data_scaling_factor);

spherical_asteroid_parameters.max_radius = ErosGravity.BodyModel.shape.maxRadius*1e3;
spherical_asteroid_parameters.min_radius = ErosGravity.BodyModel.shape.maxRadius*1e3;

[dk_dic, dk_dbandwidth, dbandwidth_dlocation, dk_dlocation_obs, dlocation_dic] = compute_integrated_gradient(swarm, bandwidth_parameters, spherical_asteroid_parameters);

dk_dlocation_bw = dk_dbandwidth*dbandwidth_dlocation;

for loc=1:num_locations_to_perturb
    index = K*N*(dir_ix(loc)-1) + K*( n_ix(loc)-1) + k_ix(loc);
    semi_analytical_gradient(loc,end-2) = dk_dlocation_bw(index);
    semi_analytical_gradient(loc,end-1) = dk_dlocation_obs(index);
    semi_analytical_gradient(loc,end) = dk_dlocation_bw(index)+dk_dlocation_obs(index);;
    temp_gradient_bw_wrt_pos = reshape(dbandwidth_dlocation(:,index),K,N,N);

    gradient_bw_wrt_pos{loc,end} = temp_gradient_bw_wrt_pos(k_ix(loc),:,:);
end
legend_labels{end-2} = sprintf("Analytical (BW)");
legend_labels{end-1} = sprintf("Analytical (Obs)");
legend_labels{end} = sprintf("Analytical (BW+Obs)");

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

% Gradient WRT bandwidth
step_index = length(gradient_steps)+1;
stroke_idx = mod(step_index, length(line_available_strokes))+1;
mark_idx = mod(step_index, length(line_available_marks))+1;
semilogy(abs(semi_analytical_gradient(:,end-2)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))
% % Gradient WRT obs
% step_index = length(gradient_steps)+2;
% stroke_idx = mod(step_index, length(line_available_strokes))+1;
% mark_idx = mod(step_index, length(line_available_marks))+1;
% semilogy(abs(semi_analytical_gradient(:,end-1)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))
% % Gradient WRT both
% step_index = length(gradient_steps)+3;
% stroke_idx = mod(step_index, length(line_available_strokes))+1;
% mark_idx = mod(step_index, length(line_available_marks))+1;
% semilogy(abs(semi_analytical_gradient(:,end)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))

legend(legend_labels, 'location', 'best')
title("Gradient of reward wrt location")

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

%% Occlusion gradients: line to point

num_locations_to_perturb = 30;
gradient_steps= [.05, .005,.0005, .00005];

legend_labels = cell(length(gradient_steps)+1,1);
legend_labels{end} = "Analytical";

numerical_gradient_v1 = zeros(length(gradient_steps), num_locations_to_perturb);
numerical_gradient_v2 = zeros(length(gradient_steps), num_locations_to_perturb);
analytical_gradient_v1 = zeros(1,num_locations_to_perturb);
analytical_gradient_v2 = zeros(1,num_locations_to_perturb);

for loc=1:num_locations_to_perturb
    fprintf("Location %d/%d\n", loc, num_locations_to_perturb)    
    v1 = rand(3,1)*1e4;
    v2 = rand(3,1)*1e3;
    vp = rand(3,1)*1e2;
    % Analytical
    [distance, closest_point, ddistance_dv1, ddistance_dv2] = distance_line_to_point(v1, v2, vp);
    
    perturbation_direction = randi(3,1);
    perturbation_sign = 2*(round(rand)-.5);
    
    analytical_gradient_v1(loc) = ddistance_dv1(perturbation_direction)*perturbation_sign;
    
    for delta_pose_ix = 1:length(gradient_steps)
        delta_pos = gradient_steps(delta_pose_ix);
        legend_labels{delta_pose_ix} = sprintf("Numerical, %d", delta_pos);
        fprintf("Delta pose %d/%d\n", delta_pose_ix, length(gradient_steps))
        
        perturbation = zeros(3,1);
        perturbation(perturbation_direction) = delta_pos*perturbation_sign;
        [distance_plus] = distance_line_to_point(v1+perturbation, v2, vp);
        [distance_minus] = distance_line_to_point(v1-perturbation, v2, vp);

        numerical_gradient_v1(delta_pose_ix, loc) = (distance_plus-distance_minus)/(2*(abs(delta_pos)));
    end
end
% Plotting
figure()
subplot(2,1,1)
line_available_strokes = {'-', ':', '-.', '--'};
line_available_marks = {'.', 'o', 'x', '+', '*', 's', 'd', 'v', '^', '<', '>', 'p', 'h'};

for step_index = 1:length(gradient_steps)
    stroke_idx = mod(step_index, length(line_available_strokes))+1;
    mark_idx = mod(step_index, length(line_available_marks))+1;
    semilogy(abs(numerical_gradient_v1(step_index, :)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end

step_index = length(gradient_steps)+1;
stroke_idx = mod(step_index, length(line_available_strokes))+1;
mark_idx = mod(step_index, length(line_available_marks))+1;
semilogy(abs(analytical_gradient_v1), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))

legend(legend_labels, 'location', 'best')
title("Gradient of distance wrt v1 (line)")

for loc=1:num_locations_to_perturb
    fprintf("Location %d/%d\n", loc, num_locations_to_perturb)    
    v1 = rand(3,1)*1e4;
    v2 = rand(3,1)*1e3;
    vp = rand(3,1)*1e2;
    % Analytical
    [distance, closest_point, ddistance_dv1, ddistance_dv2] = distance_line_to_point(v1, v2, vp);
    
    perturbation_direction = randi(3,1);
    perturbation_sign = 2*(round(rand)-.5);
    
    analytical_gradient_v2(loc) = ddistance_dv2(perturbation_direction)*perturbation_sign;
    
    for delta_pose_ix = 1:length(gradient_steps)
        delta_pos = gradient_steps(delta_pose_ix);
        legend_labels{delta_pose_ix} = sprintf("Numerical, %d", delta_pos);
        fprintf("Delta pose %d/%d\n", delta_pose_ix, length(gradient_steps))
        
        perturbation = zeros(3,1);
        perturbation(perturbation_direction) = delta_pos*perturbation_sign;
        [distance_plus_v2] = distance_line_to_point(v1, v2+perturbation, vp);
        [distance_minus_v2] = distance_line_to_point(v1, v2-perturbation, vp);

        numerical_gradient_v2(delta_pose_ix, loc) = (distance_plus_v2-distance_minus_v2)/(2*(abs(delta_pos)));
    end
end
% Plotting
subplot(2,1,2)
line_available_strokes = {'-', ':', '-.', '--'};
line_available_marks = {'.', 'o', 'x', '+', '*', 's', 'd', 'v', '^', '<', '>', 'p', 'h'};

for step_index = 1:length(gradient_steps)
    stroke_idx = mod(step_index, length(line_available_strokes))+1;
    mark_idx = mod(step_index, length(line_available_marks))+1;
    semilogy(abs(numerical_gradient_v2(step_index, :)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end

step_index = length(gradient_steps)+1;
stroke_idx = mod(step_index, length(line_available_strokes))+1;
mark_idx = mod(step_index, length(line_available_marks))+1;
semilogy(abs(analytical_gradient_v2), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))

legend(legend_labels, 'location', 'best')
title("Gradient of distance wrt v2 (line)")

%% Occlusion gradients: segment to point

num_locations_to_perturb = 30;
gradient_steps= [.05, .005,.0005, .00005];

legend_labels = cell(length(gradient_steps)+1,1);
legend_labels{end} = "Analytical";

numerical_gradient_v1 = zeros(length(gradient_steps), num_locations_to_perturb);
numerical_gradient_v2 = zeros(length(gradient_steps), num_locations_to_perturb);
analytical_gradient_v1 = zeros(1,num_locations_to_perturb);
analytical_gradient_v2 = zeros(1,num_locations_to_perturb);

for loc=1:num_locations_to_perturb
    fprintf("Location %d/%d\n", loc, num_locations_to_perturb)    
    v1 = rand(3,1)*1e3;
    v2 = rand(3,1)*1e3;
    vp = rand(3,1)*1e2;
    % Analytical
    [distance, closest_point, ddistance_dv1, ddistance_dv2] = distance_segment_to_point(v1, v2, vp);
    
    perturbation_direction = randi(3,1);
    perturbation_sign = 2*(round(rand)-.5);
    
    analytical_gradient_v1(loc) = ddistance_dv1(perturbation_direction)*perturbation_sign;
    analytical_gradient_v2(loc) = ddistance_dv2(perturbation_direction)*perturbation_sign;
    
    for delta_pose_ix = 1:length(gradient_steps)
        delta_pos = gradient_steps(delta_pose_ix);
        legend_labels{delta_pose_ix} = sprintf("Numerical, %d", delta_pos);
        fprintf("Delta pose %d/%d\n", delta_pose_ix, length(gradient_steps))
        
        perturbation = zeros(3,1);
        perturbation(perturbation_direction) = delta_pos*perturbation_sign;
        [distance_plus_v1] = distance_segment_to_point(v1+perturbation, v2, vp);
        [distance_minus_v1] = distance_segment_to_point(v1-perturbation, v2, vp);
        [distance_plus_v2] = distance_segment_to_point(v1, v2+perturbation, vp);
        [distance_minus_v2] = distance_segment_to_point(v1, v2-perturbation, vp);

        numerical_gradient_v1(delta_pose_ix, loc) = (distance_plus_v1-distance_minus_v1)/(2*(abs(delta_pos)));
        numerical_gradient_v2(delta_pose_ix, loc) = (distance_plus_v2-distance_minus_v2)/(2*(abs(delta_pos)));

    end
end
% Plotting
figure()
subplot(2,1,1)
line_available_strokes = {'-', ':', '-.', '--'};
line_available_marks = {'.', 'o', 'x', '+', '*', 's', 'd', 'v', '^', '<', '>', 'p', 'h'};

for step_index = 1:length(gradient_steps)
    stroke_idx = mod(step_index, length(line_available_strokes))+1;
    mark_idx = mod(step_index, length(line_available_marks))+1;
    semilogy(abs(numerical_gradient_v1(step_index, :)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end

step_index = length(gradient_steps)+1;
stroke_idx = mod(step_index, length(line_available_strokes))+1;
mark_idx = mod(step_index, length(line_available_marks))+1;
semilogy(abs(analytical_gradient_v1), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))

legend(legend_labels, 'location', 'best')
title("Gradient of distance wrt v1 (segment)")

subplot(2,1,2)
line_available_strokes = {'-', ':', '-.', '--'};
line_available_marks = {'.', 'o', 'x', '+', '*', 's', 'd', 'v', '^', '<', '>', 'p', 'h'};

for step_index = 1:length(gradient_steps)
    stroke_idx = mod(step_index, length(line_available_strokes))+1;
    mark_idx = mod(step_index, length(line_available_marks))+1;
    semilogy(abs(numerical_gradient_v2(step_index, :)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end

step_index = length(gradient_steps)+1;
stroke_idx = mod(step_index, length(line_available_strokes))+1;
mark_idx = mod(step_index, length(line_available_marks))+1;
semilogy(abs(analytical_gradient_v2), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))

legend(legend_labels, 'location', 'best')
title("Gradient of distance wrt v2 (segment)")

%% Occlusion gradient - occlusion wrt positions
bbandwidth_parameters.reference_bandwidth = 250;
bbandwidth_parameters.reference_distance = 100000;
bbandwidth_parameters.max_bandwidth = 100000;

num_locations_to_perturb = 30;
gradient_steps= [.05, .005,.0005, .00005];

legend_labels = cell(length(gradient_steps)+1,1);
legend_labels{end} = "Analytical";

spherical_asteroid_params_o.max_radius = ErosGravity.BodyModel.shape.maxRadius*1e3;
spherical_asteroid_params_o.min_radius = ErosGravity.BodyModel.shape.minRadius*1e3;

occlusion_test = @(x1,x2) is_occluded(x1, x2, spherical_asteroid_params_o);

numerical_gradient_v1 = zeros(length(gradient_steps), num_locations_to_perturb);
numerical_gradient_v2 = zeros(length(gradient_steps), num_locations_to_perturb);
analytical_gradient_v1 = zeros(1,num_locations_to_perturb);
analytical_gradient_v2 = zeros(1,num_locations_to_perturb);

for loc =1:num_locations_to_perturb
%     fprintf("Location %d/%d\n", loc, num_locations_to_perturb)
    perturbation_direction = randi(3,1);
    perturbation_sign = 2*(round(rand)-.5);
    
    v1 = rand(3,1)*20000;
    v2 = cross(v1,[1,0,0]');
    
    [bandwidth, db_dv1, db_dv2] =occlusion_test(v1,v2);
    
    analytical_gradient_v1(loc) = db_dv1(perturbation_direction)*perturbation_sign;
    analytical_gradient_v2(loc) = db_dv2(perturbation_direction)*perturbation_sign;

    for delta_pose_ix = 1:length(gradient_steps)
        delta_pos = gradient_steps(delta_pose_ix);
        legend_labels{delta_pose_ix} = sprintf("Numerical, %d", delta_pos);
%         fprintf("Delta pose %d/%d\n", delta_pose_ix, length(gradient_steps))
        perturbation = zeros(3,1);
        perturbation(perturbation_direction) = delta_pos*perturbation_sign;
        [oc_plus_v1] = occlusion_test(v1+perturbation, v2);
        [oc_minus_v1] = occlusion_test(v1-perturbation, v2);
        numerical_gradient_v1(delta_pose_ix, loc) = (oc_plus_v1-oc_minus_v1)/(2*(abs(delta_pos)));
        
        [oc_plus_v2] = occlusion_test(v1, v2+perturbation);
        [oc_minus_v2] = occlusion_test(v1, v2-perturbation);
        numerical_gradient_v2(delta_pose_ix, loc) = (oc_plus_v2-oc_minus_v2)/(2*(abs(delta_pos)));
    end
end


% Plotting
line_available_strokes = {'-', ':', '-.', '--'};
line_available_marks = {'.', 'o', 'x', '+', '*', 's', 'd', 'v', '^', '<', '>', 'p', 'h'};

figure()
subplot(2,1,1)

for step_index = 1:length(gradient_steps)
    stroke_idx = mod(step_index, length(line_available_strokes))+1;
    mark_idx = mod(step_index, length(line_available_marks))+1;
    semilogy(abs(numerical_gradient_v1(step_index, :)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end

step_index = length(gradient_steps)+1;
stroke_idx = mod(step_index, length(line_available_strokes))+1;
mark_idx = mod(step_index, length(line_available_marks))+1;
semilogy(abs(analytical_gradient_v1), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))

legend(legend_labels, 'location', 'best')
title("Gradient of occlusion wrt v1 (occlusion)")

% Plotting
subplot(2,1,2)

for step_index = 1:length(gradient_steps)
    stroke_idx = mod(step_index, length(line_available_strokes))+1;
    mark_idx = mod(step_index, length(line_available_marks))+1;
    semilogy(abs(numerical_gradient_v2(step_index, :)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end

step_index = length(gradient_steps)+1;
stroke_idx = mod(step_index, length(line_available_strokes))+1;
mark_idx = mod(step_index, length(line_available_marks))+1;
semilogy(abs(analytical_gradient_v2), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))

legend(legend_labels, 'location', 'best')
title("Gradient of bandwidth wrt v2 (occlusion)")

%% Occlusion gradient - bandwidth wrt pose without occlusion
bbandwidth_parameters.reference_bandwidth = 250;
bbandwidth_parameters.reference_distance = 100000;
bbandwidth_parameters.max_bandwidth = 100000;

num_locations_to_perturb = 30;
gradient_steps= [.05, .005,.0005, .00005];

legend_labels = cell(length(gradient_steps)+1,1);
legend_labels{end} = "Analytical";

spherical_asteroid_params_o.max_radius = ErosGravity.BodyModel.shape.maxRadius*1e3;
spherical_asteroid_params_o.min_radius = ErosGravity.BodyModel.shape.minRadius*1e3;

numerical_gradient_v1 = zeros(length(gradient_steps), num_locations_to_perturb);
numerical_gradient_v2 = zeros(length(gradient_steps), num_locations_to_perturb);
analytical_gradient_v1 = zeros(1,num_locations_to_perturb);
analytical_gradient_v2 = zeros(1,num_locations_to_perturb);

for loc =1:num_locations_to_perturb
%     fprintf("Location %d/%d\n", loc, num_locations_to_perturb)
    perturbation_direction = randi(3,1);
    perturbation_sign = 2*(round(rand)-.5);
    
    vl = 16000;
    v1 = rand(3,1);
    v1 = v1/norm(v1)*vl;
    v2 = cross(v1,[1,0,0]');
    v2 = v2/norm(v2)*vl;
    
    [bandwidth] = quadratic_comm_model(v1, v2, bbandwidth_parameters);
    [ag_v1, ag_v2] = diff_quadratic_comm_model(v1, v2, perturbation_direction, bbandwidth_parameters);
    analytical_gradient_v1(loc) = ag_v1*perturbation_sign;
    analytical_gradient_v2(loc) = ag_v2*perturbation_sign;

    for delta_pose_ix = 1:length(gradient_steps)
        delta_pos = gradient_steps(delta_pose_ix);
        legend_labels{delta_pose_ix} = sprintf("Numerical, %d", delta_pos);
%         fprintf("Delta pose %d/%d\n", delta_pose_ix, length(gradient_steps))
        perturbation = zeros(3,1);
        perturbation(perturbation_direction) = delta_pos*perturbation_sign;
        [bw_plus_v1] = quadratic_comm_model(v1+perturbation, v2, bbandwidth_parameters);
        [bw_minus_v1] = quadratic_comm_model(v1-perturbation, v2, bbandwidth_parameters);
        numerical_gradient_v1(delta_pose_ix, loc) = (bw_plus_v1-bw_minus_v1)/(2*(abs(delta_pos)));
        
        [bw_plus_v2] = quadratic_comm_model(v1, v2+perturbation, bbandwidth_parameters);
        [bw_minus_v2] = quadratic_comm_model(v1, v2-perturbation, bbandwidth_parameters);
        numerical_gradient_v2(delta_pose_ix, loc) = (bw_plus_v2-bw_minus_v2)/(2*(abs(delta_pos)));
    end
end


% Plotting
line_available_strokes = {'-', ':', '-.', '--'};
line_available_marks = {'.', 'o', 'x', '+', '*', 's', 'd', 'v', '^', '<', '>', 'p', 'h'};

figure()
subplot(2,1,1)

for step_index = 1:length(gradient_steps)
    stroke_idx = mod(step_index, length(line_available_strokes))+1;
    mark_idx = mod(step_index, length(line_available_marks))+1;
    semilogy(abs(numerical_gradient_v1(step_index, :)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end

step_index = length(gradient_steps)+1;
stroke_idx = mod(step_index, length(line_available_strokes))+1;
mark_idx = mod(step_index, length(line_available_marks))+1;
semilogy(abs(analytical_gradient_v1), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))

legend(legend_labels, 'location', 'best')
title("Gradient of bandwidth wrt v1 (no occlusion)")

% Plotting
subplot(2,1,2)

for step_index = 1:length(gradient_steps)
    stroke_idx = mod(step_index, length(line_available_strokes))+1;
    mark_idx = mod(step_index, length(line_available_marks))+1;
    semilogy(abs(numerical_gradient_v2(step_index, :)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end

step_index = length(gradient_steps)+1;
stroke_idx = mod(step_index, length(line_available_strokes))+1;
mark_idx = mod(step_index, length(line_available_marks))+1;
semilogy(abs(analytical_gradient_v2), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))

legend(legend_labels, 'location', 'best')
title("Gradient of of bandwidth wrt v2 (no occlusion)")

%% Occlusion gradient - bandwidth wrt pose w. occlusion
bbandwidth_parameters.reference_bandwidth = 250;
bbandwidth_parameters.reference_distance = 100000;
bbandwidth_parameters.max_bandwidth = 100000;

num_locations_to_perturb = 30;
gradient_steps= [.05, .005,.0005, .00005];

legend_labels = cell(length(gradient_steps)+1,1);
legend_labels{end} = "Analytical";

spherical_asteroid_params_o.max_radius = ErosGravity.BodyModel.shape.maxRadius*1e3;
spherical_asteroid_params_o.min_radius = ErosGravity.BodyModel.shape.minRadius*1e3;

occlusion_test = @(x1,x2) is_occluded(x1, x2, spherical_asteroid_params_o);

numerical_gradient_v1 = zeros(length(gradient_steps), num_locations_to_perturb);
numerical_gradient_v2 = zeros(length(gradient_steps), num_locations_to_perturb);
analytical_gradient_v1 = zeros(1,num_locations_to_perturb);
analytical_gradient_v2 = zeros(1,num_locations_to_perturb);

for loc =1:num_locations_to_perturb
%     fprintf("Location %d/%d\n", loc, num_locations_to_perturb)
    perturbation_direction = randi(3,1);
    perturbation_sign = 2*(round(rand)-.5);
    
    vl = 16000;
    v1 = rand(3,1);
    v1 = v1/norm(v1)*vl;
    v2 = cross(v1,[1,0,0]');
    v2 = v2/norm(v2)*vl;
    
    [bandwidth] = quadratic_comm_model(v1, v2, bbandwidth_parameters, occlusion_test);
    [ag_v1, ag_v2] = diff_quadratic_comm_model(v1, v2, perturbation_direction, bbandwidth_parameters, occlusion_test);
    analytical_gradient_v1(loc) = ag_v1*perturbation_sign;
    analytical_gradient_v2(loc) = ag_v2*perturbation_sign;

    for delta_pose_ix = 1:length(gradient_steps)
        delta_pos = gradient_steps(delta_pose_ix);
        legend_labels{delta_pose_ix} = sprintf("Numerical, %d", delta_pos);
%         fprintf("Delta pose %d/%d\n", delta_pose_ix, length(gradient_steps))
        perturbation = zeros(3,1);
        perturbation(perturbation_direction) = delta_pos*perturbation_sign;
        [bw_plus_v1] = quadratic_comm_model(v1+perturbation, v2, bbandwidth_parameters, occlusion_test);
        [bw_minus_v1] = quadratic_comm_model(v1-perturbation, v2, bbandwidth_parameters, occlusion_test);
        numerical_gradient_v1(delta_pose_ix, loc) = (bw_plus_v1-bw_minus_v1)/(2*(abs(delta_pos)));
        
        [bw_plus_v2] = quadratic_comm_model(v1, v2+perturbation, bbandwidth_parameters, occlusion_test);
        [bw_minus_v2] = quadratic_comm_model(v1, v2-perturbation, bbandwidth_parameters, occlusion_test);
        numerical_gradient_v2(delta_pose_ix, loc) = (bw_plus_v2-bw_minus_v2)/(2*(abs(delta_pos)));
    end
end


% Plotting
line_available_strokes = {'-', ':', '-.', '--'};
line_available_marks = {'.', 'o', 'x', '+', '*', 's', 'd', 'v', '^', '<', '>', 'p', 'h'};

figure()
subplot(2,1,1)

for step_index = 1:length(gradient_steps)
    stroke_idx = mod(step_index, length(line_available_strokes))+1;
    mark_idx = mod(step_index, length(line_available_marks))+1;
    semilogy(abs(numerical_gradient_v1(step_index, :)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end

step_index = length(gradient_steps)+1;
stroke_idx = mod(step_index, length(line_available_strokes))+1;
mark_idx = mod(step_index, length(line_available_marks))+1;
semilogy(abs(analytical_gradient_v1), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))

legend(legend_labels, 'location', 'best')
title("Gradient of bandwidth wrt v1 (occlusion)")

% Plotting
subplot(2,1,2)

for step_index = 1:length(gradient_steps)
    stroke_idx = mod(step_index, length(line_available_strokes))+1;
    mark_idx = mod(step_index, length(line_available_marks))+1;
    semilogy(abs(numerical_gradient_v2(step_index, :)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end

step_index = length(gradient_steps)+1;
stroke_idx = mod(step_index, length(line_available_strokes))+1;
mark_idx = mod(step_index, length(line_available_marks))+1;
semilogy(abs(analytical_gradient_v2), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))

legend(legend_labels, 'location', 'best')
title("Gradient of of bandwidth wrt v2 (occlusion)")

%% Range gradients
num_locations_to_perturb = 30;
gradient_steps= [.05, .005,.0005, .00005];

legend_labels = cell(length(gradient_steps)+1,1);
legend_labels{end} = "Analytical";

distance_ranges{1} = [0,15000];
distance_tolerances{1} = 200;
numerical_gradient = zeros(length(gradient_steps), num_locations_to_perturb);
analytical_gradient = zeros(1,num_locations_to_perturb);

for loc =1:num_locations_to_perturb
    fprintf("Location %d/%d\n", loc, num_locations_to_perturb)
    
    
    v1 = rand(3,1)*1e4;
    v2 = rand(3,1)*1e3;
    [range_margin, drange_margin] = ...
        range_check(v1, v2, distance_ranges, distance_tolerances);

    perturbation_direction = randi(3,1);
    perturbation_sign = 2*(round(rand)-.5);
    
    analytical_gradient(loc) = drange_margin(perturbation_direction)*perturbation_sign;
    for delta_pose_ix = 1:length(gradient_steps)
        delta_pos = gradient_steps(delta_pose_ix);
        legend_labels{delta_pose_ix} = sprintf("Numerical, %d", delta_pos);
        fprintf("Delta pose %d/%d\n", delta_pose_ix, length(gradient_steps))
        perturbation = zeros(3,1);
        perturbation(perturbation_direction) = delta_pos*perturbation_sign;
        [range_margin_plus] = ...
                range_check(v1+perturbation, v2, distance_ranges, distance_tolerances);
        [range_margin_minus] = ...
                range_check(v1-perturbation, v2, distance_ranges, distance_tolerances);
        numerical_gradient(delta_pose_ix, loc) = (range_margin_plus-range_margin_minus)/(2*(abs(delta_pos)));
    end
end

% Plotting
figure()
line_available_strokes = {'-', ':', '-.', '--'};
line_available_marks = {'.', 'o', 'x', '+', '*', 's', 'd', 'v', '^', '<', '>', 'p', 'h'};

for step_index = 1:length(gradient_steps)
    stroke_idx = mod(step_index, length(line_available_strokes))+1;
    mark_idx = mod(step_index, length(line_available_marks))+1;
    semilogy(abs(numerical_gradient(step_index, :)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end

step_index = length(gradient_steps)+1;
stroke_idx = mod(step_index, length(line_available_strokes))+1;
mark_idx = mod(step_index, length(line_available_marks))+1;
semilogy(abs(analytical_gradient), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))

legend(legend_labels, 'location', 'best')
title("Gradient of margin wrt range (obs)")

%% Angle gradients
num_locations_to_perturb = 30;
gradient_steps= [5, .5, .05, .005,.0005];

legend_labels = cell(length(gradient_steps)+1,1);
legend_labels{end} = "Analytical";

angle_ranges{1} = [0,3];
angle_tolerances{1} = .5;
numerical_gradient = zeros(length(gradient_steps), num_locations_to_perturb);
analytical_gradient = zeros(1,num_locations_to_perturb);

for loc =1:num_locations_to_perturb
    fprintf("Location %d/%d\n", loc, num_locations_to_perturb)
    delta_pos = gradient_steps(delta_pose_ix);
    
    v1 = rand(3,1);
    v2 = rand(3,1);
    [range_margin, drange_margin] = ...
        angle_check(v1, v2, angle_ranges, angle_tolerances);

    perturbation_direction = randi(3,1);
    perturbation_sign = 2*(round(rand)-.5);
    
    analytical_gradient(loc) = drange_margin(perturbation_direction)*perturbation_sign;
    for delta_pose_ix = 1:length(gradient_steps)
        legend_labels{delta_pose_ix} = sprintf("Numerical, %d", delta_pos);
        fprintf("Delta pose %d/%d\n", delta_pose_ix, length(gradient_steps))
        perturbation = zeros(3,1);
        perturbation(perturbation_direction) = delta_pos*perturbation_sign;
        [margin_plus] = ...
                angle_check(v1+perturbation, v2, angle_ranges, angle_tolerances);
        [margin_minus] = ...
                angle_check(v1-perturbation, v2, angle_ranges, angle_tolerances);
        numerical_gradient(delta_pose_ix, loc) = (margin_plus-margin_minus)/(2*(abs(delta_pos)));
    end
end

% Plotting
figure()
line_available_strokes = {'-', ':', '-.', '--'};
line_available_marks = {'.', 'o', 'x', '+', '*', 's', 'd', 'v', '^', '<', '>', 'p', 'h'};

for step_index = 1:length(gradient_steps)
    stroke_idx = mod(step_index, length(line_available_strokes))+1;
    mark_idx = mod(step_index, length(line_available_marks))+1;
    semilogy(abs(numerical_gradient(step_index, :)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end

step_index = length(gradient_steps)+1;
stroke_idx = mod(step_index, length(line_available_strokes))+1;
mark_idx = mod(step_index, length(line_available_marks))+1;
semilogy(abs(analytical_gradient), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))

legend(legend_labels, 'location', 'best')
title("Gradient of margin wrt angle (obs)")

%% Gradient of individual points' observation rewards with respect to sc position
num_locations_to_perturb = 30;

gradient_steps= [5, .5, .05, .005,.0005];

swarme = copy(swarm);
N = swarme.get_num_spacecraft;
K = swarme.get_num_timesteps;

bandwidth_model = @(x1,x2) min(bandwidth_parameters.reference_bandwidth * (bandwidth_parameters.reference_distance/norm(x2-x1,2))^2, bandwidth_parameters.max_bandwidth); 
% bandwidths_and_memories = zeros(K,N,N);

data_scaling_factor = 1e6;
% 
% [swarme, goal] =  observation_and_communication_optimizer(ErosGravity, swarme, bandwidth_model, data_scaling_factor);
% 
% spherical_asteroid_parameters.max_radius = ErosGravity.BodyModel.shape.maxRadius*1e3;
% spherical_asteroid_parameters.min_radius = ErosGravity.BodyModel.shape.maxRadius*1e3;
% 
% [dk_dic, dk_dbandwidth, dbandwidth_dlocation, dk_dlocation_obs, dlocation_dic] = compute_integrated_gradient(swarm, bandwidth_parameters, spherical_asteroid_parameters);

% Now let's pick a few points. For each point, let's perturb the location
% of the observing spacecraft. Let's then check whether the gradient
% matches the one returned by get_observable_points

observable_points = cell(N,K);
observable_points_values = cell(N,K);
observable_points_gradients = cell(N,K);
observable_points_gradients_next_sc = cell(N,K);

perturbed_location_index = 0;

pt_id = zeros(num_locations_to_perturb,1);
pt_index = zeros(num_locations_to_perturb,1);
k_ix = zeros(num_locations_to_perturb,1);
dir_ix = zeros(num_locations_to_perturb,1);
n_ix = zeros(num_locations_to_perturb,1);

perturbed_location_analytical_gradients = zeros(num_locations_to_perturb,1);

for i_sc=1:N
    for i_time=1:K
        [observable_points{i_sc,i_time}, ...
            observable_points_values{i_sc,i_time}, ...
            observable_points_gradients{i_sc,i_time}, ...
            observable_points_gradients_next_sc{i_sc,i_time}] = ...
            get_observable_points(ErosGravity, swarme, i_time, i_sc, 0.);
        if ~isempty(observable_points{i_sc,i_time})
            for observable_pt_ix = 1:length(observable_points{i_sc,i_time})
                if observable_points_gradients{i_sc,i_time}(observable_pt_ix)>1e-8
                    perturbed_location_index = perturbed_location_index+1;

                    pt_id(perturbed_location_index) = observable_points{i_sc,i_time}(observable_pt_ix);
                    pt_index(perturbed_location_index) = observable_pt_ix;
                    k_ix(perturbed_location_index)= i_time;
                    dir_ix(perturbed_location_index)= randi(3,1);
                    n_ix(perturbed_location_index)= i_sc;
                    perturbed_location_analytical_gradients(perturbed_location_index) = ...
                        observable_points_gradients{i_sc,i_time}(observable_pt_ix, dir_ix(perturbed_location_index));

                    if perturbed_location_index>=num_locations_to_perturb
                        break
                    end
                end
            end
        end
        if perturbed_location_index>=num_locations_to_perturb
            break
        end
    end
    if perturbed_location_index>=num_locations_to_perturb
        break
    end
end

if perturbed_location_index<num_locations_to_perturb
    disp("Not enough nonzero entries!")
    num_locations_to_perturb = perturbed_location_index
end


semi_analytical_gradient = zeros(num_locations_to_perturb, length(gradient_steps));
legend_labels = cell(length(gradient_steps)+1,1);
legend_labels{end} = "Analytical";

% Perturb locations
rel_trajectory_array = swarme.rel_trajectory_array;

for delta_pose_ix = 1:length(gradient_steps)
    fprintf("Delta pose %d/%d\n", delta_pose_ix, length(gradient_steps))
    delta_pos = gradient_steps(delta_pose_ix);
    legend_labels{delta_pose_ix} = sprintf("Numerical, %d", delta_pos);
    for loc =1:num_locations_to_perturb
        fprintf("Location %d/%d\n", loc, num_locations_to_perturb)
        rel_trajectory_array_plus = swarme.rel_trajectory_array;
        rel_trajectory_array_minus = swarme.rel_trajectory_array;
        % sample_times = swarm.sample_times;
        rel_trajectory_array_plus(k_ix(loc), dir_ix(loc), n_ix(loc))  = rel_trajectory_array(k_ix(loc), dir_ix(loc), n_ix(loc)) + delta_pos; 
        rel_trajectory_array_minus(k_ix(loc), dir_ix(loc), n_ix(loc)) = rel_trajectory_array(k_ix(loc), dir_ix(loc), n_ix(loc)) - delta_pos; 
        try
            localswarm_plus = copy(swarme);
            localswarm_plus.test_only_reset_relative_trajectory(rel_trajectory_array_plus);
            [ops_plus,  opv_plus] = get_observable_points(ErosGravity, localswarm_plus, k_ix(loc), n_ix(loc), 0);
            % Find the point
            val_plus = NaN;
            for plus_pt_ix = 1:length(ops_plus)
                if ops_plus(plus_pt_ix) == pt_id(loc)
                    val_plus = opv_plus(plus_pt_ix);
                    if plus_pt_ix ~= pt_index(loc)
                        fprintf("\nStrange, point %d has changed location (was %d, is %d)", ops_plus(plus_pt_ix), pt_ix(loc), plus_pt_ix);
                    end
                    break
                end
            end
            if isnan(val_plus)
                error("Could not find point %d in agent %d's + observations at time %d", pt_id(loc), n_ix(loc), k_ix(loc))
            end
            
            localswarm_minus = copy(swarme);
            localswarm_minus.test_only_reset_relative_trajectory(rel_trajectory_array_minus);
            [ops_minus,  opv_minus] = get_observable_points(ErosGravity, localswarm_minus, k_ix(loc), n_ix(loc), 0);
            val_minus= NaN;
            for minus_pt_ix = 1:length(ops_minus)
                if ops_minus(minus_pt_ix) == pt_id(loc)
                    val_minus = opv_minus(minus_pt_ix);
                    if minus_pt_ix ~= pt_index(loc)
                        fprintf("\nStrange, point %d has changed location (was %d, is %d)", ops_minus(minus_pt_ix), pt_index(loc), minus_pt_ix);
%                     else
%                         fprintf("\nPoint %d found at same location (was %d: %d, is %d: %d)", ops_minus(minus_pt_ix), pt_index(loc), pt_id(loc), minus_pt_ix, ops_minus(minus_pt_ix));
                    end
                    break
                end
            end
            if isnan(val_minus)
                error("Could not find point %d in agent %d's - observations at time %d", pt_id(loc), n_ix(loc), k_ix(loc))
            end
            
            local_gradient = (val_plus - val_minus)/(2*abs(delta_pos));
        catch e
           local_gradient= NaN;
           warning("No gradient could be computed.")  % at point "+ sprintf(" %d", bandwidths_and_memories+perturbation))
           fprintf(1,'The identifier was:\n%s',e.identifier);
           fprintf(1,'There was an error! The message was:\n%s',e.message);
        end
        semi_analytical_gradient(loc,delta_pose_ix) = local_gradient;
    end
end

% Plotting
figure()
line_available_strokes = {'-', ':', '-.', '--'};
line_available_marks = {'.', 'o', 'x', '+', '*', 's', 'd', 'v', '^', '<', '>', 'p', 'h'};

for step_index = 1:length(gradient_steps)
    stroke_idx = mod(step_index, length(line_available_strokes))+1;
    mark_idx = mod(step_index, length(line_available_marks))+1;
    semilogy(abs(semi_analytical_gradient(:,step_index)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end

% Gradient WRT obs
step_index = length(gradient_steps)+1;
stroke_idx = mod(step_index, length(line_available_strokes))+1;
mark_idx = mod(step_index, length(line_available_marks))+1;
semilogy(abs(perturbed_location_analytical_gradients), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))

legend(legend_labels, 'location', 'best')
title("Gradient of reward wrt location (obs)")

%% gradient of overall reward with respect to sc positions (observations only)
num_locations_to_perturb = 15;
gradient_steps= [5, .5, .05, .005];

% Change locations, but keep bandwidths constant

swarmd = copy(swarm);
N = swarmd.get_num_spacecraft;
K = swarmd.get_num_timesteps;

bandwidth_model = @(x1,x2) min(bandwidth_parameters.reference_bandwidth * (bandwidth_parameters.reference_distance/norm(x2-x1,2))^2, bandwidth_parameters.max_bandwidth); 
% bandwidths_and_memories = zeros(K,N,N);

data_scaling_factor = 1e6;

[swarmd, goal] =  observation_and_communication_optimizer(ErosGravity, swarmd, bandwidth_model, data_scaling_factor);
spherical_asteroid_parameters.max_radius = ErosGravity.BodyModel.shape.maxRadius*1e3;
spherical_asteroid_parameters.min_radius = ErosGravity.BodyModel.shape.maxRadius*1e3;
[dk_dic, dk_dbandwidth, dbandwidth_dlocation, dk_dlocation_obs, dlocation_dic] = compute_integrated_gradient(swarmd, bandwidth_parameters, spherical_asteroid_parameters);

dk_dlocation_bw = dk_dbandwidth*dbandwidth_dlocation;


% Indices of the random locations, picked to be nonzero
k_ix = zeros(num_locations_to_perturb,1);
dir_ix = zeros(num_locations_to_perturb,1);
n_ix = zeros(num_locations_to_perturb,1);

randix = 0;
%randoffset = randi(len(dk_dlocation_obs)/2,1);
for k=1:K
    for n=1:N
        for l=1:3
            dk_dloc_ix = K*N*(l-1) + K*( n-1) + k;
            if abs(dk_dlocation_obs(dk_dloc_ix)) >100*eps
                dk_dlocation_obs(dk_dloc_ix);
                randix = randix+1;
                k_ix(randix) = k;
                n_ix(randix) = n;
                dir_ix(randix) = l;
            end
            if randix == num_locations_to_perturb
                break
            end
        end
        if randix == num_locations_to_perturb
            break
        end
    end
    if randix == num_locations_to_perturb
        break
    end
end

if randix<num_locations_to_perturb
    disp("I could not find enough non-zero-gradient locations!")
    k_ix(randix+1:num_locations_to_perturb)   = randi(K, num_locations_to_perturb-randix,1);
    dir_ix(randix+1:num_locations_to_perturb) = randi(3, num_locations_to_perturb-randix,1);
    n_ix(randix+1:num_locations_to_perturb)   = randi(N, num_locations_to_perturb-randix,1);
end

numerical_gradient = zeros(num_locations_to_perturb, length(gradient_steps));
gradient_reward_wrt_pos = cell(num_locations_to_perturb, length(gradient_steps)+1);
legend_labels = cell(length(gradient_steps)+1,1);

% Perturb locations
rel_trajectory_array = swarmd.rel_trajectory_array;
sample_times = swarmd.sample_times;

for delta_pose_ix = 1:length(gradient_steps)
    fprintf("Delta pose %d/%d\n", delta_pose_ix, length(gradient_steps))
    delta_pos = gradient_steps(delta_pose_ix);
    legend_labels{delta_pose_ix} = sprintf("Numerical, %d", delta_pos);
    parfor loc =1:num_locations_to_perturb
        fprintf("Location %d/%d\n", loc, num_locations_to_perturb)
        rel_trajectory_array_plus = swarmd.rel_trajectory_array;
        rel_trajectory_array_minus = swarmd.rel_trajectory_array;
        % sample_times = swarm.sample_times;
        rel_trajectory_array_plus(k_ix(loc), dir_ix(loc), n_ix(loc))  = rel_trajectory_array(k_ix(loc), dir_ix(loc), n_ix(loc)) + delta_pos; 
        rel_trajectory_array_minus(k_ix(loc), dir_ix(loc), n_ix(loc)) = rel_trajectory_array(k_ix(loc), dir_ix(loc), n_ix(loc)) - delta_pos; 

        % Compute unperturbed bandwidths 
        bandwidths_and_memories = zeros(K,N,N);

        for k=1:K-1
            for i=1:N
                for j=1:N
                    if j~=i
                        bandwidths_and_memories(k,i,j)= bandwidth_model(rel_trajectory_array(k,1:3,i), rel_trajectory_array(k,1:3,j))* (sample_times(k+1)-sample_times(k));
                    elseif ~isnan(swarm.Parameters.available_memory(i)) && ~isinf(swarm.Parameters.available_memory(i))
                        bandwidths_and_memories(k,i,j)= swarm.Parameters.available_memory(i);
                    else
                        bandwidths_and_memories(k,i,j)= max_memory;
                    end
                end
            end
        end

        % Compute the perturbed reward numerically
        try
            localswarm_plus = copy(swarmd);
            localswarm_plus.test_only_reset_relative_trajectory(rel_trajectory_array_plus);
            [~, goalplus]   = observation_and_communication_optimizer(ErosGravity, localswarm_plus, bandwidths_and_memories, data_scaling_factor);
            localswarm_minus = copy(swarmd);
            localswarm_minus.test_only_reset_relative_trajectory(rel_trajectory_array_minus);
            [~, goalminus]  = observation_and_communication_optimizer(ErosGravity, localswarm_minus, bandwidths_and_memories, data_scaling_factor);
            local_gradient = (goalplus - goalminus)/(2*abs(delta_pos));
        catch e
           local_gradient= NaN;
           warning("No gradient could be computed.")  % at point "+ sprintf(" %d", bandwidths_and_memories+perturbation))
           fprintf(1,'The identifier was:\n%s',e.identifier);
           fprintf(1,'There was an error! The message was:\n%s',e.message);
        end
        numerical_gradient(loc,delta_pose_ix) = local_gradient;
%         temp_gradient_bw_wrt_pos = (bandwidths_and_memories_plus-bandwidths_and_memories_minus)./(2*abs(delta_pos));
%         gradient_reward_wrt_pos{loc, delta_pose_ix} = temp_gradient_bw_wrt_pos(k_ix(loc),:,:);
    end
end

analytical_gradient = zeros(num_locations_to_perturb,1);

for loc=1:num_locations_to_perturb
    index = K*N*(dir_ix(loc)-1) + K*( n_ix(loc)-1) + k_ix(loc);
    analytical_gradient(loc,1) = dk_dlocation_obs(index);
end
legend_labels{end} = sprintf("Analytical (Obs)");

% Plotting
figure()
line_type = cell(length(gradient_steps)+1, 1);
line_available_strokes = {'-', ':', '-.', '--'};
line_available_marks = {'.', 'o', 'x', '+', '*', 's', 'd', 'v', '^', '<', '>', 'p', 'h'};

for step_index = 1:length(gradient_steps)
    stroke_idx = mod(step_index, length(line_available_strokes))+1;
    mark_idx = mod(step_index, length(line_available_marks))+1;
    semilogy(abs(numerical_gradient(:,step_index)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}));
    hold all
end

% Gradient WRT pos
step_index = length(gradient_steps)+2;
stroke_idx = mod(step_index, length(line_available_strokes))+1;
mark_idx = mod(step_index, length(line_available_marks))+1;
semilogy(abs(analytical_gradient(:,1)), strcat(line_available_strokes{stroke_idx},line_available_marks{mark_idx}))

legend(legend_labels, 'location', 'best')
title("Gradient of reward wrt location (obs)")

%% Gradient of location with respect to initial position

swarmb = copy(swarm);
N = swarmb.get_num_spacecraft;
K = swarmb.get_num_timesteps;

% Numerical gradient of initial location wrt location
num_ics_to_perturb = 20;
perturbation_magnitudes = [1, 0.1, 0.01,0.001, 0.0001];

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