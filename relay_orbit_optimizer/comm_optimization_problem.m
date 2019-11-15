%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%         Initializes an example relay optimization problem.              %
%         For use with relay_optimization_driver.                         %  
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2019 by California Institute of Technology.  ALL RIGHTS RESERVED. %
% United  States  Government  sponsorship  acknowledged.   Any commercial use %
% must   be  negotiated  with  the  Office  of  Technology  Transfer  at  the %
% California Institute of Technology.                                         %
%                                                                             %
% This software may be subject to  U.S. export control laws  and regulations. %
% By accepting this document,  the user agrees to comply  with all applicable %
% U.S. export laws and regulations.  User  has the responsibility  to  obtain %
% export  licenses,  or  other  export  authority  as may be required  before %
% exporting  such  information  to  foreign  countries or providing access to %
% foreign persons.                                                            %
%                                                                             %
% This  software  is a copy  and  may not be current.  The latest  version is %
% maintained by and may be obtained from the Mobility  and  Robotics  Sytstem %
% Section (347) at the Jet  Propulsion  Laboratory.   Suggestions and patches %
% are welcome and should be sent to the software's maintainer.                %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create four orbits - two science, one relay, one carrier. Optimize comms.
function [goal, swarm, ErosGravity, GM, dgoal_dic] = comm_optimization_problem(relay_initial_condition,time_bounds,plot_flag)
if nargin<3
    plot_flag = false;
end
if nargin<2
   time_bounds = [0:300:86400];
end
if nargin<1
    relay_orbital_parameters = [50*1e3,0,-pi/4,0,0,0];
    GM = 4.462754720040000e+05;
    [sc_location,sc_vel]=op2rv(...
    relay_orbital_parameters(1),relay_orbital_parameters(2),relay_orbital_parameters(3),...
    relay_orbital_parameters(4),relay_orbital_parameters(5),relay_orbital_parameters(6),...
    GM);
    relay_initial_condition = [sc_location; sc_vel];
end
addpath(genpath('../small_body_dynamics'))
addpath('../network_flow_communication_optimizer')
addpath('../relay_orbit_optimizer')
addpath(genpath('../utilities'))
addpath('../')

global constants
constants = initialize_SBDT();

eros_sbdt = loadEros( constants, 1, 1, 3, 3 );
ErosGravity = SphericalHarmonicsGravityIntegrator_SBDT(eros_sbdt);
GM = eros_sbdt.gravity.gm * 1e9;  % Convert to m from km

% Initialize a base case

n_spacecraft = 4;
sc_types = cell(n_spacecraft,1);
% max_memory = inf(n_spacecraft,1);
max_memory = ones(n_spacecraft,1)*1e10; %1TB
max_memory(end) = 0; % Force to store delivered science right away

swarm = SpacecraftSwarm(time_bounds, sc_types, max_memory);

sc_initial_state_array = zeros(n_spacecraft,6);
% SC 1
sc_location = [30*1e3;0;0];
sc_orbital_vel = sqrt(GM/norm(sc_location));
sc_vel = [0; sc_orbital_vel*sqrt(2)/2; sc_orbital_vel*sqrt(2)/2];
sc_initial_state_array(1,:) = [sc_location; sc_vel];
% SC 2
sc_location = [30*1e3;0;0];
sc_orbital_vel = sqrt(GM/norm(sc_location));
sc_vel = [0; sc_orbital_vel; 0];
sc_initial_state_array(2,:) = [sc_location; sc_vel];
% Relay
sc_initial_state_array(3,:) = relay_initial_condition;
% Carrier
sc_location = [100*1e3;0;0];
sc_orbital_vel = sqrt(GM/norm(sc_location));
sc_vel = [0; sc_orbital_vel; 0];
sc_initial_state_array(4,:) = [sc_location; sc_vel];


swarm.integrate_trajectories(ErosGravity, sc_initial_state_array);

% Plot

if plot_flag
    AbsoluteTrajPlot= figure();
    RelativeTrajPlot=figure();
    
    ErosGravity.plot_absolute_traj(swarm.sample_times,swarm.abs_trajectory_array(:,:,1), false, AbsoluteTrajPlot)
    ErosGravity.plot_absolute_traj(time_bounds,swarm.abs_trajectory_array(:,:,2), false, AbsoluteTrajPlot)
    ErosGravity.plot_absolute_traj(time_bounds,swarm.abs_trajectory_array(:,:,3), false, AbsoluteTrajPlot)
    ErosGravity.plot_absolute_traj(time_bounds,swarm.abs_trajectory_array(:,:,4), false, AbsoluteTrajPlot)

    ErosGravity.plot_relative_traj(swarm.sample_times,swarm.rel_trajectory_array(:,:,1), false, RelativeTrajPlot)
    ErosGravity.plot_relative_traj(time_bounds,swarm.rel_trajectory_array(:,:,2), false, RelativeTrajPlot)
    ErosGravity.plot_relative_traj(time_bounds,swarm.rel_trajectory_array(:,:,3), false, RelativeTrajPlot)
    ErosGravity.plot_relative_traj(time_bounds,swarm.rel_trajectory_array(:,:,4), false, RelativeTrajPlot)

end

%% Comm optimization

swarm.Observation.flow = zeros(n_spacecraft,length(time_bounds));
swarm.Observation.flow(1:2,:) = 1e9; %1Gb/time step

swarm.Observation.priority = zeros(n_spacecraft,length(time_bounds));
swarm.Observation.priority(1:2,:) = 1;

record_video = plot_flag;
videoname = ['comm_optimization_problem_',datestr(now,'yyyymmdd_HHMMSS'),'.avi'];

bandwidth_parameters.reference_bandwidth = 250000;
bandwidth_parameters.reference_distance = 100000;
bandwidth_parameters.max_bandwidth = 100*1e6;

bandwidth_model = @(x1, x2) quadratic_comm_model(x1, x2, bandwidth_parameters);
%bandwidth_model = @(x1,x2) min(reference_bandwidth * (reference_distance/norm(x2-x1,2))^2, max_bandwidth)

% [flows, effective_science, delivered_science, bandwidths, dual_bandwidths_and_memory] = communication_optimizer(spacecraft, bandwidth_model);

[swarm] = communication_optimizer(swarm, bandwidth_model);

goal = sum(sum(swarm.Observation.priority.*swarm.Communication.effective_source_flow));

% goal = sum(swarm.Communication.delivered_science);
%goal2 = sum(sum(effective_science))
fraction_of_possible = goal/sum(sum(swarm.Observation.priority.*swarm.Observation.flow));

%% Compute gradient wrt initial conditions
dgoal_dic = compute_gradient(swarm, bandwidth_parameters);

%% Plotting
if plot_flag
    plot_communications(swarm, ErosGravity,record_video, videoname )
end

end