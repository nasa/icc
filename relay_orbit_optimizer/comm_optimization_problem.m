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
function [goal, spacecraft, ErosGravity, time_bounds, GM, dgoal_dic] = comm_optimization_problem(relay_initial_condition,time_bounds,plot_flag)
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

global SBDT_PATH
if isempty(SBDT_PATH)
    fprintf("ERROR: specify SBDT path as a global variable. Returning...\n")
    return
end
userModelsPath = strcat(SBDT_PATH,'/ExampleUserModels');
constantsModel = 1;
addpath(strcat(SBDT_PATH,'/Startup'));
global constants
constants = addSBDT(SBDT_PATH, userModelsPath, constantsModel);

eros_sbdt = loadEros( constants, 1, 1, 3, 3 );
ErosGravity = SphericalHarmonicsGravityIntegrator_SBDT(eros_sbdt);
GM = eros_sbdt.gravity.gm * 1e9;  % Convert to m from km

% Initialize a base case
sc_location = [30*1e3;0;0];
sc_radius = norm(sc_location);
sc_orbital_vel = sqrt(GM/sc_radius);
sc_vel = [0; sc_orbital_vel*sqrt(2)/2; sc_orbital_vel*sqrt(2)/2];
sc_state = [sc_location; sc_vel];
% [time_sc1,abs_traj_sc1,rel_traj_sc1] = ErosGravity.integrate(time_bounds,sc_state);
[time_sc1,abs_traj_sc1,state_transition_matrix_sc1] = ErosGravity.integrate_absolute(time_bounds,sc_state);

sc_location = [30*1e3;0;0];
sc_radius = norm(sc_location);
sc_orbital_vel = sqrt(GM/sc_radius);
sc_vel = [0; sc_orbital_vel; 0];
sc_state = [sc_location; sc_vel];
% [time_sc2,abs_traj_sc2,rel_traj_sc2] = ErosGravity.integrate(time_bounds,sc_state);
[time_sc2,abs_traj_sc2,state_transition_matrix_sc2] = ErosGravity.integrate_absolute(time_bounds,sc_state);

sc_state = relay_initial_condition;
% [time_re,abs_traj_re,rel_traj_re] = ErosGravity.integrate(time_bounds,sc_state);
[time_re,abs_traj_re,state_transition_matrix_re] = ErosGravity.integrate_absolute(time_bounds,sc_state);


sc_location = [100*1e3;0;0];
sc_radius = norm(sc_location);
sc_orbital_vel = sqrt(GM/sc_radius);
sc_vel = [0; sc_orbital_vel; 0];
sc_state = [sc_location; sc_vel];

% [time_ca,abs_traj_ca,rel_traj_ca] = ErosGravity.integrate(time_bounds,sc_state);
[time_ca,abs_traj_ca,state_transition_matrix_ca] = ErosGravity.integrate_absolute(time_bounds,sc_state);


% Plot

if plot_flag
    AbsoluteTrajPlot= figure();
%     RelativeTrajPlot=figure();
    ErosGravity.plot_absolute_traj(time_bounds,abs_traj_sc1, false, AbsoluteTrajPlot)
    ErosGravity.plot_absolute_traj(time_bounds,abs_traj_sc2, false, AbsoluteTrajPlot)
    ErosGravity.plot_absolute_traj(time_bounds,abs_traj_re, false, AbsoluteTrajPlot)
    ErosGravity.plot_absolute_traj(time_bounds,abs_traj_ca, false, AbsoluteTrajPlot)

%     ErosGravity.plot_relative_traj(time_bounds,rel_traj_sc1, 1, RelativeTrajPlot)
%     ErosGravity.plot_relative_traj(time_bounds,rel_traj_sc2, 1, RelativeTrajPlot)
%     ErosGravity.plot_relative_traj(time_bounds,rel_traj_re, 1, RelativeTrajPlot)
%     ErosGravity.plot_relative_traj(time_bounds,rel_traj_ca, 1, RelativeTrajPlot)
end

%% Comm optimization
% Inputs:
% * spacecraft, a struct with four fields.
% - time is a vector of length K. It contains the time at which the s/c
% orbits are sampled.
% - locations is a list of N vectors of length K.
% orbits{i} is the vector of locations of spacecraft i.
% orbits{i}(:, k) is the location of s/c i at time time(k).
% - science is a matrix of size N-by-K.
% science(i,k) is the amount of science (in bits) produced by
% s/c k at time i.
% - priority is a matrix of size N-by-K.
% priority(i,k) is the value of one bit of science produced by
% s/c k at time i.

spacecraft = struct();
spacecraft.time = time_bounds;
spacecraft.orbits = cell(4,1);
spacecraft.orbits{1} = abs_traj_sc1(:,1:3)';
spacecraft.orbits{2} = abs_traj_sc2(:,1:3)';
spacecraft.orbits{3} = abs_traj_re(:,1:3)';
spacecraft.orbits{4} = abs_traj_ca(:,1:3)';
% State transition matrix is not used by communication optimizer - but this
% is a good place to store it
spacecraft.state_transition_matrix{1} = state_transition_matrix_sc1;
spacecraft.state_transition_matrix{2} = state_transition_matrix_sc2;
spacecraft.state_transition_matrix{3} = state_transition_matrix_re;
spacecraft.state_transition_matrix{4} = state_transition_matrix_ca;
%
spacecraft.velocities{1} = abs_traj_sc1(:,4:6)';
spacecraft.velocities{2} = abs_traj_sc2(:,4:6)';
spacecraft.velocities{3} = abs_traj_re(:,4:6)';
spacecraft.velocities{4} = abs_traj_ca(:,4:6)';
spacecraft.science = zeros(4,length(time_bounds));
spacecraft.science(1:2,:) = 1e9; %1Gb/time step
spacecraft.priority = zeros(4,length(time_bounds));
spacecraft.priority(1:2,:) = 1;
spacecraft.memory = ones(4,1)*1e10; %1TB
spacecraft.memory(end) = 0; % Force to store delivered science right away
record_video = plot_flag;
videoname = ['comm_optimization_problem_',datestr(now,'yyyymmdd_HHMMSS'),'.avi'];

reference_bandwidth = 250000;
reference_distance = 100000;
max_bandwidth = 100*1e6;

bandwidth_model = @(x1, x2) quadratic_comm_model(x1, x2, reference_distance,reference_bandwidth,max_bandwidth);
%bandwidth_model = @(x1,x2) min(reference_bandwidth * (reference_distance/norm(x2-x1,2))^2, max_bandwidth)

[flows, effective_science, delivered_science, bandwidths, dual_bandwidths_and_memory] = communication_optimizer(spacecraft, bandwidth_model);

goal = sum(delivered_science);
%goal2 = sum(sum(effective_science))
fraction_of_possible = goal/sum(sum(spacecraft.science));

%% Compute gradient wrt initial conditions
dgoal_dic = compute_gradient(spacecraft, dual_bandwidths_and_memory, reference_distance, reference_bandwidth, max_bandwidth);


%% Plotting
if plot_flag
    plot_communications(spacecraft, flows, effective_science, delivered_science, bandwidths, dual_bandwidths_and_memory, ErosGravity,record_video, videoname )
end

end

function [bandwidth] = quadratic_comm_model(x1, x2, reference_distance,reference_bandwidth,max_bandwidth)
    bandwidth = min(max_bandwidth, reference_bandwidth*(reference_distance/norm(x2-x1,2))^2);
end