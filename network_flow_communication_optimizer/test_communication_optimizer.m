%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%         Tests for the network flow communication optimizer              %
%          Asserts if any tests fail.                                     %
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

function [] = test_communication_optimizer()

% A set of simple tests to sanity check the behavior of the communication
% optimizer.
% Syntax: test_communication_optimizer()
% Asserts if any tests fail.

% Add base path with SpacecraftSwarm definition
addpath(genpath("../"));
% Add SBDT
addpath(genpath("../utilities/"));
constants = initialize_SBDT();

% Tolerance accepted in solver output
assert_tol = 1e-10;
close_enough = @(x,y) (abs(x-y)<=assert_tol);

% Load Eros
eros_sbdt = loadEros( constants, 1, 1, 3, 3 );
ErosGravity = SphericalHarmonicsGravityIntegrator_SBDT(eros_sbdt);
GM = eros_sbdt.gravity.gm * 1e9;  % Convert to m from km
%% Generate a simple test case

% Set a repeatable RNG
rng(0);

% Set number of spacecraft and time
sc_number = 3;
T = 1000;
t_stride=10;
time_vector = (1:t_stride:T);

% Set orbit radius
orbit_radius = 50000;

% Initialize spacecraft types (currently not in use)
sc_types = cell(1,sc_number);
for sc =1:sc_number
    sc_types{sc} = [1];
end

% Initialize spacecraft maximum memory
sc_max_memory = 9999 * ones(1,sc_number);

% Create swarm object
swarm = SpacecraftSwarm(time_vector, sc_types, sc_max_memory);

% Create orbits
sc_initial_state_array = zeros(sc_number, 6);
rotmat_x = @(angle) [1, 0, 0; 0, cos(angle), -sin(angle); 0, sin(angle), cos(angle)];
for sc = 1:sc_number
    sc_initial_state_array(sc, 1:3) = [orbit_radius; 0; 0];
    sc_orbital_vel = sqrt(GM/orbit_radius);    
    sc_initial_state_array(sc, 4:6) = rotmat_x((sc-1)/sc_number*pi)*[0; sc_orbital_vel; 0;];
end
swarm.integrate_trajectories(ErosGravity, sc_initial_state_array);

for sc = 1:sc_number
    swarm.Observation.flow(sc,:) = randn(size(time_vector))+5;
    swarm.Observation.priority(sc,:) = rand(size(time_vector));
end

assert(swarm.is_valid());

bandwidth_model = @(x1,x2) min(250000 * ((100000/norm(x2-x1,2))^2+ 0.2*randn()), 100*1e6);


[swarm] = communication_optimizer(swarm, bandwidth_model);
%% Test 1: no science, nothing is returned
T = 2;
t_stride=.5;
sc_number = 2;
orbit_radius = 50000;

time_vector=(1:t_stride:T);
sc_types = cell(1,sc_number);
for sc =1:sc_number
    sc_types{sc} = [1];
end
sc_max_memory = 9999*ones(sc_number,1);

swarm = SpacecraftSwarm(time_vector, sc_types, sc_max_memory);

% Create orbits
sc_initial_state_array = zeros(sc_number, 6);
rotmat_x = @(angle) [1, 0, 0; 0, cos(angle), -sin(angle); 0, sin(angle), cos(angle)];
for sc = 1:sc_number
    sc_initial_state_array(sc, 1:3) = [orbit_radius; 0; 0];
    sc_orbital_vel = sqrt(GM/orbit_radius);    
    sc_initial_state_array(sc, 4:6) = rotmat_x((sc-1)/sc_number*pi)*[0; sc_orbital_vel; 0;];
end
swarm.integrate_trajectories(ErosGravity, sc_initial_state_array);

% Science
for sc = 1:sc_number
    swarm.Observation.flow(sc,:) = 0.;
    swarm.Observation.priority(sc,:) = 1.;
end

[swarm] = communication_optimizer(swarm);

assert(swarm.is_valid());
assert(close_enough(sum(sum(swarm.Communication.effective_source_flow)),0))
assert(close_enough(sum(sum(sum(swarm.Communication.flow))),0))
%% Test 2: one unit of science delivered immediately
T = 3;
t_stride=1;
orbit_radius = 50000;
sc_number = 2;

time_vector=(1:t_stride:T);
sc_types = cell(1,sc_number);
for sc =1:sc_number
    sc_types{sc} = [1];
end
sc_max_memory = 9999*ones(sc_number,1);

swarm = SpacecraftSwarm(time_vector, sc_types, sc_max_memory);

% Create orbits
sc_initial_state_array = zeros(sc_number, 6);
rotmat_x = @(angle) [1, 0, 0; 0, cos(angle), -sin(angle); 0, sin(angle), cos(angle)];
for sc = 1:sc_number
    sc_initial_state_array(sc, 1:3) = [orbit_radius; 0; 0];
    sc_orbital_vel = sqrt(GM/orbit_radius);    
    sc_initial_state_array(sc, 4:6) = rotmat_x((sc-1)/sc_number*pi)*[0; sc_orbital_vel; 0;];
end
swarm.integrate_trajectories(ErosGravity, sc_initial_state_array);

% Science
for sc = 1:sc_number
    swarm.Observation.flow(sc,:) = 0.;
    swarm.Observation.priority(sc,:) = 1.;
end

swarm.Observation.flow(1,1) = 1;

[swarm] = communication_optimizer(swarm);

assert(swarm.is_valid());
assert(close_enough(swarm.Communication.effective_source_flow(1,1),1))
assert(close_enough(sum(sum(swarm.Communication.effective_source_flow)),1))
assert(close_enough(swarm.Communication.flow(2,1,2),1))

%% Test 3: two agents produce two units of science
T = 3;
t_stride=1;
orbit_radius = 50000;
sc_number = 3;

time_vector=(1:t_stride:T);
sc_types = cell(1,sc_number);
for sc =1:sc_number
    sc_types{sc} = [1];
end
sc_max_memory = 9999*ones(sc_number,1);

swarm = SpacecraftSwarm(time_vector, sc_types, sc_max_memory);

% Create orbits
sc_initial_state_array = zeros(sc_number, 6);
rotmat_x = @(angle) [1, 0, 0; 0, cos(angle), -sin(angle); 0, sin(angle), cos(angle)];
for sc = 1:sc_number
    sc_initial_state_array(sc, 1:3) = [orbit_radius; 0; 0];
    sc_orbital_vel = sqrt(GM/orbit_radius);    
    sc_initial_state_array(sc, 4:6) = rotmat_x((sc-1)/sc_number*pi)*[0; sc_orbital_vel; 0;];
end
swarm.integrate_trajectories(ErosGravity, sc_initial_state_array);

% Science
for sc = 1:sc_number
    swarm.Observation.flow(sc,:) = 0.;
    swarm.Observation.priority(sc,:) = 1.;
end
swarm.Observation.flow(1,1) = 1;
swarm.Observation.flow(2,1) = 1;

[swarm] = communication_optimizer(swarm);

assert(swarm.is_valid());
assert(close_enough(swarm.Communication.effective_source_flow(1,1),1))
assert(close_enough(swarm.Communication.effective_source_flow(2,1),1))
assert(close_enough(sum(sum(swarm.Communication.effective_source_flow)),2))
assert(close_enough(swarm.Communication.flow(2,1,3),1))
assert(close_enough(swarm.Communication.flow(2,2,3),1))

%% Test 4: bandwidth constraints
T = 3;
t_stride=1;
sc_number = 3;

time_vector=(1:t_stride:T);
sc_types = cell(1,sc_number);
for sc =1:sc_number
    sc_types{sc} = [1];
end
sc_max_memory = 9999*ones(sc_number,1);

swarm = SpacecraftSwarm(time_vector, sc_types, sc_max_memory);

% Create orbits
sc_initial_state_array = zeros(sc_number, 6);
rotmat_x = @(angle) [1, 0, 0; 0, cos(angle), -sin(angle); 0, sin(angle), cos(angle)];
for sc = 1:sc_number
    sc_initial_state_array(sc, 1:3) = [orbit_radius; 0; 0];
    sc_orbital_vel = sqrt(GM/orbit_radius);    
    sc_initial_state_array(sc, 4:6) = rotmat_x((sc-1)/sc_number*pi)*[0; sc_orbital_vel; 0;];
end
swarm.integrate_trajectories(ErosGravity, sc_initial_state_array);

% Science
for sc = 1:sc_number
    swarm.Observation.flow(sc,:) = 0.;
    swarm.Observation.priority(sc,:) = 1.;
end

bandwidth_model = @(x1,x2) .5;

swarm.Observation.flow(1,1) = 1;
swarm.Observation.flow(2,1) = 1;

[swarm] = communication_optimizer(swarm,bandwidth_model);

assert(swarm.is_valid());
assert(close_enough(swarm.Communication.effective_source_flow(1,1),.5))
assert(close_enough(swarm.Communication.effective_source_flow(2,1),.5))
assert(close_enough(sum(sum(swarm.Communication.effective_source_flow)),1))
assert(close_enough(swarm.Communication.flow(2,1,3),.5))
assert(close_enough(swarm.Communication.flow(2,2,3),.5))

%% Test 5a: priorities, priorities...part 1
T = 4;
t_stride=1;
sc_number = 4;

time_vector=(1:t_stride:T);
sc_types = cell(1,sc_number);
for sc =1:sc_number
    sc_types{sc} = [1];
end
sc_max_memory = 9999*ones(sc_number,1);

swarm = SpacecraftSwarm(time_vector, sc_types, sc_max_memory);

% % Create orbits
sc_initial_state_array = zeros(sc_number, 6);

sc_initial_state_array(1, 1:3) = [orbit_radius; 0; 0];
sc_orbital_vel = sqrt(GM/orbit_radius);    
sc_initial_state_array(1, 4:6) = 	[0; sc_orbital_vel; 0;];

sc_initial_state_array(2, 1:3) = [-orbit_radius; 0; 0];
sc_orbital_vel = sqrt(GM/orbit_radius);    
sc_initial_state_array(2, 4:6) = 	[0; -sc_orbital_vel; 0;];

sc_initial_state_array(3, 1:3) = [0; orbit_radius; 0];
sc_orbital_vel = sqrt(GM/orbit_radius);    
sc_initial_state_array(3, 4:6) = 	[sc_orbital_vel; 0; 0;];

sc_initial_state_array(4, 1:3) = [0; 2*orbit_radius; 0];
sc_orbital_vel = sqrt(GM/(2*orbit_radius));    
sc_initial_state_array(4, 4:6) = [sc_orbital_vel; 0; 0;];
% 1 and 2 are 2 radii apart, 1-3 and 2-3 are 1.41 radii apart, 3-4 is 1
% radius apart, 1-4 and 2-4 are 2.23 radii apart

swarm.integrate_trajectories(ErosGravity, sc_initial_state_array);

% Science
for sc = 1:sc_number
    swarm.Observation.flow(sc,:) = 0.;
    swarm.Observation.priority(sc,:) = 1.;
end


bandwidth_model = @(x1,x2) 1*(norm(x1-x2)<1.5*orbit_radius);
% 1 bandwidth between 1-3 and 2-3. 1 bandwidth between 3-4.
% 0 bandwidth between 1-2, 1-4, 2-4

swarm.Observation.flow(1,1) = 1;
swarm.Observation.flow(2,1) = 1;
swarm.Observation.priority(1,1) = 1;
swarm.Observation.priority(2,1) = .9;

[swarm] = communication_optimizer(swarm,bandwidth_model);

assert(swarm.is_valid());
assert(close_enough(swarm.Communication.effective_source_flow(1,1),1))
assert(close_enough(swarm.Communication.effective_source_flow(2,1),0))
assert(close_enough(sum(sum(swarm.Communication.effective_source_flow)),1))
assert(close_enough(swarm.Communication.flow(2,1,3),1))
assert(close_enough(swarm.Communication.flow(3,3,4),1))
% assert(network_flows(2,2,3)==.5)

%% Test 5b: priorities, priorities...part 2
T = 4;
t_stride=1;
sc_number = 4;


time_vector=(1:t_stride:T);
sc_types = cell(1,sc_number);
for sc =1:sc_number
    sc_types{sc} = [1];
end
sc_max_memory = 9999*ones(sc_number,1);

swarm = SpacecraftSwarm(time_vector, sc_types, sc_max_memory);

% % Create orbits
sc_initial_state_array = zeros(sc_number, 6);

sc_initial_state_array(1, 1:3) = [orbit_radius; 0; 0];
sc_orbital_vel = sqrt(GM/orbit_radius);    
sc_initial_state_array(1, 4:6) = [0; sc_orbital_vel; 0;];

sc_initial_state_array(2, 1:3) = [-orbit_radius; 0; 0];
sc_orbital_vel = sqrt(GM/orbit_radius);    
sc_initial_state_array(2, 4:6) = [0; -sc_orbital_vel; 0;];

sc_initial_state_array(3, 1:3) = [0; orbit_radius; 0];
sc_orbital_vel = sqrt(GM/orbit_radius);    
sc_initial_state_array(3, 4:6) = [sc_orbital_vel; 0; 0;];

sc_initial_state_array(4, 1:3) = [0; 2*orbit_radius; 0];
sc_orbital_vel = sqrt(GM/(2*orbit_radius));    
sc_initial_state_array(4, 4:6) = [sc_orbital_vel; 0; 0;];
% 1 and 2 are 2 radii apart, 1-3 and 2-3 are 1.41 radii apart, 3-4 is 1
% radius apart, 1-4 and 2-4 are 2.23 radii apart

swarm.integrate_trajectories(ErosGravity, sc_initial_state_array);

% Science
for sc = 1:sc_number
    swarm.Observation.flow(sc,:) = 0.;
    swarm.Observation.priority(sc,:) = 1.;
end


bandwidth_model = @(x1,x2) 1*(norm(x1-x2)<1.5*orbit_radius);
% 1 bandwidth between 1-3 and 2-3. 1 bandwidth between 3-4.
% 0 bandwidth between 1-2, 1-4, 2-4

swarm.Observation.flow(1,1) = 1;
swarm.Observation.flow(2,1) = 1;
swarm.Observation.priority(1,1) = 1;
swarm.Observation.priority(2,1) = 1.9;

[swarm] = communication_optimizer(swarm,bandwidth_model);

assert(swarm.is_valid());
assert(close_enough(swarm.Communication.effective_source_flow(1,1),0))
assert(close_enough(swarm.Communication.effective_source_flow(2,1),1))
assert(close_enough(sum(sum(swarm.Communication.effective_source_flow)),1))
assert(close_enough(swarm.Communication.flow(2,2,3),1))
assert(close_enough(swarm.Communication.flow(3,3,4),1))
% assert(network_flows(2,2,3)==.5)

%% Test 6: memorize
orbit_radius = 50000;
sc_number = 2;
sc_types = cell(1,sc_number);
for sc =1:sc_number
    sc_types{sc} = [1];
end

orbital_period = 2*pi*sqrt(orbit_radius^3/GM);
T=4/8*orbital_period;
t_stride = T/4;
time_vector=(1:t_stride:T);

assert(length(time_vector) == 4)

sc_max_memory = 1*ones(sc_number,1);

swarm = SpacecraftSwarm(time_vector, sc_types, sc_max_memory);

% % Create orbits
sc_initial_state_array = zeros(sc_number, 6);

sc_initial_state_array(1, 1:3) = [orbit_radius; 0; 0];
sc_orbital_vel = sqrt(GM/orbit_radius);    
sc_initial_state_array(1, 4:6) = [0; sc_orbital_vel; 0;];

sc_initial_state_array(2, 1:3) = [-orbit_radius; 0; 0];
sc_orbital_vel = sqrt(GM/orbit_radius);    
sc_initial_state_array(2, 4:6) = [0; sc_orbital_vel; 0;];

% Spacecraft start at opposite sides of Eros on coplanar circular orbits
% with same h. At time 3, they pass very close to each other.

swarm.integrate_trajectories(ErosGravity, sc_initial_state_array);

% close all
% figure()
% plot3(swarm.abs_trajectory_array(:,1,1),swarm.abs_trajectory_array(:,2,1),swarm.abs_trajectory_array(:,3,1))
% hold all
% plot3(swarm.abs_trajectory_array(:,1,2),swarm.abs_trajectory_array(:,2,2),swarm.abs_trajectory_array(:,3,2))
% axis equal


% Science
for sc = 1:sc_number
    swarm.Observation.flow(sc,:) = 0.;
    swarm.Observation.priority(sc,:) = 1.;
end

swarm.Observation.flow(1,1) = 1;

bandwidth_model = @(x1,x2) 1*(norm(x1-x2)<1e4);
% 1 bandwidth at time 3. 


assert(swarm.is_valid());
[swarm] = communication_optimizer(swarm,bandwidth_model);

assert(close_enough(sum(sum(swarm.Communication.effective_source_flow)),1))
assert(close_enough(swarm.Communication.flow(3,1,2),1))

disp("Tests succeeded!")
return
