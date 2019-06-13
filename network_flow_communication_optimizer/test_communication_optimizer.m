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

% Tolerance accepted in solver output
assert_tol = 1e-10;
close_enough = @(x,y) (abs(x-y)<=assert_tol);
%% Generate a simple test case

rng(0);

sc_number = 3;
T = 1000;
t_stride=10;
orbit_radius = 50000;

spacecraft = struct();
time = (1:t_stride:T);
sc_loc = cell(sc_number,1);
sc_science = zeros(sc_number,length(time));
sc_priority = zeros(sc_number,length(time));
sc_memory = zeros(sc_number,1);


planar_orbit = [orbit_radius.*cos(time/max(time)*2*pi); orbit_radius.*sin(time/max(time)*2*pi); 0*time];
rotmat_x = @(angle) [1, 0, 0; 0, cos(angle), -sin(angle); 0, sin(angle), cos(angle)];

for sc=1:sc_number
    sc_loc{sc} = rotmat_x((sc-1)/sc_number*pi)*planar_orbit;
    %plot3(sc_loc{sc}(1,:), sc_loc{sc}(2,:), sc_loc{sc}(3,:))
    %hold all
    sc_science(sc,:) = randn(size(time))+5;
    sc_priority(sc,:) = rand(size(time));
    sc_memory(sc) = 9999;
end

bandwidth_model = @(x1,x2) min(250000 * ((100000/norm(x2-x1,2))^2+ 0.2*randn()), 100*1e6);

spacecraft.time = time;
spacecraft.orbits = sc_loc;
spacecraft.science = sc_science;
spacecraft.priority = sc_priority;
spacecraft.memory = sc_memory;

[network_flows, effective_science, delivered_science, bandwidths, dual_bandwidth_and_memory] = communication_optimizer(spacecraft, bandwidth_model);
%% Test 1: no science, nothing is returned
T = 2;
t_stride=1;
sc_number = 2;
orbit_radius = 50000;

spacecraft = struct();
time = (1:t_stride:T);
sc_loc = cell(sc_number,1);
sc_science = zeros(sc_number,length(time));
sc_priority = zeros(sc_number,length(time));
sc_memory = zeros(sc_number,1);


planar_orbit = [orbit_radius.*cos(time/max(time)*2*pi); orbit_radius.*sin(time/max(time)*2*pi); 0*time];
rotmat_x = @(angle) [1, 0, 0; 0, cos(angle), -sin(angle); 0, sin(angle), cos(angle)];

for sc=1:sc_number
    sc_loc{sc} = rotmat_x((sc-1)/sc_number*pi)*planar_orbit;
    sc_science(sc,:) = 0.0;
    sc_priority(sc,:) = 1;
    sc_memory(sc) = 9999;
end
sc_science(1,1) = 0;

spacecraft.time = time;
spacecraft.orbits = sc_loc;
spacecraft.science = sc_science;
spacecraft.priority = sc_priority;
spacecraft.memory = sc_memory;

[network_flows, effective_science, delivered_science] = communication_optimizer(spacecraft);

assert(close_enough(sum(sum(effective_science)),0))
assert(close_enough(sum(sum(sum(network_flows))),0))
%% Test 2: one unit of science delivered immediately
T = 3;
t_stride=1;
orbit_radius = 50000;
sc_number = 2;

spacecraft = struct();
time = (1:t_stride:T);
sc_loc = cell(sc_number,1);
sc_science = zeros(sc_number,length(time));
sc_priority = zeros(sc_number,length(time));
sc_memory = zeros(sc_number,1);


planar_orbit = [orbit_radius.*cos(time/max(time)*2*pi); orbit_radius.*sin(time/max(time)*2*pi); 0*time];
rotmat_x = @(angle) [1, 0, 0; 0, cos(angle), -sin(angle); 0, sin(angle), cos(angle)];

for sc=1:sc_number
    sc_loc{sc} = rotmat_x((sc-1)/sc_number*pi)*planar_orbit;
    sc_science(sc,:) = 0.0;
    sc_priority(sc,:) = 1;
    sc_memory(sc) = 9999;
end
sc_science(1,1) = 1;

spacecraft.time = time;
spacecraft.orbits = sc_loc;
spacecraft.science = sc_science;
spacecraft.priority = sc_priority;
spacecraft.memory = sc_memory;

[network_flows, effective_science, delivered_science] = communication_optimizer(spacecraft);
assert(close_enough(effective_science(1,1),1))
assert(close_enough(sum(sum(effective_science)),1))
assert(close_enough(network_flows(2,1,2),1))


%% Test 3: two agents produce two units of science
T = 3;
t_stride=1;
orbit_radius = 50000;
sc_number = 3;

spacecraft = struct();
time = (1:t_stride:T);
sc_loc = cell(sc_number,1);
sc_science = zeros(sc_number,length(time));
sc_priority = zeros(sc_number,length(time));
sc_memory = zeros(sc_number,1);


planar_orbit = [orbit_radius.*cos(time/max(time)*2*pi); orbit_radius.*sin(time/max(time)*2*pi); 0*time];
rotmat_x = @(angle) [1, 0, 0; 0, cos(angle), -sin(angle); 0, sin(angle), cos(angle)];

for sc=1:sc_number
    sc_loc{sc} = rotmat_x((sc-1)/sc_number*pi)*planar_orbit;
    sc_science(sc,:) = 0.0;
    sc_priority(sc,:) = 1;
    sc_memory(sc) = 9999;
end
sc_science(1,1) = 1;
sc_science(2,1) = 1;

spacecraft.time = time;
spacecraft.orbits = sc_loc;
spacecraft.science = sc_science;
spacecraft.priority = sc_priority;
spacecraft.memory = sc_memory;

[network_flows, effective_science, delivered_science] = communication_optimizer(spacecraft);
assert(close_enough(effective_science(1,1),1))
assert(close_enough(effective_science(2,1),1))
assert(close_enough(sum(sum(effective_science)),2))
assert(close_enough(network_flows(2,1,3),1))
assert(close_enough(network_flows(2,2,3),1))

%% Test 4: bandwidth constraints
T = 3;
t_stride=1;
sc_number = 3;

spacecraft = struct();
time = (1:t_stride:T);
sc_loc = cell(sc_number,1);
sc_science = zeros(sc_number,length(time));
sc_priority = zeros(sc_number,length(time));
sc_memory = zeros(sc_number,1);

for sc=1:sc_number
    sc_science(sc,:) = 0.0;
    sc_priority(sc,:) = 1;
    sc_memory(sc) = 9999;
end
sc_loc{1} = [1 1;0 0; 0 0;];
sc_loc{2} = [1 1;0 0; 0 0;];
sc_loc{3} = [1 1;0 0; 0 0;];

bandwidth_model = @(x1,x2) .5;

sc_science(1,1) = 1;
sc_science(2,1) = 1;

spacecraft.time = time;
spacecraft.orbits = sc_loc;
spacecraft.science = sc_science;
spacecraft.priority = sc_priority;
spacecraft.memory = sc_memory;

[network_flows, effective_science, delivered_science] = communication_optimizer(spacecraft, bandwidth_model);
assert(close_enough(effective_science(1,1),.5))
assert(close_enough(effective_science(2,1),.5))
assert(close_enough(sum(sum(effective_science)),1))
assert(close_enough(network_flows(2,1,3),.5))
assert(close_enough(network_flows(2,2,3),.5))

%% Test 5a: priorities, priorities...part 1
T = 4;
t_stride=1;
sc_number = 4;

spacecraft = struct();
time = (1:t_stride:T);
sc_loc = cell(sc_number,1);
sc_science = zeros(sc_number,length(time));
sc_priority = zeros(sc_number,length(time));
sc_memory = zeros(sc_number,1);

for sc=1:sc_number
    sc_science(sc,:) = 0.0;
    sc_priority(sc,:) = 1;
    sc_memory(sc) = 9999;
end
sc_loc{1} = [1 1 1 1; 0 0 0 0; 0 0 0 0;];
sc_loc{2} = [1 1 1 1; 0 0 0 0; 0 0 0 0;];
sc_loc{3} = [0 0 0 0; 0 0 0 0; 0 0 0 0;];
sc_loc{4} = [1 1 1 1; 0 0 0 0; 0 0 0 0;];

bandwidth_model = @(x1,x2) abs(x1(1) - x2(1));
% 1 bandwidth between 1-3 and 2-3. 1 bandwidth between 3-4. 

sc_science(1,1) = 1;
sc_science(2,1) = 1;
sc_priority(1,1) = 1;
sc_priority(2,1) = .9;

spacecraft.time = time;
spacecraft.orbits = sc_loc;
spacecraft.science = sc_science;
spacecraft.priority = sc_priority;
spacecraft.memory = sc_memory;

[network_flows, effective_science, delivered_science, bandwidths] = communication_optimizer(spacecraft, bandwidth_model);
assert(close_enough(effective_science(1,1),1))
assert(close_enough(effective_science(2,1),0))
assert(close_enough(sum(sum(effective_science)),1))
assert(close_enough(network_flows(2,1,3),1))
assert(close_enough(network_flows(3,3,4),1))
% assert(network_flows(2,2,3)==.5)

%% Test 5b: priorities, priorities...part 2
T = 4;
t_stride=1;
sc_number = 4;

spacecraft = struct();
time = (1:t_stride:T);
sc_loc = cell(sc_number,1);
sc_science = zeros(sc_number,length(time));
sc_priority = zeros(sc_number,length(time));
sc_memory = zeros(sc_number,1);

for sc=1:sc_number
    sc_science(sc,:) = 0.0;
    sc_priority(sc,:) = 1;
    sc_memory(sc) = 9999;
end
sc_loc{1} = [1 1 1 1; 0 0 0 0; 0 0 0 0;];
sc_loc{2} = [1 1 1 1; 0 0 0 0; 0 0 0 0;];
sc_loc{3} = [0 0 0 0; 0 0 0 0; 0 0 0 0;];
sc_loc{4} = [1 1 1 1; 0 0 0 0; 0 0 0 0;];

bandwidth_model = @(x1,x2) abs(x1(1) - x2(1));
% 1 bandwidth between 1-3 and 2-3. 1 bandwidth between 3-4. 

sc_science(1,1) = 1;
sc_science(2,1) = 1;
sc_priority(1,1) = 1;
sc_priority(2,1) = 1.9;

spacecraft.time = time;
spacecraft.orbits = sc_loc;
spacecraft.science = sc_science;
spacecraft.priority = sc_priority;
spacecraft.memory = sc_memory;

[network_flows, effective_science, delivered_science, bandwidths] = communication_optimizer(spacecraft, bandwidth_model);
assert(close_enough(effective_science(1,1),0))
assert(close_enough(effective_science(2,1),1))
assert(close_enough(sum(sum(effective_science)),1))
assert(close_enough(network_flows(2,2,3),1))
assert(close_enough(network_flows(3,3,4),1))
% assert(network_flows(2,2,3)==.5)

%% Test 6: memorize
T = 4;
t_stride=1;
orbit_radius = 50000;
sc_number = 2;

spacecraft = struct();
time = (1:t_stride:T);
sc_loc = cell(sc_number,1);
sc_science = zeros(sc_number,length(time));
sc_priority = zeros(sc_number,length(time));
sc_memory = zeros(sc_number,1);


planar_orbit = [orbit_radius.*cos(time/max(time)*2*pi); orbit_radius.*sin(time/max(time)*2*pi); 0*time];
rotmat_x = @(angle) [1, 0, 0; 0, cos(angle), -sin(angle); 0, sin(angle), cos(angle)];

for sc=1:sc_number
    sc_science(sc,:) = 0.0;
    sc_priority(sc,:) = 1;
    sc_memory(sc) = 1;
end
sc_loc{1} = [1 1 1 1; 0 0 0 0; 0 0 0 0;];
sc_loc{2} = [1 1 0 1; 0 0 0 0; 0 0 0 0;];

bandwidth_model = @(x1,x2) abs(x1(1) - x2(1));
% 1 bandwidth at time 3. 

sc_science(1,1) = 1;

spacecraft.time = time;
spacecraft.orbits = sc_loc;
spacecraft.science = sc_science;
spacecraft.priority = sc_priority;
spacecraft.memory = sc_memory;

[network_flows, effective_science, delivered_science, bandwidths] = communication_optimizer(spacecraft, bandwidth_model);
% assert(effective_science(1,1)==1)
assert(close_enough(sum(sum(effective_science)),1))
assert(close_enough(network_flows(3,1,2),1))
% assert(abs(network_flows(3,3,4)-1)<assert_tol)
% assert(network_flows(2,2,3)==.5)

return
