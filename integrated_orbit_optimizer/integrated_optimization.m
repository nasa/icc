%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%       Fmincon-based optimizer for relay and observation orbits.         %
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

function [swarm] = integrated_optimization(swarm, gravity_model, bandwidth_parameters, max_optimization_time)
% Optimizes all orbits by using a gradient-based trust region algorithm (fmincon).
% For a given set of orbits, observations and relays are optimized by
% calling observation_and_communication_optimizer
% Syntax: [swarm] = integrated_optimization(swarm, gravity_model, bandwidth_parameters. max_optimization_time)
% Inputs:
%  - swarm, a SpacecraftSwarm object
%  - gravity_model, a GravityField object as used by SpacecraftSwarm
%  - bandwidth_parameters, a struct with three parameters:
%    - reference_bandwidth, the bandwidth at a given reference distance
%    - reference_distance, the reference distanc3e
%    - max_bandwidth, the maximum available bandwidth at close range
%     The model assumes a quadratic bandwidth model
%  - max_optimization_time, the maximum time allocated to fmincon.
% Output:
%  - swarm, a SpacecraftSwarm object with updated orbits.

if nargin<3
    max_optimization_time = inf;
end

if nargin<2
    bandwidth_parameters.reference_bandwidth = 250000;
    bandwidth_parameters.reference_distance = 100000;
    bandwidth_parameters.max_bandwidth = 100*1e6;
end

addpath(genpath('../utilities/'))
initialize_SBDT();

% Empty inputs to the optimizer
ub = [];
lb = [];
options = optimoptions('fmincon',... %     'SpecifyObjectiveGradient',true,...
    'Display', 'Iter',...
    'CheckGradients', true, ...
    'UseParallel', true, ...
    'PlotFcn', 'optimplotstepsize');
A = [];
b = [];
Aeq = [];
beq = [];

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
[goal] = integrated_optimization_cost_function(swarm,initial_conditions, optvar_scaling_factor, gravity_model, bandwidth_parameters);
% if (isnan(goal) || any(isnan(gradient)))
if (isnan(goal))
    error('ERROR: initial location is infeasible. fmincon will crash.')
end

% Proper cost function
fun = @(params) integrated_optimization_cost_function(swarm, params, optvar_scaling_factor, gravity_model, bandwidth_parameters);

% Try calling the "proper cost function"
% [goal, gradient] = fun(initial_conditions);
[goal] = fun(initial_conditions);

% num_gradient = numerical_gradient(fun, relay_initial_condition, 1e-4);


% Nonlinear constraint
% TODO de-hardcode
max_distance = 120000;
min_distance = 25000;
% nonlcon = @(params) communication_constraints(spacecraft,params, gravity_model,ctime,GM, max_distance, min_distance, optvar_scaling_factor);
relay_orbit_indices = 1:1:N;
nonlcon = @(params) communication_constraints(swarm,relay_orbit_indices, params, optvar_scaling_factor, gravity_model, max_distance, min_distance);
% nonlcon = [];

% Stop function - to enforce stop after a given amount of time
start_time=tic;
stop_fun = @(x,optimValues,state) stop_function(x,optimValues,state, start_time, max_optimization_time);
options.OutputFcn = stop_fun;

% Optimize!
[x,fval,exitflag,output] = fmincon(fun,initial_conditions,A,b,Aeq,beq,lb,ub,nonlcon,options);


% Add the inputs to the swarm
for sc =1:N
    offset = 6*(sc-1);
    assert(all(size(x(1+offset:6+offset)) == size(optvar_scaling_factor)), "ERROR - now you miss Numpy's pure vectors, don't you?")
    sc_initial_condition = (x(1+offset:6+offset)./optvar_scaling_factor)';
    swarm.integrate_trajectory(sc, gravity_model, sc_initial_condition, 'absolute');
end
end

function [stop] = stop_function(x,optimValues,state, start_time, max_time)
    time_elapsed = toc(start_time);
    if time_elapsed>max_time
        stop=1;
    else
        stop=0;
    end
end