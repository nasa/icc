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
options = optimoptions('fmincon',... %     'SpecifyObjectiveGradient',true,...
    'Display', 'Iter',...
    'CheckGradients', true, ...
    'UseParallel', true, ...
    'PlotFcn', []);
%     'PlotFcn', 'optimplotstepsize');
A = [];
b = [];
Aeq = [];
beq = [];

% This scales the variables size to attempt to make the problem better
% conditioned. 
optvar_scaling_factor = [1e-4; 1e-4; 1e-4; 1; 1; 1];

N = swarm.get_num_spacecraft();
% Create initial conditions from Swarm fobject



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


% Nonlinear constraint. We no longer integrate a second time - rather, we
% check the ICs, and we throw an exception if the integrator ends up inside
% the reference ellipse.
% TODO de-hardcode. These are in KM
max_distance_km = 120;
min_distance_km = 10;
% nonlcon = @(params) communication_constraints(spacecraft,params, gravity_model,ctime,GM, max_distance, min_distance, optvar_scaling_factor);
% relay_orbit_indices = 1:1:N;
% nonlcon = @(params) communication_constraints(swarm,relay_orbit_indices, params, optvar_scaling_factor, gravity_model, max_distance, min_distance);
% nonlcon = [];

% Upper and lower bounds: never go farther than max distance, max speed is
% consistent with a circular orbit at min distance, never go closer than
% min_distance, min speed is speed of an elliptical orbit with apocenter at
% max_distance and pericenter at min_distance, at the apocenter.
ub = [];
lb = [];
max_speed_circ = sqrt(gravity_model.BodyModel.gravity.gm/min_distance_km)*1e3; % [m/s]
max_speed_ellip = sqrt(gravity_model.BodyModel.gravity.gm*(2/min_distance_km-2/(max_distance_km+min_distance_km)))*1e3;
max_speed = max_speed_ellip;
% 1/sqrt(3) because we impose the limit on any one direction. A speed of v
% in x, y, and z would result in a speed of sqrt(3) v in magnitude.
% min_speed = 1/sqrt(3)*sqrt(gravity_model.BodyModel.gravity.gm*(2/max_distance_km-2/(max_distance_km+min_distance_km)))*1e3; % [m/s]


for i=1:N
    offset = 6*(i-1);
    ub(offset+1:offset+3) = max_distance_km*1e3;
    ub(offset+4:offset+6) = max_speed;
%     lb(offset+1:offset+3) = min_distance_km*1e3;
%     lb(offset+4:offset+6) = min_speed;
    ub(offset+1:offset+6) = ub(offset+1:offset+6)'.*optvar_scaling_factor;
%     lb(offset+1:offset+6) = lb(offset+1:offset+6)'.*optvar_scaling_factor;
end
lb = -ub;


% Stop function - to enforce stop after a given amount of time
start_time=tic;
stop_fun = @(x,optimValues,state) stop_function(x,optimValues,state, start_time, max_optimization_time);
options.OutputFcn = stop_fun;

problem = createOptimProblem('fmincon', 'objective', fun, ...
    'x0', initial_conditions, 'Aineq', A, 'bineq', b, ...
    'Aeq', Aeq, 'beq', beq, 'lb', lb, 'ub', ub, ...
    'options', options);
% [x,fval,exitflag,output] = fmincon(fun,initial_conditions,A,b,Aeq,beq,lb,ub,nonlcon,options);

%% Global Search
% disp("Global search")
% gs = GlobalSearch('Display','iter');
% [x, fval, exitflag, output] = run(gs,problem);

%% Multistart with 50 starting points
% disp("Multistart")
% ms = MultiStart;
% [x, fval, exitflag, output] = run(ms,problem,50);

%% Pattern Search
disp("Pattern search")
psoptions = optimoptions('fmincon',... %     'SpecifyObjectiveGradient',true,...
    'Display', 'Iter',...
    'CheckGradients', true, ...
    'UseParallel', true);
    % Missing OutputFcn for now
[x, fval, exitflag, output] = patternsearch(fun,initial_conditions,A,b,Aeq,beq,lb,ub,psoptions);

%% Genetic Algorithn - does not quite work, need to refine UBs and LBs?
% disp("Genetic algorithm")
% gaoptions = optimoptions('fmincon',... %     'SpecifyObjectiveGradient',true,...
%     'Display', 'Iter',...
%     'CheckGradients', true, ...
%     'UseParallel', true);
%     % Missing OutputFcn for now
% disp(length(initial_conditions))
% [x, fval, exitflag, output] = ga(fun,length(initial_conditions),A,b,Aeq,beq,lb,ub,gaoptions);

%% Particle Swarm - does not support nonlinear constraint
% disp("Particle Swarm")
% psooptions = optimoptions('fmincon',... %     'SpecifyObjectiveGradient',true,...
%     'Display', 'Iter',...
%     'CheckGradients', true, ...
%     'UseParallel', true);
%     % Missing OutputFcn for now
% [x, fval, exitflag, output] = particleswarm(fun,length(initial_conditions),lb,ub,psooptions);

%% Surrogate optimization - requires constraints on the inputs
% disp("Surrogate optimization")
% surrogateopt(fun, lb, ub, options);

%% Simulated annealing
% disp("Simulated annealing")
% [x, fval, exitflag, output] = simulannealbnd(fun, initial_conditions, lb, ub);

%% Vanilla fmincon
% disp("Local fmincon")
% [x,fval,exitflag,output] = fmincon(problem);

%% Add the inputs to the swarm
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