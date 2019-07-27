%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%             Fmincon-based optimizer for relay orbits.               %
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

function [swarm] = relay_optimization(swarm, gravity_model, bandwidth_parameters, relay_orbit_indices)

if nargin<3
    relay_orbit_indices = [3];
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
options = optimoptions('fmincon',...
    'SpecifyObjectiveGradient',true,...
    'Display', 'Iter',...
    'CheckGradients', true, ...
    'UseParallel', true, ...
    'PlotFcn', 'optimplotstepsize');
A = [];
b = [];
Aeq = [];
beq = [];

optvar_scaling_factor = [1e-4; 1e-4; 1e-4; 1; 1; 1];

% Create bandwidth model

relay_initial_condition = zeros(6*length(relay_orbit_indices),1);
for relay_sc =1:length(relay_orbit_indices)
    offset = 6*(relay_sc-1);
    assert(all(size(swarm.abs_trajectory_array(1,:,relay_orbit_indices(relay_sc))') == size(optvar_scaling_factor)));
    relay_initial_condition(1+offset:6+offset) = swarm.abs_trajectory_array(1,:,relay_orbit_indices(relay_sc))'.*optvar_scaling_factor;
end

% One test call to the cost function
[goal, gradient] = relay_optimization_cost_function(swarm,relay_orbit_indices,relay_initial_condition, optvar_scaling_factor, gravity_model, bandwidth_parameters);
if (isnan(goal) || any(isnan(gradient)))
    error('ERROR: initial location is infeasible. fmincon will crash.')
end

% Proper cost function
fun = @(params) relay_optimization_cost_function(swarm, relay_orbit_indices, params, optvar_scaling_factor, gravity_model, bandwidth_parameters);

% Try calling the "proper cost function"
[goal, gradient] = fun(relay_initial_condition);
num_gradient = numerical_gradient(fun, relay_initial_condition, 1e-4);


% Nonlinear constraint
max_distance = 120000;
min_distance = 25000;
% nonlcon = @(params) communication_constraints(spacecraft,params, gravity_model,ctime,GM, max_distance, min_distance, optvar_scaling_factor);
nonlcon = [];


% Optimize!
[x,fval,exitflag,output] = fmincon(fun,relay_initial_condition,A,b,Aeq,beq,lb,ub,nonlcon,options);

% Add the inputs to the swarm
for relay_sc =1:length(relay_orbit_indices)
    offset = 6*(relay_sc-1);
    assert(all(size(x(1+offset:6+offset)) == size(optvar_scaling_factor)), "ERROR - now you miss Numpy's pure vectors, don't you?")
    relay_initial_condition = (x(1+offset:6+offset)./optvar_scaling_factor)';
    swarm.integrate_trajectory(relay_orbit_indices(relay_sc), gravity_model, relay_initial_condition, 'absolute');
end

