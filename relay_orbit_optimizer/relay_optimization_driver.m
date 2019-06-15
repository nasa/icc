%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%             Fmincon-based optimizer for relay orbits. WIP               %
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

clear all; close all; clc;

addpath(genpath('../utilities/'))
initialize_SBDT()

% % a: between 40 and 100;
% % e: between 0 and 0.5
% % i: between -pi/2 and pi/2
% % Omega: between -pi and pi
% % omega: between -pi and pi
% % theta: between -pi and pi
% ub = [10, 0.5, pi/2, pi, pi, pi];
% lb = [4, 0, -pi/2, -pi, -pi, -pi];
% x0 = [50000, 0, 0, 0, 0, 0];

ub = [];
lb = [];

relay_parameters = [55*1e3,0,0.001,0,0,0];
GM = 4.462754720040000e+05;
[sc_location,sc_vel]=op2rv(...
relay_parameters(1),relay_parameters(2),relay_parameters(3),...
relay_parameters(4),relay_parameters(5),relay_parameters(6),...
GM);
relay_initial_condition = [sc_location; sc_vel];

% options = optimset('Display','iter');
options = optimoptions('fmincon','SpecifyObjectiveGradient',true, 'Display', 'Iter');
A = [];
b = [];
Aeq = [];
beq = [];

% Create an instance of the problem, so we only compute the fixed orbits
% once
[goal, spacecraft, gravity_model, ctime, GM] = comm_optimization_problem(relay_initial_condition, [0 : 300 : 86400], 0);

% Scaling may or may not help
location_scaling_factor = 1e-4;
relay_initial_condition(1:3) = relay_initial_condition(1:3) * location_scaling_factor;

% One test call to the cost function
[goal] = relay_optimization_cost_function(spacecraft,relay_initial_condition, gravity_model,ctime,GM, location_scaling_factor);
if isnan(goal)
    error('ERROR: initial location is infeasible. fmincon will crash.')
end

% Proper cost function
fun = @(params) relay_optimization_cost_function(spacecraft, params, gravity_model, ctime, GM, location_scaling_factor);
% Nonlinear constraint
max_distance = 120000;
min_distance = 25000;
nonlcon = @(params) communication_constraints(spacecraft,params, gravity_model,ctime,GM, max_distance, min_distance, location_scaling_factor);

% Optimize!
[x,fval,exitflag,output] = fmincon(fun,relay_initial_condition,A,b,Aeq,beq,lb,ub,nonlcon,options)

% Rescale the inputs.
x(1:3) = x(1:3) / location_scaling_factor;

% Plot
[goal] = comm_optimization_problem(x,[0 : 300 : 86400], true);
