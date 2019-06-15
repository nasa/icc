%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%               Usage example of ICC comms optimization                   %
%                                                                         %
% Demonstrates the usage of the network communications optimization       %
% module. The module creates a simple scenario with two instrumented      %
% spacecraft and a relay spacecraft, and computes the optimal             %
% communication architecture to relay data to the carrier.                %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

addpath(genpath('../utilities'));
addpath(genpath('../relay_orbit_optimizer'));

clear, clc, close all, run ../startup.m  % refresh

plot_flag = true;

time_bounds = [0:300:86400];

relay_orbital_parameters = [50*1e3,0,-pi/4,0,0,0];
GM = 4.462754720040000e+05;
[sc_location,sc_vel]=op2rv(...
relay_orbital_parameters(1),relay_orbital_parameters(2),relay_orbital_parameters(3),...
relay_orbital_parameters(4),relay_orbital_parameters(5),relay_orbital_parameters(6),...
GM);
relay_initial_condition = [sc_location; sc_vel];

[goal, spacecraft, ErosGravity, time_bounds, GM, dgoal_dic] = comm_optimization_problem(relay_initial_condition,time_bounds,plot_flag);