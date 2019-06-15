%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%             Enforces constraints on the relay orbits.                   %
%             For use with relay_optimization_driver.                     %                  
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

function [C, Ceq] = communication_constraints(spacecraft, sc_state, gravity_model,ctime,GM, max_distance, min_distance, location_scaling_factor)

Ceq = 0;
if nargin<7
    disp("Default min. distance of 25 km")
    min_distance = 25000;
end
if nargin<6
    disp("Default max. distance of 120 km")    
    max_distance=120000;
end

addpath(genpath('../utilities'))

% In order to ensure we do not get too close, we re-integrate the orbit.
% This is all sorts of atrocious.

sc_state(1:3) = sc_state(1:3) / location_scaling_factor;

[time_re,abs_traj_re,~] = gravity_model.integrate([ctime(1), ctime(end)], sc_state);

abs_traj_re = interp1(time_re,abs_traj_re,ctime);

abs_pos_re = abs_traj_re(:,1:3);

distance = sqrt(diag(abs_pos_re *abs_pos_re'));
min_distance_constraint = -(distance-min_distance);
max_distance_constraint = (distance-max_distance);
C = [min_distance_constraint; max_distance_constraint]';