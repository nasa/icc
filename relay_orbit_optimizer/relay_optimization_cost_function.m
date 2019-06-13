%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%  Wrapper function. Given the parameters of a relay orbit, returns the   %
%  amount of science delivered to the base station.                       %
%  For use with relay_optimization_driver.                                %  
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

function [goal, gradient] = relay_optimization_cost_function(spacecraft,sc_initial_condition, gravity_model,ctime,GM, location_scaling_factor)

addpath('../utilities')
addpath('../network_flow_communication_optimizer')

sc_initial_condition(1:3) = sc_initial_condition(1:3) / location_scaling_factor;

[time_re,abs_traj_re, stm_re] = gravity_model.integrate_absolute(ctime, sc_initial_condition);

spacecraft.orbits{3} = abs_traj_re(:,1:3)';

[flows, effective_science, delivered_science, bandwidths, dual_bandwidths_and_memory] = communication_optimizer(spacecraft);

reference_bandwidth = 250000;
reference_distance = 100000;
max_bandwidth = 100*1e6;

spacecraft.state_transition_matrix{3} = stm_re;
spacecraft.state_transition_matrix{1} = zeros(size(stm_re));
spacecraft.state_transition_matrix{2} = zeros(size(stm_re));
spacecraft.state_transition_matrix{4} = zeros(size(stm_re));


goal = sum(delivered_science);

if nargout>1  % compute gradient
    dgoal_dic = compute_gradient(spacecraft, dual_bandwidths_and_memory, reference_distance, reference_bandwidth, max_bandwidth);
    gradient = dgoal_dic{3};
    % Minimize
    gradient = -gradient;
    gradient(1:3) = gradient(1:3) / location_scaling_factor;
end

% Minimize
goal = -goal;

end