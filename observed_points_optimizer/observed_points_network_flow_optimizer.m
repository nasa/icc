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

function Swarm = observed_points_network_flow_optimizer(Swarm, AsteroidModel, bandwidth_parameters)

N = Swarm.get_num_spacecraft(); % number of spacecraft

for i_sc = setdiff(1:N, Swarm.get_indicies_of_type(0))
    sc_same_type = intersect(1:i_sc, Swarm.get_indicies_of_type(Swarm.Parameters.types{i_sc})); % all SC <= i_sc of same type
    Swarm = observed_points_optimizer_main(AsteroidModel, Swarm, sc_same_type, i_sc); % observe the asteroid, and update the coverage reward to include the new orbit
end

spherical_asteroid_parameters.max_radius = AsteroidModel.BodyModel.shape.maxRadius*1e3;
spherical_asteroid_parameters.min_radius = AsteroidModel.BodyModel.shape.maxRadius*1e3;

occlusion_test =  @(x1, x2) is_occluded(x1, x2, spherical_asteroid_parameters);
bandwidth_model = @(x1, x2) quadratic_comm_model(x1, x2, bandwidth_parameters, occlusion_test);

data_scaling_factor = mean(mean(Swarm.Observation.flow(Swarm.Observation.flow>0)));

[Swarm, ~] = communication_optimizer(Swarm, bandwidth_model, data_scaling_factor);
