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

function reward_map = get_coverage_reward_map(AsteroidModel, Swarm, sc_optimized)
%GET_COVERAGE_REWARD Defines the value of the observed points
%   reward_map{i}(j,k) defines the reward accociated with agent i observing
%   vertex j at time k
% 
%   The coverage reward may be based on factors such as the location of the
%   vertex on the asteroid, or how far away the spacecraft angle, sun angle
%   and height are from their optimal values. 
%   
%   Note: it is only neccesary to define reward_map{i}(j,k) for
%    observable combinations of i,j,k (i.e. if
%    observable_points_map{i}(j,k)==1)

if nargin<3
    sc_optimized = Swarm.which_trajectories_set();
end

flag_map = 3; % 0 for uniform reward

N = Swarm.get_num_spacecraft();
pos_points = AsteroidModel.BodyModel.shape.faceCenters;
Nv = size(pos_points,1);
K = Swarm.get_num_timesteps();
reward_map = cell(1,N);

for i_sc = sc_optimized
    sc_type = Swarm.Parameters.types{i_sc};
    % Every instrument has a different reward
    instr_reward = instrument_reward(sc_type);
    
    if flag_map==0
        %% Uniform Map
        reward_map{i_sc} = ones(Nv,K);

    elseif flag_map==1
        %% Random Map
        reward_map{i_sc} = randi(10,Nv,K);
        
    elseif flag_map==2
        %% Value points closer to center of gravity
        location_reward = zeros(Nv,1);
        for i_v = 1:Nv
            location_reward(i_v) = norm(pos_points(i_v,:))./1000;
        end
        for k=1:K
            reward_map{i_sc}(:,k) = location_reward;
        end
    elseif flag_map==3
        %% Value points further from center of gravity
        location_reward = zeros(Nv,1);
        for i_v = 1:Nv
            location_reward(i_v) = AsteroidModel.BodyModel.shape.maxRadius-norm(pos_points(i_v,:))./1000;
        end
        for k=1:K
            reward_map{i_sc}(:,k) = location_reward;
        end
    end
    reward_map{i_sc} = reward_map{i_sc}*instr_reward;
    
end

end

function [reward] = instrument_reward(sc_type)
    switch sc_type
        case -1
            reward = 1;
        case -2
            reward = 0;
        case -3
            reward = 0.9;
        otherwise
            reward = 1.;
    end
end