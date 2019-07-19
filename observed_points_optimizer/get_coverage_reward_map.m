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

function reward_map = get_coverage_reward_map(AsteroidModel, observable_points_map)
%GET_COVERAGE_REWARD Defines the value of the observed points 
%   reward_map{i}(j,k) defines the reward accociated with agent i observing
%   vertex j at time k 

flag_map = 2;

N = length(observable_points_map);
pos_points = AsteroidModel.BodyModel.shape.vertices;
Nv = size(pos_points,1);
K = size(observable_points_map{1},2);
reward_map = cell(1,N);

if flag_map==1
    %% Random Map 
    for i_sc = 1:N
        reward_map{i_sc} = randi(10,Nv,K);
    end
elseif flag_map==2
    %% Value points closer to center of gravity
    location_reward = zeros(Nv,1); 
    for i_v = 1:Nv
        location_reward(i_v) = norm(pos_points(i_v,:))./1000; 
    end
    for i_sc=1:N
        for k=1:K
            reward_map{i_sc}(:,k) = location_reward;
        end
    end
elseif flag_map==3
    %% Value points further from center of gravity
    location_reward = zeros(Nv,1); 
    for i_v = 1:Nv
        location_reward(i_v) = AsteroidModel.BodyModel.shape.maxRadius-norm(pos_points(i_v,:))./1000; 
    end
    for i_sc=1:N
        for k=1:K
            reward_map{i_sc}(:,k) = location_reward;
        end
    end
end

end