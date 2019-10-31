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

function [dk_dic, dk_dbandwidth, dbandwidth_dlocation, dlocation_dic] = compute_gradient(swarm, reference_distance, reference_bandwidth, max_bandwidth)
if ~swarm.is_valid()
    error("Swarm is not valid")
end

N=swarm.get_num_spacecraft();
K = swarm.get_num_timesteps;
% Derivative of goal function wrt bandwidth

dk_dbandwidth = reshape(swarm.Communication.dual_bandwidths_and_memories,1,K*N*N);

% Derivative of bandwidth wrt spacecraft location
dbandwidth_dlocation_full = zeros(K,N,N,K,N,3);
for k=1:K
    for i=1:N
        for j=1:N
            x1 = swarm.abs_trajectory_array(k,1:3,i);
            x2 = swarm.abs_trajectory_array(k,1:3,j);
            if k<K
                temp_time_step = swarm.sample_times(k+1)-swarm.sample_times(k);
            else
                temp_time_step = swarm.sample_times(k)-swarm.sample_times(k-1);
            end
            % Derivatives wrt all three directions are vectorized
            for dir=1:3
                dbandwidth_dlocation_full(k,i,j,k,i,dir) = diff_quadratic_comm_model_x1(x1, x2, dir, reference_distance,reference_bandwidth,max_bandwidth)*temp_time_step;
                dbandwidth_dlocation_full(k,i,j,k,j,dir) = diff_quadratic_comm_model_x2(x1, x2, dir, reference_distance,reference_bandwidth,max_bandwidth)*temp_time_step;
            end
        end
    end
end

dbandwidth_dlocation = reshape(dbandwidth_dlocation_full,[K*N*N,K*N*3]);

% Derivative of sc location wrt initial conditions
dlocation_dic_full = zeros(K,N,3,N,6);
for sc = 1:N
    for t = 1:K
        for new_state = 1:1:3
            dlocation_dic_full(t,sc,new_state,sc,:) = swarm.state_transition_matrix(new_state, :, t, sc);
        end
    end
end

dlocation_dic_mat = reshape(dlocation_dic_full,[K*N*3,N,6]);
dlocation_dic = cell(N,1);
for sc=1:N
    dlocation_dic{sc} = reshape(dlocation_dic_mat(:,sc,:),[K*N*3,6]);
end

dk_dic = cell(N,1);
for sc = 1:N
    dk_dic{sc} = dk_dbandwidth*dbandwidth_dlocation*dlocation_dic{sc};
end
end

function [bandwidth] = quadratic_comm_model(x1, x2, reference_distance,reference_bandwidth,max_bandwidth, scaling_factor)
    if nargin<6
        scaling_factor=reference_distance;
    end
    x1=x1./scaling_factor;
    x2=x2./scaling_factor;
    reference_distance=reference_distance./scaling_factor;
    bandwidth = min(max_bandwidth, reference_bandwidth*(reference_distance/norm(x2-x1,2))^2);
end

function [db_dx1] = diff_quadratic_comm_model_x1(x1, x2, dir, reference_distance,reference_bandwidth,max_bandwidth,scaling_factor)
    if nargin<7
        scaling_factor=reference_distance;
    end

    if quadratic_comm_model(x1, x2, reference_distance,reference_bandwidth,max_bandwidth,scaling_factor)>=max_bandwidth
        db_dx1 = 0; %zeros(size(x1));
    else
        x1=x1./scaling_factor;
        x2=x2./scaling_factor;
        reference_distance=reference_distance./scaling_factor;        
%          db_dx1 = -2*reference_bandwidth*(reference_distance/norm(x2-x1,2)^2)^2 * (x1(dir)-x2(dir));
        % Numerical conditioning - let's try and get something that looks
        % like a distance before squaring
         db_dx1 = -2*reference_bandwidth.* ( reference_distance/norm(x2-x1,2)^2).^2 .* (x1(dir)-x2(dir));
         db_dx1 = db_dx1./scaling_factor;
    end
    
end

function [db_dx2] = diff_quadratic_comm_model_x2(x1, x2, dir, reference_distance,reference_bandwidth,max_bandwidth, scaling_factor)
    if nargin<7
        scaling_factor=reference_distance;
    end
    db_dx2 = - diff_quadratic_comm_model_x1(x1, x2, dir, reference_distance,reference_bandwidth,max_bandwidth, scaling_factor);
end