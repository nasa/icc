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

function [dk_dic] = compute_gradient(spacecraft, dual_bandwidths_and_memory, reference_distance, reference_bandwidth, max_bandwidth)
N=length(spacecraft.orbits);
if N == 0
    disp('ERROR: no orbits')
else
    K = size(spacecraft.orbits{1},2);
end
% Derivative of goal function wrt bandwidth
dk_dbandwidth = reshape(dual_bandwidths_and_memory,1,K*N*N);

% Derivative of bandwidth wrt spacecraft location
dbandwidth_dlocation_full = zeros(K,N,N,K,N,3);
for k=1:K
    for i=1:N
        for j=1:N
            x1 = spacecraft.orbits{i}(:,k);
            x2 = spacecraft.orbits{j}(:,k);
            % Derivatives wrt all three directions are the same
            dbandwidth_dlocation_full(k,i,j,k,i,:) = diff_quadratic_comm_model_x1(x1, x2, reference_distance,reference_bandwidth,max_bandwidth);
            dbandwidth_dlocation_full(k,i,j,k,j,:) = diff_quadratic_comm_model_x2(x1, x2, reference_distance,reference_bandwidth,max_bandwidth);
        end
    end
end

dbandwidth_dlocation = reshape(dbandwidth_dlocation_full,[K*N*N,K*N*3]);

% Derivative of sc location wrt initial conditions
dlocation_dic_full = zeros(K,N,3,N,6);
for sc = 1:length(spacecraft.state_transition_matrix)
    for t = 1:K
        for new_state = 1:1:3
            dlocation_dic_full(t,sc,new_state,sc,:) = spacecraft.state_transition_matrix{sc}(new_state, :, t);
        end
    end
end

dlocation_dic = reshape(dlocation_dic_full,[K*N*3,6,N]);

dk_dic = cell(N,1);
for sc = 1:N
    dk_dic{sc} = dk_dbandwidth*dbandwidth_dlocation*dlocation_dic(:,:,sc);
end
end

function [bandwidth] = quadratic_comm_model(x1, x2, reference_distance,reference_bandwidth,max_bandwidth)
    bandwidth = min(max_bandwidth, reference_bandwidth*(reference_distance/norm(x2-x1,2))^2);
end

function [db_dx1] = diff_quadratic_comm_model_x1(x1, x2, reference_distance,reference_bandwidth,max_bandwidth)
if quadratic_comm_model(x1, x2, reference_distance,reference_bandwidth,max_bandwidth)>=max_bandwidth
    db_dx1 = zeros(size(x1));
else
    db_dx1 = reference_bandwidth*reference_distance^2*(-2/norm(x2-x1,2)^4) * (x1-x2);
end
end

function [db_dx2] = diff_quadratic_comm_model_x2(x1, x2, reference_distance,reference_bandwidth,max_bandwidth)
db_dx2 = - diff_quadratic_comm_model_x1(x1, x2, reference_distance,reference_bandwidth,max_bandwidth);
end