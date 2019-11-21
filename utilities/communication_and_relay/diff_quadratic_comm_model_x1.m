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

function [db_dx1] = diff_quadratic_comm_model_x1(x1, x2, dir, bandwidth_parameters, occlusion_test, scaling_factor)
    if nargin<6
        scaling_factor=bandwidth_parameters.reference_distance;
    end
    if nargin<5
        occlusion_test = @(x1, x2) 0.;
    end
    reference_bandwidth=bandwidth_parameters.reference_bandwidth;
    reference_distance=bandwidth_parameters.reference_distance;
    max_bandwidth=bandwidth_parameters.max_bandwidth;
    if occlusion_test(x1, x2) == 1
        db_dx1 = 0; % Fully occluded
    elseif quadratic_comm_model(x1, x2, bandwidth_parameters,occlusion_test,scaling_factor)>=max_bandwidth
        db_dx1 = 0; %zeros(size(x1));
    else
        scaled_x1=x1./scaling_factor;
        scaled_x2=x2./scaling_factor;
        reference_distance=reference_distance./scaling_factor;        
%          db_dx1 = -2*reference_bandwidth*(reference_distance/norm(x2-x1,2)^2)^2 * (x1(dir)-x2(dir));
        % Numerical conditioning - let's try and get something that looks
        % like a distance before squaring
         db_dx1 = -2*reference_bandwidth.* ( reference_distance/norm(scaled_x2-scaled_x1,2)^2).^2 .* (scaled_x1(dir)-scaled_x2(dir));
         db_dx1 = db_dx1./scaling_factor;
    end
    if occlusion_test(x1, x2) > 0 && db_dx1 ~= 0
        warning("[diff_quadratic_comm_model_x1]: Behavior of gradient with partial occlusions is not well-defined");
    end
    
end