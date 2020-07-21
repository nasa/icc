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

function [bandwidth] = quadratic_comm_model(x1, x2, bandwidth_parameters, occlusion_test, scaling_factor)
    if nargin<3
        bandwidth_parameters.reference_distance = 100000;
        bandwidth_parameters.reference_bandwidth = 250000;
        bandwidth_parameters.max_bandwidth = 100*1e6;
    end
    if nargin<5
        scaling_factor=bandwidth_parameters.reference_distance;
    end
    if nargin<4
        occlusion_test = @(x1, x2) 0.;
    end

    scaled_x1=x1./scaling_factor;
    scaled_x2=x2./scaling_factor;
    scaled_reference_distance=bandwidth_parameters.reference_distance./scaling_factor;
    max_bandwidth=bandwidth_parameters.max_bandwidth;
    reference_bandwidth=bandwidth_parameters.reference_bandwidth;
    bandwidth = min(max_bandwidth, reference_bandwidth*(scaled_reference_distance/norm(scaled_x2-scaled_x1,2))^2);
    occluded = occlusion_test(x1, x2);
    assert(occluded>=0);
    assert(occluded<=1);
    bandwidth = bandwidth*(1-occluded);
end