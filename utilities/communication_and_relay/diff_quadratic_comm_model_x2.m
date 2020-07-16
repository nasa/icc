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

function [db_dx2] = diff_quadratic_comm_model_x2(x1, x2, dir, bandwidth_parameters, occlusion_test, scaling_factor)
    if nargin<6
        scaling_factor=bandwidth_parameters.reference_distance;
    end
    if nargin<5
        occlusion_test = @(x1, x2) 0.;
    end
    [db_dx1, db_dx2] = diff_quadratic_comm_model(x1, x2, dir, bandwidth_parameters, occlusion_test, scaling_factor);
        
%     db_dx2 = - diff_quadratic_comm_model_x1(x1, x2, dir, bandwidth_parameters, occlusion_test, scaling_factor);
end