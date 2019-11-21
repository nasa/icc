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

function [occluded] = is_occluded(x1, x2, spherical_asteroid_parameters)
    % Compute minimum distance between segment going through x1, x2 and the
    % asteroid, assumed to be at (0,0,0)
    % Careful! You care about the minimum distance between the objects
    % between the first and the second point! That is, if the closest
    % approach point is not between x1 and x2, you do not especially care
    distance = distance_segment_to_point(x1, x2, [0;0;0]);
    if distance > spherical_asteroid_parameters.max_radius
        occluded = 0;
    elseif distance < spherical_asteroid_parameters.min_radius
        occluded = 1;
    elseif spherical_asteroid_parameters.max_radius == spherical_asteroid_parameters.min_radius
        occluded = 0.5;
    else
        occluded = (distance-spherical_asteroid_parameters.min_radius)/(spherical_asteroid_parameters.max_radius-spherical_asteroid_parameters.min_radius);
    end
end