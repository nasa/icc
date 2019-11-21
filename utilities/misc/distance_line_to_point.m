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

function [distance, closest_point] = distance_line_to_point(line_pt_1, line_pt_2, ref_point)
    % Compute distance between a line defined by points line_pt_1 and
    % line_pt_2, and a point ref_point.
    % Follows https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    % Ensure everything is a column vector
    if size(line_pt_1,1) ==1
        line_pt_1 = line_pt_1';
    end
    if size(line_pt_2,1) ==1
        line_pt_2 = line_pt_2';
    end
    if size(ref_point,1) ==1
        ref_point = ref_point';
    end
   % First, if line_pt_1 coincides with line_pt_2, just compute the norm
    if norm(line_pt_2-line_pt_1) == 0
        distance = norm(line_pt_2-ref_point);
        closest_point = line_pt_2;
        return
    end
    % Define the line as line_pt_1 + (line_pt_2-line_pt_1)*t
    % For ease of notation, define 
    n_unit_vec = (line_pt_2-line_pt_1)/norm(line_pt_2-line_pt_1);
    % Then the line is line_pt_1 + n_unit_vec*t, where t is a scalar.
    % The component of the distance from ref_point to line_pt_1 that is
    % parallel to the line is
    distance_parallel_to_line = ((line_pt_1-ref_point)'*n_unit_vec)*n_unit_vec;
    % Then the distance from line_pt_1 to ref_point that is perpendicular
    % to the line (i.e., the actual distance from the line to ref_point) is
    distance_vector = (line_pt_1-ref_point) - distance_parallel_to_line;
    distance = norm(distance_vector);
    closest_point = distance_vector + ref_point;
    % Just to be sure, assert that distance_vector is perpendicular to n
%   assert(distance_vector'*n_unit_vec<10*eps, "ERROR: distance vector is not perpendicular to line")

end