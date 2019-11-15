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

function [distance, closest_point] = distance_segment_to_point(line_pt_1, line_pt_2, ref_point)
    % Compute distance between a segment defined by points line_pt_1 and
    % line_pt_2, and a point ref_point.
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
    
    [distance, closest_point] = distance_line_to_point(line_pt_1, line_pt_2, ref_point);
    % Now, is closest_point on the segment between line_pt_1 and line_pt_2?
    pt_1_to_closest = closest_point-line_pt_1;
    pt_2_to_closest = closest_point-line_pt_2;
    line_unit_vector = (line_pt_2 - line_pt_1)/norm( line_pt_2 - line_pt_1);
    % If closest_point is between pt1 and pt2, then pt1_to_closest and
    % pt2_to_closest point in opposite directions
    if sign(pt_1_to_closest'*line_unit_vector) ~= sign(pt_2_to_closest'*line_unit_vector)
        % The closest point is indeed between pt1 and pt2
        return
    else
        distance_1 = norm(line_pt_1-ref_point);
        distance_2 = norm(line_pt_2-ref_point);
        if distance_1<distance_2
            distance = distance_1;
            closest_point = line_pt_1;
        else
            distance = distance_2;
            closest_point = line_pt_2;
        end
    end
end