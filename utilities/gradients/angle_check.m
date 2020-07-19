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

function [range_valid, drange_valid] = angle_check(sc_position, r_vertices, range_cell, angle_tolerances)
% ANGLE_CHECK Checks if the angle between two vectors is inside a given window.   
    assert(size(sc_position,2) == 3, "first entry should be 1 by 3 or Nv by 3")
    assert(size(r_vertices,2) == 3, "second entry should be Nv by 3")
    Nv = size(r_vertices,1);
    range_valid = -inf*ones(Nv,1);
    drange_valid = zeros(Nv,3);
    for i = 1:length(range_cell)
        % TODO vectorize
        [new_range_valid,new_drange_valid, ~] = fast_differentiable_window_of_angle(sc_position, r_vertices, range_cell{i}(1),range_cell{i}(2), angle_tolerances{i});
        % Update gradient and range to take the max
        drange_valid(new_range_valid>range_valid,:) = new_drange_valid(new_range_valid>range_valid,:);
        range_valid = max(range_valid, new_range_valid);
    end
end