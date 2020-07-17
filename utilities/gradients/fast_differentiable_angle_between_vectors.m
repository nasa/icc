%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2020 by California Institute of Technology.  ALL RIGHTS RESERVED. %
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

function [a,da_dv1, da_dv2] = fast_differentiable_angle_between_vectors(v1, v2)
%DIFFERENTIABLE_ANGLE_BETWEEN_VECTORS Returns a function computing the
%angle between two vectors, and its derivative with respect to the
%components of both vectors.
%  The function returns
%  - a(v1,v2) = arccos(1/(norm(v1)*norm(v2) *dot(v1,v2))
%  - da_dv1(v1,v2), the derivative of a wrt v1
%  - da_dv2(v1,v2), the derivative of a wrt v2
%

a = acos(1./(norm(v1).*norm(v2)) *dot(v1,v2));

da_dv1 = -1/sqrt(1-(1/(norm(v1).*norm(v2)) *dot(v1,v2)).^2) * ...
    ((-(v1/norm(v1))./norm(v1)^2 .* 1./norm(v2) .* dot(v1,v2)) + ...
    (1./norm(v1).*1./norm(v2).*v2));

da_dv2 = -1./sqrt(1-(1./(norm(v1).*norm(v2)) .*dot(v1,v2)).^2) * ...
    ((1./norm(v1).*(-v2./norm(v2))./norm(v2).^2 .* dot(v1,v2)) + ...
    (1./norm(v1).*1./norm(v2).*v1));

end

