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
function [normdiff,dnormdiff_dv1, dnormdiff_dv2] = fast_differentiable_norm_of_difference(v1, v2)
%DIFFERENTIABLE_NORM_OF_DIFFERENCE Returns functions to compute the norm of
% the difference between two vectors, and its derivative.
%   v1 is a 3x1 vector.
%   v2 is a 3xNv vector.
%   The function returns three values:
%   Normdiff(v1,v2) is simply norm(v1-v2,2);
%   dnormdiff_dv1(v1,v2) is a vector of the same length as
%   v1, containing d(norm(v1-v2))/d(v1)
%   dnormdiff_dv2(v1,v2) is a vector of the same length as
%   v1, containing d(norm(v1-v2))/d(v2)

assert(all(size(v1,2)==3), "ERROR: v1 should be a matrix with three rows and one column per vector of interest");
assert(size(v2,2)==3, "ERROR: v2 should be a matrix with three rows and one column per vector of interest");
if size(v1,1) == 1
    v1 = repmat(v1, size(v2,1),1);
end
assert(all(size(v1)==size(v2)), "v1 and v2 should have the same size, OR v1 should be a 1x3 vector");


% TODO vectorize (this should be norm(v1(i,:)-v2(j,:));
difference = v1-v2;
normdiff = vecnorm(difference,2,2); % This is a row vector of the same length as
%size(v2,2). normdiff(i) is the norm of v1-v2(:,i)
% normdiff = norm(v1-v2,2);

% vectorized
% v1-v2 is of the same size as v2. Each column is v1-v2(:,i)
% We want dnormdiff_dv1 = 1/normdiff(i)*(v1-v2(:,i));
dnormdiff_dv1 = 1./normdiff.* (v1-v2);
dnormdiff_dv2 = -1./normdiff.* (v1-v2);

end

