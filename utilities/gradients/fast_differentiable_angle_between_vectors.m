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

assert(all(size(v1,2)==3), "ERROR: v1 should be a matrix with three rows and one column per vector of interest");
assert(size(v2,2)==3, "ERROR: v2 should be a matrix with three rows and one column per vector of interest");
if size(v1,1) == 1
    v1 = repmat(v1, size(v2,1),1);
end
assert(all(size(v1)==size(v2)), "v1 and v2 should have the same size, OR v1 should be a 1x3 vector");

% TODO vectorize.
% dot(v1, v2) = v1'*v2
% norm(v1) is OK
% norm(v2) = v2'

if size(v2,1) == 1
    a = acos(1./(norm(v1).*norm(v2)) *dot(v1,v2));
    
    if nargout>1
        da_dv1 = -1/sqrt(1-(1/(norm(v1).*norm(v2)) *dot(v1,v2)).^2) * ...
            ((-(v1/norm(v1))./norm(v1)^2 .* 1./norm(v2) .* dot(v1,v2)) + ...
            (1./norm(v1).*1./norm(v2).*v2));
        if nargout>2
            da_dv2 = -1./sqrt(1-(1./(norm(v1).*norm(v2)) .*dot(v1,v2)).^2) * ...
                ((1./norm(v1).*(-v2./norm(v2))./norm(v2).^2 .* dot(v1,v2)) + ...
                (1./norm(v1).*1./norm(v2).*v1));
        end
    end
else
    normv1 = vecnorm(v1,2,2);
    normv2 = vecnorm(v2,2,2);
    dotv1v2 = dot(v1,v2,2);
    a = acos(1./(normv1.*normv2).*dotv1v2);

    if nargout>1
%         da_dv1s = zeros(size(v2));
%         for i=1:size(v2,1)
%             da_dv1s(i,:) = -1./sqrt(1-(1./(normv1(i).*normv2(i)') .*dotv1v2(i)).^2) .* ...  % This is size(v2,1)
%                 ((-(v1(i,:)./normv1(i))./normv1(i).^2 .* 1./normv2(i)' .* dotv1v2(i)) + ...  % The very first bit is of size 3, the next bit is of size(v2,1)
%                 (1./normv1(i).*1./normv2(i).*v2(i,:)));  % The first bit of of size(v2,1), the next bit is 3xsize(v2,1)
%         end
        da_dv1 = repmat(-1./sqrt(1-(1./(normv1.*normv2) .*dotv1v2).^2), 1, 3) .* ...  % This is size(v2,1)
                ((-(v1./normv1)./normv1.^2 .* 1./normv2 .* dotv1v2) + ...  % The very first bit is of size 3, the next bit is of size(v2,1)
                (1./normv1.*1./repmat(normv2,1,3).*v2));
%         assert (all(all(da_dv1==da_dv1s)), "ERROR on dv1");
%         warning("Slow angles, remove asserts!")
        if nargout>2
%                 da_dv2s = zeros(size(v2));
%                 for i=1:size(v2,1)
%                     da_dv2s(i,:) = -1./sqrt(1-(1./(normv1(i).*normv2(i)') .*dotv1v2(i)).^2)' .* ...
%                         ((1./normv1(i).*(-v2(i,:)./normv2(i))./normv2(i).^2 .* dotv1v2(i)) + ...
%                         (1./normv1(i).*1./normv2(i).*v1(i,:)));
%                 end
            da_dv2 = repmat(-1./sqrt(1-(1./(normv1.*normv2) .*dotv1v2).^2),1,3) .* ...
                ((1./normv1.*(-v2./normv2)./normv2.^2 .* dotv1v2) + ...
                (1./normv1.*1./normv2.*v1));
%             assert (all(all(da_dv2==da_dv2s)), "ERROR on dv2");
%             warning("Slow angles, remove asserts!")
        end
    end
end

end

