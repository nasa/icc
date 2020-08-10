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

function [sf, dsf] = fast_differentiable_logistic_function(x, edge, width)
%DIFFERENTIABLE_LOGISTIC_FUNCTION returns a logistic function and its derivative
%   The function takes three inputs:
%   - edge, the location of the sigmoid transition
%   - width, the width of the transition
%   The return value is a function.
%   The function returned takes in a 1d input and returns
%   (1/(1+exp(-(x-edge)/width)))
if width==0
    if x == edge
        sf = 0.5*ones(size(x));
        dsf = NaN*ones(size(x));
    else
        sf = (x>edge);
        dsf = 0*ones(size(x));
    end
    return
end

sf = (1./(1+exp(-(x-edge)./width)));
dsf = sf.*(1-sf)./width;

end