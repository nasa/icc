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
function [wf,dwf] = fast_differentiable_window(x, lower_edge,upper_edge, lower_width, upper_width)
%DIFFERENTIABLE_WINDOW returns a "window" function and its derivative
%   The function returns a one-dimensional "window" function, the product
%   of two sigmoids. It takes four inputs:
%   - lower_edge, the lower edge of the window
%   - upper_edge, the upper edge of the window
%   - lower_width, the "width" of the lower edge. By default, 1.
%   - upper_width, the "width" of the upper edge. By default, equal to
%      lower_width
%   The return value is a function.
%   The function returned takes in a 1d input and returns
%   (1/(1+exp(-(x-lower_edge)/lower_width)))*(1-(1/(1+exp(-(x-upper_edge)/upper_width))))

if nargin<4
    lower_width = 1.;
end
if nargin<5
    upper_width = lower_width;
end

[lower_lf, dlower_lf] = fast_differentiable_logistic_function(x, lower_edge, lower_width);
[upper_lf, dupper_lf] = fast_differentiable_logistic_function(x, upper_edge, upper_width);

wf = lower_lf.*(1-upper_lf);
dwf =dlower_lf.*(1-upper_lf) - lower_lf.*dupper_lf;

end

