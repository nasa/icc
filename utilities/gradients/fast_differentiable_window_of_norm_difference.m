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

function [wnd,dwnd_dv1, dwnd_dv2] = fast_differentiable_window_of_norm_difference(v1, v2, lower_edge, upper_edge, lower_width, upper_width)
%DIFFERENTIABLE_WINDOW_OF_NORM_DIFFERENCE returns a "window of norm difference"
%   function and its derivative with respect to the input vectors
%   The function returns a function that computes window(norm(v1-v2)), where
%   "window" is the product of two logistic functions (as defined in
%   "differentiable_window". It also returns the derivative of the function
%   with respect to the vectors v1 and v2.
%   This function takes four inputs:
%   - lower_edge, the lower edge of the window
%   - upper_edge, the upper edge of the window
%   - lower_width, the "width" of the lower edge. By default, 1.
%   - upper_width, the "width" of the upper edge. By default, equal to
%      lower_width
%   The return value is a trio of functions: the window-of-norm-diff function
%   and its derivatives with respect to v1 and v2 respectively.

if nargin<5
    lower_width=1;
end
if nargin<6
    upper_width = lower_width;
end

% Get the norm function
[normdiff,dnormdiff_dv1, dnormdiff_dv2] = fast_differentiable_norm_of_difference(v1,v2);

% Get the window function
[wnd,dwf] = fast_differentiable_window(normdiff, lower_edge,upper_edge, lower_width, upper_width);

dwnd_dv1 = dwf.*dnormdiff_dv1;
dwnd_dv2 = dwf.*dnormdiff_dv2;

end