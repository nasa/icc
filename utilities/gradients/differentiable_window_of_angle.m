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

function [wa,dwa_dv1, dwa_dv2] = differentiable_window_of_angle(varargin)
%DIFFERENTIABLE_WINDOW_OF_ANGLE returns a "window of angle" function and
%   its derivative with respect to the input vectors
%   The function returns a function that computes window(angle(v1,v2)), where
%   "window" is the product of two logistic functions (as defined in
%   "differentiable_window" and angle(v1,v2) = acos(1/norm(v1)*1/norm(v2)*dot(v1,v2)).
%   It also returns the derivative of the function with respect to the
%   vectors v1 and v2.
%   This function takes four inputs:
%   - lower_edge, the lower edge of the window
%   - upper_edge, the upper edge of the window
%   - lower_width, the "width" of the lower edge. By default, 1.
%   - upper_width, the "width" of the upper edge. By default, equal to
%      lower_width
%   The inputs are passed to differentiable_window
%   The return value is a trio of functions: the window-of-angle function
%   and its derivatives with respect to v1 and v2 respectively.

% Get the window function
[wf,dwf] = differentiable_window(varargin{:});

% Get the angle function
[a,da_dv1, da_dv2] = differentiable_angle_between_vectors();

wa = @(v1,v2) wf(a(v1,v2));
dwa_dv1 = @(v1,v2) dwf(a(v1,v2)).*da_dv1(v1,v2);
dwa_dv2 = @(v1,v2) dwf(a(v1,v2)).*da_dv2(v1,v2);

end