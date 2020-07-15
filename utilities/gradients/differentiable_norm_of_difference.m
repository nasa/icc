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
function [normdiff,dnormdiff_dv1, dnormdiff_dv2] = differentiable_norm_of_difference()
%DIFFERENTIABLE_NORM_OF_DIFFERENCE Returns functions to compute the norm of
% the difference between two vectors, and its derivative.
%   The function returns three functions:
%   Normdiff(v1,v2) is simply norm(v1-v2,2);
%   dnormdiff_dv1(v1,v2) is a function that returns a vector of the same length as
%   v1, containing d(norm(v1-v2))/d(v1)
%   dnormdiff_dv2(v1,v2) is a function that returns a vector of the same length as
%   v1, containing d(norm(v1-v2))/d(v2)

normdiff = @(v1,v2) norm(v1-v2,2);

dnormdiff_dv1 = @(v1,v2) 1/norm(v1-v2,2).* (v1-v2);
dnormdiff_dv2 = @(v1,v2) -1/norm(v1-v2,2).* (v1-v2);

end

