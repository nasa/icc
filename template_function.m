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

function [output_arg1, output_arg2] = template_function(input_arg1, input_arg2)
%TEMPLATE_FUNCTION Summary of this function 
%   Syntax: [outputArg1,outputArg2] = template_function(inputArg1,*inputArg2)
%    *optional input
%   
%   Inputs: 
%    - input_arg1 [units]: Description
%    - *input_arg2 [units]: Description
% 
%   Outputs: 
%    - input_arg1 [units]: Description
%    - input_arg2 [units]: Description

% Code goes here 
output_arg1 = input_arg1;

if nargin > 1 
    output_arg2 = input_arg2;
else 
    output_arg2 = input_arg1;
end

end

