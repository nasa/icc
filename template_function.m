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

