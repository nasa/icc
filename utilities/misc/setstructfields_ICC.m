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

function output_structure = setstructfields_ICC(input_structure, new_data)
% struct_out = SETSTRUCTFIELDS_ICC(input_structure, fields_to_add)
% A function to add fields to a struct.
% Inputs:
% - input_structure, a struct
% - new_data, another structure whose content will be copied in
%   struct_in
% Fields in input_structure that are also contained in fields_to_add will
% be overwritten.
% Fields in input_structure that are not contained in fields_to_add will be
% carried over as is.
% Clean-room implementation of MATLAB's setstructfields following
% https://stackoverflow.com/questions/38645/what-are-some-efficient-ways-to-combine-two-structures-in-matlab/23767610#23767610

% Create a copy of the input structure
output_structure = struct;

% Go over the fields in fields_to_add
fields_to_copy= fieldnames(input_structure);

% First, copy the original input_structure in output_structure
if ~isempty(fields_to_copy)
    for field_ix = 1:length(fields_to_copy)
        field = fields_to_copy{field_ix};
        output_structure.(field) = input_structure.(field);
    end
end

% Next, go over the fields in fields_to_add
fields_to_add = fieldnames(new_data);
for field_ix = 1:length(fields_to_add)
    field = fields_to_add{field_ix};
    % If a struct, recurse - so we do not delete existing content
    % Also note the unusual syntax to convert a variable name in a field:
    % https://www.mathworks.com/help/matlab/matlab_prog/generate-field-names-from-variables.html
    if ismember(field, fieldnames(output_structure)) && isstruct(output_structure.(field))
        output_structure.(field) =  setstructfields_ICC(input_structure.(field), new_data.(field));
    % If not a struct, overwrite
    else
        output_structure.(field) = new_data.(field);
    end
end

% Return
return