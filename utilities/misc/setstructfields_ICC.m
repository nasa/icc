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