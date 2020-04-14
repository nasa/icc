% Clean up
% Spec:
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

clear all; close all; clc;

% Test: add entry to empty struct
input_structure = struct;
new_fields = struct;
new_fields.aa = 'aaa';
new_fields.bb = 'bbb';
struct_out = setstructfields_ICC(input_structure, new_fields);
assert(strcmp(struct_out.aa, 'aaa'))
assert(strcmp(struct_out.bb, 'bbb'))

% Test: add entry to non-empty struct
input_structure = struct;
input_structure.aa = 'aaa';

new_fields = struct;
new_fields.bb = 'bbb';
struct_out = setstructfields_ICC(input_structure, new_fields);
assert(strcmp(struct_out.aa, 'aaa'))
assert(strcmp(struct_out.bb, 'bbb'))

% Test: add entry to non-struct
input_structure = 1;
new_fields = struct;
new_fields.aa = 'aaa';
new_fields.ba = 'bbb';
try
    struct_out = setstructfields_ICC(input_structure, new_fields);
    assert(false)
catch exc
    if ~strcmp(exc.identifier, "MATLAB:fieldnames:InvalidInput")
        assert(false)
    end
end
    
% Test: add entry to nested struct
input_structure = struct;
input_nest = struct;
input_nest.aa = 'aaa';
input_nest.bb = 'bbb';
input_structure.nest = input_nest;

new_fields = struct;
new_nest = struct;
new_nest.aa = 'aaaa';
new_nest.cc = 'ccc';
new_fields.nest = new_nest;


struct_out = setstructfields_ICC(input_structure, new_fields);
assert(strcmp(struct_out.nest.aa, 'aaaa'))
assert(strcmp(struct_out.nest.bb, 'bbb'))
assert(strcmp(struct_out.nest.cc, 'ccc'))


% Test: update field in struct

input_structure = struct;
input_structure.aa = 'aaa';

new_fields = struct;
new_fields.aa = 'aaaa';

struct_out = setstructfields_ICC(input_structure, new_fields);
assert(strcmp(struct_out.aa, 'aaaa'))