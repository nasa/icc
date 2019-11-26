%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

function [] = create_input_commands(Swarm, output_folder)
%CREATE_INPUT_COMMANDS Creates file Inp_Cmd.txt for 42 simulation from a
% Swarm object
% Syntax: [] = create_input_commands(Swarm, output_folder)
% The function creates the file Inp_Cmd.txt and specifies an initial
% attitude for all spacecraft. The file is placed in output_folder.


if nargin<2
    output_folder='Eros';
end

[mkdir_success, mkdir_warn] = mkdir(output_folder);

inp_cmd_header = [...
	"<<<<<<<<<<<<<<<<<  42:  Command Script File  >>>>>>>>>>>>>>>>>", ...
];

inp_cmd_pointing = [];

for sc_index = 0:Swarm.get_num_spacecraft() - 1
	inp_cmd_pointing = [inp_cmd_pointing, ...
        sprintf("0.0 Align SC[%d].B[0] Primary Vector [0.0 0.0 1.0] with L-frame Vector [1.0 0.0 1.0]",sc_index), ...
        sprintf("0.0 Point SC[%d].B[0] Secondary Vector [1.0 0.0 0.0] at SUN",sc_index), ...
	];
end

inp_cmd_footer = [...
    "", ...
    "EOF"...
];

inp_cmd_file = fopen(fullfile(output_folder,"Inp_Cmd.txt"),'w');
for line = inp_cmd_header
    fprintf(inp_cmd_file, "%s\n", line );
end
for line = inp_cmd_pointing
    fprintf(inp_cmd_file, "%s\n", line );
end
for line = inp_cmd_footer
    fprintf(inp_cmd_file, "%s\n", line );
end
fclose(inp_cmd_file);