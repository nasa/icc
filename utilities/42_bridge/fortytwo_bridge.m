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

function [] = fortytwo_bridge(Swarm, Small_body_model, defaults_path, output_folder, time_step, graphics_front_end)
%FORTYTWO_BRIDGE Creates input files for 42 simulation from a Swarm
% Syntax: [] = fortytwo_bridge(Swarm, Small_body_model, output_folder, time_step, graphics_front_end)
% The function creates a file Inp_Sim.txt describing the simulation
% environment and, for every orbit, a file Orb_scX.txt containing
% spacecraft properties. The files are placed in the folder output_folder.

if nargin<6
    graphics_front_end="TRUE";
end

if nargin<5
    time_step=0.1;
end

if nargin<4
    output_folder = strcat("42_EROS_ICC_",datestr(now,'yyyymmdd_HHMMSS'));
end

if nargin<3
    defaults_path = "defaults";
end

mkdir_state = mkdir(output_folder);

% Build the file Inp_Sim.txt
create_input_sim(Swarm, output_folder, time_step, graphics_front_end);

% Build the spacecraft orbit files
create_spacecraft(Swarm, Small_body_model, output_folder);

% Build the commands file
create_input_commands(Swarm, output_folder);

% If we have access to the defaults, copy them
try
    % FOV
    copyfile(fullfile(defaults_path,"Inp_FOV.txt"), output_folder);

    % Graphics
    copyfile(fullfile(defaults_path,"Inp_Graphics.txt"), output_folder);

    % IPC
    copyfile(fullfile(defaults_path,"Inp_IPC.txt"), output_folder);

    % Region
    copyfile(fullfile(defaults_path,"Inp_Region.txt"), output_folder);

    % TRDS
    copyfile(fullfile(defaults_path,"Inp_TDRS.txt"), output_folder);

catch
   warning(sprintf(...
       "Folder %s not found.\n" + ...
       "You should create or copy the 42 configuration files for\n" + ...
       "- FOV: Inp_FOV.txt \n" + ...
       "- Graphics: Inp_Graphics.txt \n" + ...
       "- IPC: Inp_IPC.txt \n" + ...
       "- Regions: Inp_Region.txt \n" + ...
       "- TDRS: Inp_TDRS.txt \n" + ...
       "manually and place them in the folder %s."...
       ,defaults_path,output_folder));
end

end