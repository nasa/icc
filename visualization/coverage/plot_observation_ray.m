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

function [h_line] = plot_observation_ray(varargin)
% PLOT_OBSERVATION_RAY plots ray from spacecraft to observed point
% Syntax: [handle] = plot_observation_ray(Swarm, AsteroidModel, time_index, spacecraft_ids*, absolute_frame_flag*, color_array*).
% * Optional keyword args


Swarm = varargin{1};
AsteroidModel = varargin{2};
i_time = varargin{3};
spacecraft_ids = [1:1:Swarm.get_num_spacecraft()];
absolute= false;
color_array = ['r']; %, 'b', 'g', 'c', 'm'];
if length(varargin) > 3
    for i = 4:2:length(varargin)
        if strcmpi(varargin{i},'spacecraft_ids')
            spacecraft_ids = varargin{i+1};
        end
        if strcmpi(varargin{i},'absolute') || strcmpi(varargin{i},'absolute_frame_flag')
            absolute = varargin{i+1};
        end
        if strcmpi(varargin{i},'color_array') || strcmpi(varargin{i},'colorArray') ||  strcmpi(varargin{i},'color')
            color_array = varargin{i+1};
        end
    end
end
plot_time = Swarm.sample_times(i_time);

% Trajectory and body vertices
if absolute == true
    sc_trajectories = Swarm.abs_trajectory_array;
    rot_angle_z = AsteroidModel.BodyModel.bodyFrame.pm.w0+ AsteroidModel.BodyModel.bodyFrame.pm.w * plot_time;
    Rot_i2b = rotmat(rot_angle_z,3);
    body_vertices = (Rot_i2b'*AsteroidModel.BodyModel.shape.vertices')';
else
    sc_trajectories = Swarm.rel_trajectory_array;
    body_vertices =  AsteroidModel.BodyModel.shape.vertices;
end

for i_sc = spacecraft_ids
    if Swarm.Observation.observed_points(i_sc, i_time)>0
        h_line(i_sc) = plot3([sc_trajectories(i_time,1,i_sc)./1000, body_vertices(Swarm.Observation.observed_points(i_sc,i_time),1)], ...
            [sc_trajectories(i_time,2,i_sc)./1000, body_vertices(Swarm.Observation.observed_points(i_sc,i_time),2)],...
            [sc_trajectories(i_time,3,i_sc)./1000, body_vertices(Swarm.Observation.observed_points(i_sc,i_time),3)],'--',...
            'color',color_array(:,mod(Swarm.Parameters.types{i_sc},size(color_array,2))+1)','linewidth',0.5);
    end
end
if ~exist('h_line','var')
    h_line = gobjects(0); %Empty handle
end
