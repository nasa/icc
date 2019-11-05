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

function [h_op] = plot_observed_points(varargin)
% PLOT_OBSERVED_POINTS plots observed points
% Syntax: [handle] = plot_observed_points(Swarm, AsteroidModel,
% [start_time_index end_time_index], spacecraft_ids*
% absolute_frame_flag*, color_array*)
% * Optional keyword args

Swarm = varargin{1};
AsteroidModel = varargin{2};
start_time = varargin{3}(1);
i_time = varargin{3}(2);
spacecraft_ids = [1:1:Swarm.get_num_spacecraft()];
absolute= false;
color_array = ['r', 'b', 'g', 'c', 'm'];
if length(varargin) >= 4
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

for i_sc = spacecraft_ids
    % Actually observed points - if any
    if sum(Swarm.Observation.observed_points(i_sc, start_time:i_time))>0
        % Note that, for some time steps, there may be no
        % points to observe. render_these_points_3d takes care
        % of filtering those out
        h_op(i_sc) = render_these_points_3d(AsteroidModel, Swarm.Observation.observed_points(i_sc, start_time:i_time), 'color', color_array(:,mod(i_sc-1,size(color_array,2))+1)', 'markersize', 6, 'absolute', absolute, 'plot_time', plot_time);

    end
end
if ~exist('h_op','var')
    h_op = gobjects(0); %Empty handle
end
