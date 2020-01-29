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

function [h_os] = plot_observable_points(varargin)
% PLOT_OBSERVABLE_POINTS plots observable points
% Syntax: [handle] = plot_observable_points(Swarm, AsteroidModel,
% fig_handle,time_index, spacecraft_ids*, absolute_frame_flag*)
% * Optional keyword args

Swarm = varargin{1};
AsteroidModel = varargin{2};
i_time = varargin{3};
coverage_color = 'y';
spacecraft_ids = [1:1:Swarm.get_num_spacecraft()];
absolute = true;

if length(varargin) >= 4
    for i = 4:2:length(varargin)
        if strcmpi(varargin{i},'spacecraft_ids')
            spacecraft_ids = varargin{i+1};
        end
        if strcmpi(varargin{i},'absolute') || strcmpi(varargin{i},'absolute_frame_flag')
            absolute = varargin{i+1};
        end
        if strcmpi(varargin{i},'coverage_color') || strcmpi(varargin{i},'color')
            coverage_color = varargin{i+1};
        end
    end
end
plot_time = Swarm.sample_times(i_time);

%figure(fig)
for i_sc = spacecraft_ids
    % If the spacecraft had something to observe
    if ~isempty(Swarm.Observation.observable_points{i_sc, i_time})
        % Observable points
        if ~isempty(Swarm.Observation.observable_points(i_sc, i_time))>0
            h_os(i_sc) = render_these_points_3d(AsteroidModel, Swarm.Observation.observable_points{i_sc, i_time}, 'color', coverage_color, 'markersize', 3, 'absolute', absolute, 'plot_time', plot_time); %#ok<*SAGROW>
        end
    end
end
if ~exist('h_os','var')
    h_os = gobjects(0); %Empty handle
end
