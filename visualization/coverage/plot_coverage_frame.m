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

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%                          Coverage Plotter                               %
%                                                                         %
%    Plots spacecraft coverage as output by the Monte Carlo coverage      %
%    optimizer                                                            %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [plot_handles] = plot_coverage_frame(varargin)
% PLOT_COVERAGE Plots the instrument coverage of a spacecraft swarm.
% Syntax: [view] = plot_coverage_frame(Swarm, ErosModel, plot_time_index, axes_limits*, color_array*, absolute*)
% *optional input

Swarm = varargin{1};
AsteroidModel = varargin{2};
i_time = varargin{3};
absolute= false;
color_array = ['r', 'b', 'g', 'c', 'm'];
axes_limits = [-1 1 -1 1 -1 1].*40;
if length(varargin) > 3
    for i = 4:2:length(varargin)
        if strcmpi(varargin{i},'color_array') || strcmpi(varargin{i},'colorArray') ||  strcmpi(varargin{i},'color')
            color_array = varargin{i+1};
        end
        if strcmpi(varargin{i},'absolute')
            absolute = varargin{i+1};
        end
        if strcmpi(varargin{i},'axes_limits') || strcmpi(varargin{i},'axes')
            axes_limits = varargin{i+1};
            assert(length(axes_limits)==6, "ERROR: axes limits size is incorrect")
        end
        if strcmpi(varargin{i},'figure_handle') || strcmpi(varargin{i},'figure') || strcmpi(varargin{i},'handle')
            fig = varargin{i+1};
        end
    end
end
if exist('fig','var')
    current_fig = gcf();
    if current_fig ~= fig
        figure(fig);
    end
end

% Which trajectories do we use? This will help make the plotting code
% cleaner
if absolute == true
    sc_trajectories = Swarm.abs_trajectory_array;
else
    sc_trajectories = Swarm.rel_trajectory_array;
end

% Time
plot_time = Swarm.sample_times(i_time);

% Draw the asteroid in its new location
if absolute == true
    h_ast = render_asteroid_3d(AsteroidModel, true, Swarm.sample_times(i_time));
else
    h_ast = render_asteroid_3d(AsteroidModel, false);
end

% Plot observable points
h_os = plot_observable_points(Swarm, AsteroidModel, i_time, 'spacecraft_ids', [1:1:Swarm.get_num_spacecraft()], 'absolute', absolute);
% Actually observed points - if any
h_op = plot_observed_points(Swarm, AsteroidModel, [1, i_time], 'spacecraft_ids', [1:1:Swarm.get_num_spacecraft()], 'absolute', absolute, 'color_array', color_array);
% Line from observer to observed point
h_line = plot_observation_ray(Swarm, AsteroidModel, i_time, 'spacecraft_ids', [1:1:Swarm.get_num_spacecraft()], 'absolute', absolute, 'color_array', color_array);
% Spacecraft trajectory
h_sc = render_spacecraft_3d(sc_trajectories(1:i_time,:,:)./1000, 'color', color_array, 'linewidth', 0.75, 'absolute', absolute, 'plot_time', plot_time);

plot_handles = cell(5,1);
plot_handles{1} = h_ast;
if exist('h_os','var') && ~isempty(h_os)
    plot_handles{2} = h_os;   % Optional
end
if exist('h_op','var') && ~isempty(h_op)
    plot_handles{3} = h_op;   % Optional
end
if exist('h_line','var') && ~isempty(h_line)
    plot_handles{4} = h_line; % Optional
end
plot_handles{5} = h_sc;

end