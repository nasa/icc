%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%     Plotting tools for the network flow communication optimizer         %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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



function [fig_handles] = plot_communications_frame(varargin)
% PLOT_COMMUNICATIONS_FRAME A function to plot the output of the communication optimizer
% Syntax: [axes_limits] = plot_communications_frame(swarm, gravity_model, plot_time, color_array*, absolute*, figure_handle*, max_memory*, max_bandwidth*)
% * Optional keyword inputs

swarm = varargin{1};
gravity_model = varargin{2};
time = varargin{3};

absolute = false;
n_spacecraft = swarm.get_num_spacecraft();
color_array = rand(3,n_spacecraft);
max_memory = NaN;
max_bandwidth = NaN;

if length(varargin) > 3
    for i = 4:2:length(varargin)
        if strcmpi(varargin{i},'color_array') || strcmpi(varargin{i},'colorArray') ||  strcmpi(varargin{i},'color')
            color_array = varargin{i+1};
        end
        if strcmpi(varargin{i},'absolute')
            absolute = varargin{i+1};
        end
        if strcmpi(varargin{i},'figure_handle') || strcmpi(varargin{i},'handle') || strcmpi(varargin{i},'figure')
            fig = varargin{i+1};
        end
        if strcmpi(varargin{i},'max_memory')
            max_memory = varargin{i+1};
        end
        if strcmpi(varargin{i},'max_bandwidth')
            max_bandwidth = varargin{i+1};
        end
    end
end

abs_trajectory_array = swarm.abs_trajectory_array/1e3;
rel_trajectory_array = swarm.rel_trajectory_array/1e3;
if absolute == true
    trajectory_array = abs_trajectory_array;
else
    trajectory_array = rel_trajectory_array;
end

min_line_thickness = 1;
max_line_thickness = 20;
max_memory_marker_size = 40;
link_color_steps = 100;

if exist('fig', 'var')
    current_fig = gcf();
    if current_fig ~= fig
        figure(fig);
    end
end

% Plot asteroid
h_ast = render_asteroid_3d(gravity_model, absolute, swarm.sample_times(time));

% Plot the spacecraft trajectories
h_tr = render_spacecraft_3d(trajectory_array(1:time,:,:),...
    'color', color_array, 'linewidth', 0.75, 'absolute', absolute, ...
    'show_trail', true, 'markersize', 0, ...
    'LineWidth',min_line_thickness);

% Plot spacecraft markers with memory use
h_sc = plot_memory_use(swarm, time, ...
    'color_array', color_array, 'absolute', absolute, ...
    'figure_handle', fig, 'max_memory', max_memory, ...
        'max_memory_marker_size', max_memory_marker_size);
% Plot information flow
h_bw = plot_information_flow(swarm, time, ...
    'color_array',color_array, 'absolute', absolute, ...
    'figure_handle', fig, 'max_bandwidth', max_bandwidth, ...
    'min_line_thickness', min_line_thickness, ...
    'max_line_thickness', max_line_thickness, ...
    'link_color_steps', link_color_steps);
fig_handles = cell(5,1);
fig_handles{1} = h_tr;
fig_handles{2} = h_sc;
fig_handles{3} = h_bw;
fig_handles{4} = h_ast;

end
