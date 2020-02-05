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
%                 Coverage and Communications Plotter                     %
%                                                                         %
%    Plots spacecraft coverage and communications as output by the        %
%    ICC optimizer                                                        %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [plot_handles] = plot_coverage_and_communications_frame(varargin)
% PLOT_COVERAGE_AND_COMMUNICATIONS Plots both coverage and comms.
% Syntax: [view] = plot_coverage_and_communications_frame(Swarm, ErosModel, time_step, axes_limits*, color_array*, absolute*, standard_font_size*, title_font_size*, font_name*, min_line_thickness*, max_line_thickness*, max_memory_marker_size*, link_color_steps*)
% *optional input

Swarm = varargin{1};
AsteroidModel = varargin{2};
time_step = varargin{3};

% Defaults
absolute = true;
color_array = ['r', 'b', 'g', 'c', 'm']; 
min_line_thickness = 1;
max_line_thickness = 20;
max_memory_marker_size = 40;
link_color_steps = 100;
title_font_size = 30;
standard_font_size = 25;
font_name = 'Times New Roman';


if length(varargin) > 3
    for i = 4:1:length(varargin)
        if strcmpi(varargin{i},'color_array') || strcmpi(varargin{i},'colorArray') ||  strcmpi(varargin{i},'color')
            color_array = varargin{i+1};
        end
        if strcmpi(varargin{i},'absolute')
            absolute = varargin{i+1};
        end
        if strcmpi(varargin{i},'font_size') || strcmpi(varargin{i},'fontsize') || strcmpi(varargin{i},'standard_font_size')
            standard_font_size = varargin{i+1};
        end
        if strcmpi(varargin{i},'title_font_size')
            title_font_size = varargin{i+1};
        end
        if strcmpi(varargin{i},'font_name')
            font_name = varargin{i+1};
        end
        if strcmpi(varargin{i},'min_line_thickness')
            min_line_thickness = varargin{i+1};
        end
        if strcmpi(varargin{i},'max_line_thickness')
            max_line_thickness = varargin{i+1};
        end
        if strcmpi(varargin{i},'max_memory_marker_size')
            max_memory_marker_size = varargin{i+1};
        end
        if strcmpi(varargin{i},'link_color_steps')
            link_color_steps = varargin{i+1};
        end
    end
end


% Time
plot_time = Swarm.sample_times(time_step);

% if fontSizeSpecified==false
%     title_font_size = 30;
% end

h_title = title(['Time = ', num2str(floor(plot_time/8640)/10), ' day '],'fontsize',title_font_size,'fontname',font_name);


% Draw the asteroid in its new location
h_ast = render_asteroid_3d(AsteroidModel, absolute, Swarm.sample_times(time_step));

% Plot the spacecraft trajectories
h_sc = render_spacecraft_3d(Swarm, time_step,...
    'color', color_array, 'linewidth', 0.75, 'absolute', absolute, ...
    'show_trail', true, 'markersize', 0, ...
    'LineWidth',min_line_thickness);

% Coverage

% Plot observable points
h_os = plot_observable_points(Swarm, AsteroidModel, time_step, 'spacecraft_ids', [1:1:Swarm.get_num_spacecraft()], 'absolute', absolute);
% Actually observed points - if any
h_op = plot_observed_points(Swarm, AsteroidModel, [1, time_step], 'spacecraft_ids', [1:1:Swarm.get_num_spacecraft()], 'absolute', absolute, 'color_array', color_array);
% Line from observer to observed point
h_line = plot_observation_ray(Swarm, AsteroidModel, time_step, 'spacecraft_ids', [1:1:Swarm.get_num_spacecraft()], 'absolute', absolute, 'color_array', color_array);

% Communications

% Plot spacecraft markers with memory use
h_mem = plot_memory_use(Swarm, time_step, 'color_array', color_array, 'absolute', absolute, 'max_memory_marker_size', max_memory_marker_size);

% Plot information flow
h_bw = plot_information_flow(Swarm, time_step, ...
    'color_array',color_array, 'absolute', absolute, ...
    'min_line_thickness', min_line_thickness, ...
    'max_line_thickness', max_line_thickness, ...
    'link_color_steps', link_color_steps);

% Plot the Sun
sun_state = get_sun_state(Swarm.sample_times(time_step), 'absolute', absolute);
sun_pos = 100*sun_state(1:3)/(norm(sun_state(1:3)));
h_sun = plot3(sun_pos(1), sun_pos(2), sun_pos(3), 'o','MarkerFaceColor','y','MarkerEdgeColor','k','MarkerSize',15);


plot_handles = cell(9,1);
plot_handles{1} = h_ast;
plot_handles{2} = h_sc;
if exist('h_os','var') && ~isempty(h_os)
    plot_handles{3} = h_os;   % Optional
end
if exist('h_op','var') && ~isempty(h_op)
    plot_handles{4} = h_op;   % Optional
end
if exist('h_line','var') && ~isempty(h_line)
    plot_handles{5} = h_line; % Optional
end
if exist('h_mem','var') && ~isempty(h_mem)
    plot_handles{6} = h_mem; % Optional
end
if exist('h_bw','var') && ~isempty(h_bw)
    plot_handles{7} = h_bw; % Optional
end
if exist('h_title','var') && ~isempty(h_title)
    plot_handles{8} = h_title; % Optional
end
if exist('h_sun','var') && ~isempty(h_sun)
    plot_handles{9} = h_sun; % Optional
end


end