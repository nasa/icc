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

function [ ] = plot_communication_topology_2d(varargin) %i_time, Swarm, AsteroidModel, color_array, font_size, max_line_thickness, max_bandwidth)
%PLOT_COMMUNICATION_TOPOLOGY_2D Plots a 2D projection of the spacecraft
%locations with lines indicating the data flow on the current iteration.
%The asteroid is approximated by a spherical outer bound.
%   Syntax: [] = plot_communication_topology(time_step, Swarm, AsteroidModel, color_array*, font_size*, max_bandwidth*)
%    *optional input

time_step = varargin{1};
Swarm = varargin{2};
AsteroidModel = varargin{3};

if length(varargin) > 3
    for i = 4:1:length(varargin)
        if strcmpi(varargin{i},'color_array') || strcmpi(varargin{i},'colorArray') ||  strcmpi(varargin{i},'color')
            color_array = varargin{i+1};
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

n_timesteps = Swarm.get_num_timesteps;
n_spacecraft = Swarm.get_num_spacecraft;

cla()
hold on
% Plot the radius encircling the asteroid
asteroid_max_radius = AsteroidModel.BodyModel.shape.maxRadius;
asteroid_mean_radius = AsteroidModel.BodyModel.shape.meanRadius;
asteroid_min_radius = AsteroidModel.BodyModel.shape.minRadius;

theta = 0:pi/50:2*pi;
plot(asteroid_max_radius * cos(theta), asteroid_max_radius * sin(theta),':k','LineWidth',1.0);
plot(asteroid_mean_radius * cos(theta), asteroid_mean_radius * sin(theta),'-k','LineWidth',2.0);
patch(asteroid_min_radius * cos(theta), asteroid_min_radius * sin(theta),'k');


% Communications

absolute = true;

% Plot spacecraft markers with memory use
h_mem = plot_memory_use(Swarm, time_step, ...
    'color_array', color_array, 'absolute', absolute, ...
        'max_memory_marker_size', max_memory_marker_size);

% Plot information flow
h_bw = plot_information_flow(Swarm, time_step, ...
    'color_array',color_array, 'absolute', absolute, ...
    'min_line_thickness', min_line_thickness, ...
    'max_line_thickness', max_line_thickness, ...
    'link_color_steps', link_color_steps);

view(2)
xlabel('X Axis [km]','fontsize',standard_font_size, 'fontname',font_name)
ylabel('Y Axis [km]','fontsize',standard_font_size, 'fontname',font_name)
title('Communication Topology','fontsize',title_font_size, 'fontname',font_name)
set(gca, 'fontsize',standard_font_size, 'fontname',font_name)
xlim = (1e-3)*[min(min(Swarm.abs_trajectory_array(:,1,:))), max(max(Swarm.abs_trajectory_array(:,1,:)))];
ylim = (1e-3)*[min(min(Swarm.abs_trajectory_array(:,2,:))), max(max(Swarm.abs_trajectory_array(:,2,:)))];
axis([xlim, ylim])
axis equal

end