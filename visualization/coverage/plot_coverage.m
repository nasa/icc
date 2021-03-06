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

function [] = plot_coverage(varargin)
% PLOT_COVERAGE Plots the instrument coverage of a spacecraft swarm.
% Syntax: [] = plot_coverage(Swarm, ErosModel, color_array*, absolute*)
% *optional input

Swarm = varargin{1};
AsteroidModel = varargin{2};
absolute= false;
color_array = ['r', 'b', 'g', 'c', 'm'];
title_font_size = 30;
standard_font_size = 25;
font_name = 'Times New Roman';

if length(varargin) > 2
    for i = 3:2:length(varargin)
        if strcmpi(varargin{i},'color_array') || strcmpi(varargin{i},'colorArray') ||  strcmpi(varargin{i},'color')
            color_array = varargin{i+1};
        end
        if strcmpi(varargin{i},'absolute')
            absolute = varargin{i+1};
        end
    end
end

% Initialize figure and estimate how big it should be
h1=figure();
set(h1,'Color',[1 1 1]);
set(h1,'units','normalized','outerposition',[0 0 1 1])
set(h1,'PaperPositionMode','auto');
initialize_spatial_plot_3d('standard_font_size', standard_font_size, 'font_name', font_name);
hold on 
% Initial plot - just to get a sense of the size
plot_handles = plot_coverage_frame(Swarm, AsteroidModel,length(Swarm.sample_times), 'absolute', absolute, 'figure_handle', h1, 'color_array', color_array, 'title_font_size', title_font_size, 'standard_font_size', standard_font_size, 'font_name', font_name);
axis equal
three_d_plot_axes = axis();

for time_step = 1:length(Swarm.sample_times)
    
    for entry_ix = 1:length(plot_handles)
        if ~isempty(plot_handles{entry_ix})
            delete(plot_handles{entry_ix})
        end
    end
    
    if time_step == 1
        initialize_spatial_plot_3d('standard_font_size', standard_font_size, 'font_name', font_name);
        axis(three_d_plot_axes);
        axis equal
    end
    plot_handles = plot_coverage_frame(Swarm, AsteroidModel,time_step, 'absolute', absolute, 'figure_handle', h1, 'color_array', color_array, 'title_font_size', title_font_size, 'standard_font_size', standard_font_size, 'font_name', font_name);
    
    xlabel('X axis [km]','fontsize',standard_font_size, 'fontname',font_name)
    ylabel('Y axis [km]','fontsize',standard_font_size, 'fontname',font_name)
    zlabel('Z axis [km]','fontsize',standard_font_size, 'fontname',font_name)
    set(gca, 'fontsize',standard_font_size, 'fontname',font_name)
    
    grid on;
    view(3)

        
    drawnow limitrate
    pause(0.125);
    
end

