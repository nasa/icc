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

function [] = plot_coverage_and_communications_with_insets(varargin)
% plot_coverage_and_communications A function to plot both coverage and communications with insets.
% Syntax: [] = plot_coverage_and_communications_with_insets(Swarm, ErosModel, absolute*, color_array*, record_video*, videoname*)
% * Optional keyword inputs

Swarm = varargin{1};
AsteroidModel = varargin{2};

% Defaults
absolute = true;
color_array = ['r', 'b', 'g', 'c', 'm']; 
record_video = false;
videoname = ['ICC_simulation_',datestr(now,'yyyymmdd_HHMMSS'),'.mp4'];
min_line_thickness = 1;
max_line_thickness = 20;
max_memory_marker_size = 40;
link_color_steps = 100;
title_font_size = 30;
standard_font_size = 25;
font_name = 'Times New Roman';

if length(varargin) > 2
    for i = 3:1:length(varargin)
        if strcmpi(varargin{i},'color_array') || strcmpi(varargin{i},'colorArray') ||  strcmpi(varargin{i},'color')
            color_array = varargin{i+1};
        end
        if strcmpi(varargin{i},'absolute')
            absolute = varargin{i+1};
        end
        if strcmpi(varargin{i},'record_video') || strcmpi(varargin{i},'video') || strcmpi(varargin{i},'save_video')
            record_video = varargin{i+1};
        end
        if strcmpi(varargin{i},'video_name') || strcmpi(varargin{i},'videoname')
            videoname = varargin{i+1};
        end
    end
end

if record_video    
    writerObj = VideoWriter(videoname, 'MPEG-4');
    writerObj.FrameRate = 30;   % Default 30
    writerObj.Quality = 100;    % Default 75
    open(writerObj);
end

% Test 3d plot to get the axes extent
h1 = figure();
set(h1,'Color',[1 1 1]);
set(h1,'units','normalized','outerposition',[0 0 1 1])
set(h1,'PaperPositionMode','auto');
initialize_spatial_plot_3d('standard_font_size', standard_font_size, 'font_name', font_name);
hold on 

axis equal
% Initial plot - just to get a sense of the size
plot_coverage_and_communications_frame(Swarm, AsteroidModel,length(Swarm.sample_times), 'absolute', absolute, 'color_array', color_array, 'title_font_size', title_font_size, 'standard_font_size', standard_font_size, ... 
    'font_name', font_name, 'min_line_thickness',min_line_thickness, 'max_line_thickness',max_line_thickness, 'max_memory_marker_size',max_memory_marker_size, 'link_color_steps',link_color_steps);
axis equal
three_d_plot_axes = axis();

clf;

for time_step = 1:length(Swarm.sample_times)
    subplot(2,4,[1 2 5 6]);
    if time_step == 1
        initialize_spatial_plot_3d('standard_font_size', standard_font_size, 'font_name', font_name);
        axis(three_d_plot_axes);
        axis equal
    end

    plot_handles = plot_coverage_and_communications_frame(Swarm, AsteroidModel, time_step, 'absolute', absolute,'color_array', color_array, 'title_font_size', title_font_size, 'standard_font_size', standard_font_size, ... 
    'font_name', font_name, 'min_line_thickness',min_line_thickness, 'max_line_thickness',max_line_thickness, 'max_memory_marker_size',max_memory_marker_size, 'link_color_steps',link_color_steps);
    
    subplot(2,4,3)
    render_observed_points_2d(AsteroidModel, Swarm, 'above', 'time_limits', [1, time_step],'color_array', color_array, 'title_font_size', title_font_size, 'standard_font_size', standard_font_size, 'font_name', font_name) % Show which points have been observed above equator
    
    subplot(2,4,4)
    render_observed_points_2d(AsteroidModel, Swarm, 'below', 'time_limits', [1, time_step],'color_array', color_array, 'title_font_size', title_font_size, 'standard_font_size', standard_font_size, 'font_name', font_name) % Show which points have been observed above equator
    
    subplot(2,4,7)
    plot_memory_comparison_2d(time_step, Swarm, 'semilogflag', true,'color_array', color_array, 'title_font_size', title_font_size, 'standard_font_size', standard_font_size, 'font_name', font_name);
    
    subplot(2,4,8)
    plot_communication_topology_2d(time_step, Swarm, AsteroidModel, 'color_array', color_array, 'title_font_size', title_font_size, 'standard_font_size', standard_font_size, 'font_name', font_name, ...
        'min_line_thickness',min_line_thickness, 'max_line_thickness',max_line_thickness, 'max_memory_marker_size',max_memory_marker_size, 'link_color_steps',link_color_steps);
    
    drawnow limitrate
    pause(0.125);
    if record_video
        F = getframe(h1);
        writeVideo(writerObj,F);
    end
    
    for entry_ix = 1:length(plot_handles)
        if ~isempty(plot_handles{entry_ix})
            delete(plot_handles{entry_ix})
        end
    end
    
    
end

subplot(2,4,[1 2 5 6]);
plot_coverage_and_communications_frame(Swarm, AsteroidModel,length(Swarm.sample_times), 'absolute', absolute, 'color_array', color_array, 'title_font_size', title_font_size, 'standard_font_size', standard_font_size, ... 
    'font_name', font_name, 'min_line_thickness',min_line_thickness, 'max_line_thickness',max_line_thickness, 'max_memory_marker_size',max_memory_marker_size, 'link_color_steps',link_color_steps);

if record_video
    close(writerObj);
end

