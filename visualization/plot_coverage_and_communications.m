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

function [] = plot_coverage_and_communications(varargin)
% plot_coverage_and_communications A function to plot both coverage and
% communications.
% Syntax: [] = plot_coverage_and_communications(Swarm, ErosModel, color_array*, absolute*, record_video*, videoname*)
% * Optional keyword inputs

Swarm = varargin{1};
AsteroidModel = varargin{2};
absolute = true;
color_array = ['r', 'b', 'g', 'c', 'm']; %rand(3,Swarm.get_num_spacecraft());
record_video = false;
videoname = ['ICC_simulation_',datestr(now,'yyyymmdd_HHMMSS'),'.mp4'];

if length(varargin) > 2
    for i = 3:2:length(varargin)
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
initialize_spatial_plot_3d();
hold on 
axis equal
% Initial plot - just to get a sense of the size
plot_coverage_and_communications_frame(Swarm, AsteroidModel,length(Swarm.sample_times), 'absolute', absolute, 'figure_handle', h1, 'color_array', color_array);
axis equal
three_d_plot_axes = axis();
clf;

for time_step = 1:length(Swarm.sample_times)
    subplot(2,4,[1 2 5 6]);
    if time_step == 1
        initialize_spatial_plot_3d();
        axis(three_d_plot_axes);
        axis equal
    end

    plot_handles = plot_coverage_and_communications_frame(Swarm, AsteroidModel, time_step, 'absolute', absolute,'color_array', color_array);
    
    subplot(2,4,3)
    render_observed_points_2d(AsteroidModel, Swarm, 'above', 'time_limits', [1, time_step],'color_array', color_array) % Show which points have been observed above equator
    
    subplot(2,4,4)
    render_observed_points_2d(AsteroidModel, Swarm, 'below', 'time_limits', [1, time_step],'color_array', color_array) % Show which points have been observed above equator
    
    subplot(2,4,7)
    plot_memory_comparison_2d(time_step, Swarm, 'semilogflag', true,'color_array', color_array);
    
    subplot(2,4,8)
    plot_communication_topology_2d(time_step, Swarm, AsteroidModel,color_array);
    
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
plot_coverage_and_communications_frame(Swarm, AsteroidModel,length(Swarm.sample_times), 'absolute', absolute, 'figure_handle', h1, 'color_array', color_array);

if record_video
    close(writerObj);
end

