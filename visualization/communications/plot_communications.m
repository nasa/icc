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



function [] = plot_communications(varargin)
% PLOT_COMMUNICATIONS A function to plot the output of the communication optimizer
% Syntax: [] = plot_communications(swarm, gravity_model, record_video*, videoname*, absolute*, color_array*)
% * Optional keyword inputs


swarm = varargin{1};
gravity_model = varargin{2};
absolute= false;
color_array = ['c' 'r' 'b' 'g' 'm'];

record_video = false;

videoname = ['plot_communications_',datestr(now,'yyyymmdd_HHMMSS'),'.avi'];

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
    

abs_trajectory_array = swarm.abs_trajectory_array/1e3;
rel_trajectory_array = swarm.rel_trajectory_array/1e3;
if absolute == true
    trajectory_array = abs_trajectory_array;
else
    trajectory_array = rel_trajectory_array;
end

n_spacecraft = swarm.get_num_spacecraft();
n_timesteps = swarm.get_num_timesteps();
%time_vector = swarm.sample_times;

% How big do we have to go?
fig = figure();
for sc =1:n_spacecraft
    plot3(trajectory_array(:, 1, sc),trajectory_array(:, 2, sc),trajectory_array(:, 3, sc),'LineWidth',1);
    x0 = 100;
    y0 = 100;
    width = 1920;
    height = 1080;
    set(gcf,'position',[x0,y0,width,height])
	hold all;
end
V = axis();

cla;
figure(fig);
axis equal
axis(V);

tmp_bandwidths = swarm.Communication.bandwidths_and_memories;
tmp_memories = zeros(n_timesteps, n_spacecraft);
for t = 1:n_timesteps
    for sc = 1:n_spacecraft
        tmp_memories(t,sc) = tmp_bandwidths(t, sc, sc);
        tmp_bandwidths(t, sc, sc) = 0;
    end
end

max_bandwidth = max(max(max(tmp_bandwidths)));
max_memory = max(max(tmp_memories));

if record_video
    writerObj = VideoWriter(videoname);
    writerObj.FrameRate = 30;
    open(writerObj);
end


for time = 1:n_timesteps
    fig_handles = plot_communications_frame(swarm, gravity_model, time, 'color_array', color_array, 'absolute', absolute, 'figure_handle', fig, 'max_memory', max_memory, 'max_bandwidth', max_bandwidth);
    if record_video
        writeVideo(writerObj,getframe);
    end
    drawnow limitrate
    pause(0.125)
    for handle_ix = 1:length(fig_handles)
        if ~isempty(fig_handles{handle_ix})
            delete(fig_handles{handle_ix});
        end
    end
end

if record_video
    close(writerObj);
end

plot_communications_frame(swarm, gravity_model, n_timesteps, 'color_array', color_array, 'absolute', absolute, 'figure_handle', fig, 'max_memory', max_memory, 'max_bandwidth', max_bandwidth);