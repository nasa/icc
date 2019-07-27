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



function [] = plot_communications(swarm, gravity_model, record_video, videoname)

% A function to plot the output of the communication optimizer

POSTER_FLAG = false;
if POSTER_FLAG
    min_line_thickness = 2;
    max_line_thickness = 30;
    max_memory_marker_size = 40;
    link_color_steps = 100;
else
    min_line_thickness = 1;
    max_line_thickness = 20;
    max_memory_marker_size = 40;
    link_color_steps = 100;
end

% for sc = 1:length(spacecraft.orbits)
%     spacecraft.orbits{sc} = spacecraft.orbits{sc}/1e3; %Convert to km
% end

abs_trajectory_array = swarm.abs_trajectory_array/1e3;
rel_trajectory_array = swarm.rel_trajectory_array/1e3;

n_spacecraft = swarm.get_num_spacecraft();
n_timesteps = swarm.get_num_timesteps();
%time_vector = swarm.sample_times;

% How big do we have to go?
fig = figure();
for sc =1:n_spacecraft
	%plot3(spacecraft.orbits{sc}(1,:),spacecraft.orbits{sc}(2,:),spacecraft.orbits{sc}(3,:),'LineWidth',min_line_thickness);
    plot3(abs_trajectory_array(:, 1, sc),abs_trajectory_array(:, 2, sc),abs_trajectory_array(:, 3, sc),'LineWidth',min_line_thickness);
    x0 = 100;
    y0 = 100;
    width = 1920;
    height = 1080;
    set(gcf,'position',[x0,y0,width,height])
	hold all;
end
hold off
% cam = struct();
% cam.pos = campos;
% cam.target = camtarget;
% cam.va = camva;
% cam.up = camup;
V = axis();


tmp_bandwidths = swarm.Communication.bandwidths_and_memories;
tmp_memories = zeros(n_timesteps, n_spacecraft);
for t = 1:n_timesteps
    for sc = 1:n_spacecraft
        tmp_memories(t,sc) = tmp_bandwidths(t, sc, sc);
        tmp_bandwidths(t, sc, sc) = 0;
    end
end

flows_to_carrier = swarm.Communication.flow(:,:,end);
flows_from_carrier = squeeze(swarm.Communication.flow(:,end,:));
delivered_science = zeros(n_timesteps,1);
delivered_science(2:end) = sum(flows_to_carrier(1:end-1,:),2)-sum(flows_from_carrier(2:end,:),2)+swarm.Communication.effective_source_flow(end,1:end-1)';


max_bandwidth = max(max(max(tmp_bandwidths)));


max_memory = max(max(tmp_memories));
max_delivered = sum(sum(swarm.Communication.effective_source_flow));

if nargin<2
    plot_body = false;
else
    plot_body = true;
end

if nargin<3
    record_video = false;
end

if nargin<4
    videoname = ['plot_communications',datestr(now,'yyyymmdd_HHMMSS'),'.avi'];
end

if record_video
    writerObj = VideoWriter(videoname);
    writerObj.FrameRate = 30;
    open(writerObj);
end

sc_colors = rand(n_spacecraft,3);


link_colors = [summer(link_color_steps/2);flipud(autumn(link_color_steps/2))]; %flipud(hot(link_color_steps));

bandwidth_duals = swarm.Communication.dual_bandwidths_and_memories;
memory_duals = zeros(n_timesteps, n_spacecraft);
for k=1:n_timesteps
    for i=1:n_spacecraft
        bandwidth_duals(k,i,i) = 0;
        memory_duals(k,i) = swarm.Communication.dual_bandwidths_and_memories(k,i,i);
    end
end

max_bandwidth_duals = max(max(max(bandwidth_duals)));
min_bandwidth_duals = min(min(min(bandwidth_duals)));

for time = 1:n_timesteps
    if plot_body
        % Just to get the body
        gravity_model.plot_absolute_traj(swarm.sample_times(time), abs_trajectory_array(time, 1:3, 1), true, fig)
        hold all
    end
    
    
    for sc1 = 1:n_spacecraft
        % Plot the trajectory
        %plot3(spacecraft.orbits{sc1}(1,1:time),spacecraft.orbits{sc1}(2,1:time),spacecraft.orbits{sc1}(3,1:time),...
        plot3(abs_trajectory_array(1:time, 1, sc1), abs_trajectory_array(1:time, 2, sc1), abs_trajectory_array(1:time, 3, sc1), ...
            'Color',sc_colors(sc1,:), ...
            'LineWidth',min_line_thickness);
        
        
        hold all;
        
        % Plot the s/c location and memory use
        curr_memory = swarm.Communication.bandwidths_and_memories(time,sc1,sc1);
        if sc1 == n_spacecraft
            % If the sc is the carrier, also count the science we delivered
            curr_memory = curr_memory+sum(delivered_science(1:time));
            ssymbol = 's';
            ssize = curr_memory/max_delivered*max_memory_marker_size +1;
        else
            ssymbol = '.';
            ssize = curr_memory/max_memory*max_memory_marker_size +1;
        end
        % plot3(spacecraft.orbits{sc1}(1,time),spacecraft.orbits{sc1}(2,time),spacecraft.orbits{sc1}(3,time), ...
        plot3(abs_trajectory_array(time, 1, sc1), abs_trajectory_array(time, 2, sc1), abs_trajectory_array(time, 3, sc1), ...
            ssymbol,'markersize',ssize, 'MarkerFaceColor',sc_colors(sc1,:));
        hold all;
        
        for sc2 = 1:n_spacecraft
            link_color_index = ceil(bandwidth_duals(time,sc1,sc2)/(max_bandwidth_duals-min_bandwidth_duals)*(link_color_steps-1)+1);
            if isnan(link_color_index) || link_color_index<1
                warning("Problem is not communication-limited at all: all bandwidth duals are zero");
                link_color_index = 1;
            end
            % Plot the bandwidths
%             lh = plot3([spacecraft.orbits{sc1}(1,time),spacecraft.orbits{sc2}(1,time)], ...
%                 [spacecraft.orbits{sc1}(2,time),spacecraft.orbits{sc2}(2,time)], ...
%                 [spacecraft.orbits{sc1}(3,time),spacecraft.orbits{sc2}(3,time)], ...
            lh = plot3([abs_trajectory_array(time, 1, sc1), abs_trajectory_array(time, 1, sc2)], ...
                [abs_trajectory_array(time, 2, sc1), abs_trajectory_array(time, 2, sc2)], ...
                [abs_trajectory_array(time, 3, sc1), abs_trajectory_array(time, 3, sc2)], ...
                'Color','k', ...
                'linewidth', (swarm.Communication.bandwidths_and_memories(time,sc1,sc2))/max_bandwidth*max_line_thickness+min_line_thickness);
            lh.Color = [link_colors(link_color_index,:),.3];
            
            % Plot the actual info flow
            if swarm.Communication.flow(time,sc1,sc2)>0
                plot3([abs_trajectory_array(time, 1, sc1), abs_trajectory_array(time, 1, sc2)], ...
                    [abs_trajectory_array(time, 2, sc1), abs_trajectory_array(time, 2, sc2)], ...
                    [abs_trajectory_array(time, 3, sc1), abs_trajectory_array(time, 3, sc2)], ...
                    ':', ...
                    'Color', link_colors(link_color_index,:), ...
                    'linewidth', (swarm.Communication.flow(time,sc1,sc2))/max_bandwidth*max_line_thickness+min_line_thickness);
                hold all;
            end
            
        end
    end
% %     campos(cam.pos);
% %     camtarget(cam.target);
% %     camva(cam.va);
% %     camup(cam.up);
    axis(V);
    if record_video
        writeVideo(writerObj,getframe);
    end
    hold off;
end

if record_video
    close(writerObj);
end