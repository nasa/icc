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



function [] = plot_communications(spacecraft, flows, effective_science, delivered_science, bandwidths, bandwidth_and_memory_duals, gravity_model, record_video, videoname)

% A function to plot the output of the communication optimizer


for sc = 1:length(spacecraft.orbits)
    spacecraft.orbits{sc} = spacecraft.orbits{sc}/1e3; %Convert to km
end

% How big do we have to go?
fig = figure();
for sc =1:length(spacecraft.orbits)
	plot3(spacecraft.orbits{sc}(1,:),spacecraft.orbits{sc}(2,:),spacecraft.orbits{sc}(3,:));
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

max_bandwidth = max(max(max(bandwidths)));
max_line_thickness = 20;

max_memory = max(spacecraft.memory);
max_delivered = sum(delivered_science);
max_memory_marker_size = 40;

if nargin<7
    plot_body = false;
else
    plot_body = true;
end

if nargin<8
    record_video = false;
end

if nargin<9
    videoname = 'video.avi';
end

if record_video
    writerObj = VideoWriter(videoname);
    writerObj.FrameRate = 30;
    open(writerObj);
end

if plot_body
    sc_colors = rand(length(spacecraft.orbits),3);
end

link_color_steps = 100;
link_colors = [summer(link_color_steps/2);flipud(autumn(link_color_steps/2))]; %flipud(hot(link_color_steps));

bandwidth_duals = bandwidth_and_memory_duals;
memory_duals = zeros(length(spacecraft.time),length(spacecraft.orbits));
for k=1:length(spacecraft.time)
    for i=1:length(spacecraft.orbits)
        bandwidth_duals(k,i,i) = 0;
        memory_duals(k,i) = bandwidth_and_memory_duals(k,i,i);
    end
end

max_bandwidth_duals = max(max(max(bandwidth_duals)));
min_bandwidth_duals = min(min(min(bandwidth_duals)));

for time = 1:length(spacecraft.time)
    if plot_body
        % Just to get the body
        gravity_model.plot_absolute_traj(spacecraft.time(time), spacecraft.orbits{1}(:,time)', true, fig)
        hold all
    end
    
    
    for sc1 = 1:length(spacecraft.orbits)
        % Plot the trajectory
        plot3(spacecraft.orbits{sc1}(1,1:time),spacecraft.orbits{sc1}(2,1:time),spacecraft.orbits{sc1}(3,1:time),...
            'Color',sc_colors(sc1,:));
        hold all;
        
        % Plot the s/c location and memory use
        curr_memory = flows(time,sc1,sc1);
        if sc1 == length(spacecraft.orbits)
            % If the sc is the carrier, also count the science we delivered
            curr_memory = curr_memory+sum(delivered_science(1:time));
            ssymbol = 's';
            ssize = curr_memory/max_delivered*max_memory_marker_size +1;
        else
            ssymbol = '.';
            ssize = curr_memory/max_memory*max_memory_marker_size +1;
        end
        plot3(spacecraft.orbits{sc1}(1,time),spacecraft.orbits{sc1}(2,time),spacecraft.orbits{sc1}(3,time), ...
            ssymbol,'markersize',ssize, 'MarkerFaceColor',sc_colors(sc1,:));
        hold all;
        
        for sc2 = 1:length(spacecraft.orbits)
            link_color_index = ceil(bandwidth_duals(time,sc1,sc2)/(max_bandwidth_duals-min_bandwidth_duals)*(link_color_steps-1)+1);
            % Plot the bandwidths
            lh = plot3([spacecraft.orbits{sc1}(1,time),spacecraft.orbits{sc2}(1,time)], ...
                [spacecraft.orbits{sc1}(2,time),spacecraft.orbits{sc2}(2,time)], ...
                [spacecraft.orbits{sc1}(3,time),spacecraft.orbits{sc2}(3,time)], ...
                'Color','k', ...
                'linewidth', (bandwidths(time,sc1,sc2))/max_bandwidth*max_line_thickness+1);
            lh.Color = [link_colors(link_color_index,:),.3];
            % Plot the actual info flow
            if flows(time,sc1,sc2)>0
                plot3([spacecraft.orbits{sc1}(1,time),spacecraft.orbits{sc2}(1,time)], ...
                    [spacecraft.orbits{sc1}(2,time),spacecraft.orbits{sc2}(2,time)], ...
                    [spacecraft.orbits{sc1}(3,time),spacecraft.orbits{sc2}(3,time)], ...
                    ':', ...
                    'Color', link_colors(link_color_index,:), ...
                    'linewidth', (flows(time,sc1,sc2))/max_bandwidth*max_line_thickness+1);
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