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

function [ ] = plot_communication_topology_2d(time_step, Swarm, AsteroidModel, color_array, font_size, max_line_thickness, max_bandwidth)
%PLOT_COMMUNICATION_TOPOLOGY_2D Plots a 2D projection of the spacecraft
%locations with lines indicating the data flow on the current iteration.
%The asteroid is approximated by a spherical outer bound.
%   Syntax: [] = plot_communication_topology(time_step, Swarm, AsteroidModel, color_array*, font_size*, max_bandwidth*)
%    *optional input

n_timesteps = Swarm.get_num_timesteps;
n_spacecraft = Swarm.get_num_spacecraft;
if nargin<7
    % Estimate max and min bandwidth so we can plot accordingly
    tmp_bandwidths = Swarm.Communication.bandwidths_and_memories;
    for t = 1:n_timesteps
        for sc = 1:n_spacecraft
            tmp_bandwidths(t, sc, sc) = 0;
        end
    end
    max_bandwidth = max(max(max(tmp_bandwidths)));
end

if nargin<6
    max_line_thickness = 5;
end

if nargin<5
    font_size = 25;
end

if nargin < 4
    color_array = ['c' 'r' 'b' 'g' 'm'];
end


cla()
hold on
% Plot the radius encircling the asteroid
asteroid_max_radius = AsteroidModel.BodyModel.shape.maxRadius*1e3;
asteroid_mean_radius = AsteroidModel.BodyModel.shape.meanRadius*1e3;
asteroid_min_radius = AsteroidModel.BodyModel.shape.minRadius*1e3;

theta = 0:pi/50:2*pi;
plot(asteroid_max_radius * cos(theta), asteroid_max_radius * sin(theta),':k','LineWidth',1.0);
plot(asteroid_mean_radius * cos(theta), asteroid_mean_radius * sin(theta),'-k','LineWidth',2.0);
patch(asteroid_min_radius * cos(theta), asteroid_min_radius * sin(theta),'k');


n_spacecraft = Swarm.get_num_spacecraft;
% Plot the spacecraft locations
for ns = 1:1:n_spacecraft
    plot(Swarm.abs_trajectory_array(time_step,1,ns),Swarm.abs_trajectory_array(time_step,2,ns),'o',...
        'MarkerFaceColor',color_array(:,mod(ns-1,size(color_array,2))+1)',...
        'MarkerEdgeColor',color_array(:,mod(ns-1,size(color_array,2))+1)',...
        'MarkerSize',10)
end
carrier_index = Swarm.get_indicies_of_type(0);
assert(length(carrier_index) <= 1, "ERROR: too many carriers")
if ~isempty(carrier_index) && carrier_index>0
    plot(Swarm.abs_trajectory_array(time_step,1,carrier_index),Swarm.abs_trajectory_array(time_step,2,carrier_index),'o',...
        'MarkerFaceColor','k',...
        'MarkerEdgeColor','k',...
        'MarkerSize',10)
end

% Communications
for sc1 = 1:n_spacecraft
    for sc2 = 1:n_spacecraft
        if Swarm.Communication.bandwidths_and_memories(time_step,sc1, sc2) > 0
            plot([Swarm.abs_trajectory_array(time_step, 1, sc1) Swarm.abs_trajectory_array(time_step, 1, sc2)] , ...
                 [Swarm.abs_trajectory_array(time_step, 2, sc1) Swarm.abs_trajectory_array(time_step, 2, sc2)],'-', ...
                 'LineWidth',Swarm.Communication.bandwidths_and_memories(time_step,sc1, sc2)/max_bandwidth*max_line_thickness, ...
                 'Color',color_array(:,mod(ns-1,size(color_array,2))+1)')
        end
    end
end

xlabel('X Axis [m]','fontsize',font_size)
ylabel('Y Axis [m]','fontsize',font_size)
title('Communication Topology','fontsize',font_size)
set(gca, 'fontsize',font_size)
xlim = [min(min(Swarm.abs_trajectory_array(:,1,:))), max(max(Swarm.abs_trajectory_array(:,1,:)))];
ylim = [min(min(Swarm.abs_trajectory_array(:,2,:))), max(max(Swarm.abs_trajectory_array(:,2,:)))];
axis equal
axis([xlim, ylim])

% 
% function [ ] = plot_communication_topology(sc_current_state, carrier_sc_position, communicating_sc_index, asteroid_radius, color_array, standard_font_size)
% %PLOT_COMMUNICATION_TOPOLOGY Plots a 2D projection of the spacecraft
% %locations with lines indicating the data flow on the current iteration.
% %The asteroid is approximated by a spherical outer bound.
% %   Syntax: [] = plot_communication_topology(sc_current_state, carrier_sc_position, communicating_sc_index, asteroid_radius, *color_array, *standard_font_size)
% %    *optional input
% 
% %% Interpret Input
% n_spacecraft = size(sc_current_state,1);
% 
% 
% %% Plot
% cla()
% hold on
% theta = 0:pi/50:2*pi;
% x_circle = asteroid_radius * cos(theta);
% y_circle = asteroid_radius * sin(theta);
% plot(x_circle, y_circle,'-k','LineWidth',2.0);
% for ns = 1:1:n_spacecraft
%     plot(sc_current_state(ns,1),sc_current_state(ns,2),'o','MarkerFaceColor',color_array(mod(ns,length(color_array))+1),...
%         'MarkerEdgeColor',color_array(mod(ns,length(color_array))+1),'MarkerSize',10)
% end
% plot(carrier_sc_position(1),carrier_sc_position(2),'o','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',10)
% 
% % single hop
% if communicating_sc_index > 0
%     plot([sc_current_state(communicating_sc_index,1) carrier_sc_position(1)],...
%         [sc_current_state(communicating_sc_index,2) carrier_sc_position(2)],...
%         '-','LineWidth',2,'Color',color_array(mod(communicating_sc_index,length(color_array))+1))
% end
% 
% xlabel('X Axis [km]','fontsize',standard_font_size,'FontName','Times New Roman')
% ylabel('Y Axis [km]','fontsize',standard_font_size,'FontName','Times New Roman')
% title('Communication Topology','fontsize',standard_font_size,'FontName','Times New Roman')
% set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')
% axis equal
% 
% end
% 
