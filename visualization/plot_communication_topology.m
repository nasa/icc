function [ ] = plot_communication_topology(sc_current_state, carrier_sc_position, communicating_sc_index, asteroid_radius, color_array, standard_font_size)
%PLOT_COMMUNICATION_TOPOLOGY Plots a 2D projection of the spacecraft
%locations with lines indicating the data flow on the current iteration.
%The asteroid is approximated by a spherical outer bound.
%   Syntax: [] = plot_communication_topology(sc_current_state, carrier_sc_position, communicating_sc_index, asteroid_radius, *color_array, *standard_font_size)
%    *optional input

%% Interpret Input
n_spacecraft = size(sc_current_state,1);

if nargin < 6
    standard_font_size = 25;
end
if nargin < 5
    color_array = ['c' 'r' 'b' 'g' 'm'];
end

%% Plot
cla()
hold on
theta = 0:pi/50:2*pi;
x_circle = asteroid_radius * cos(theta);
y_circle = asteroid_radius * sin(theta);
plot(x_circle, y_circle,'-k','LineWidth',2.0);
for ns = 1:1:n_spacecraft
    plot(sc_current_state(ns,1),sc_current_state(ns,2),'o','MarkerFaceColor',color_array(mod(ns,length(color_array))+1),...
        'MarkerEdgeColor',color_array(mod(ns,length(color_array))+1),'MarkerSize',10)
end
plot(carrier_sc_position(1),carrier_sc_position(2),'o','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',10)

% single hop
if communicating_sc_index > 0
    plot([sc_current_state(communicating_sc_index,1) carrier_sc_position(1)],...
        [sc_current_state(communicating_sc_index,2) carrier_sc_position(2)],...
        '-','LineWidth',2,'Color',color_array(mod(communicating_sc_index,length(color_array))+1))
end

xlabel('X Axis [km]','fontsize',standard_font_size,'FontName','Times New Roman')
ylabel('Y Axis [km]','fontsize',standard_font_size,'FontName','Times New Roman')
title('Communication Topology','fontsize',standard_font_size,'FontName','Times New Roman')
set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')
axis equal

end

