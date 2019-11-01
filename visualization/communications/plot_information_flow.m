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



function [fig_handles] = plot_information_flow(varargin)
% PLOT_INFORMATION_FLOW A function to plot the information flow between
% spacecraft
% Syntax: [axes_limits] = plot_information_flow(swarm, plot_time,
% absolute*, figure_handle*, 
% max_bandwidth*, min_line_thickness*, max_line_thickness*, link_color_steps*)
% * Optional keyword inputs

swarm = varargin{1};
time = varargin{2};

absolute = false;
n_spacecraft = swarm.get_num_spacecraft();
max_bandwidth = NaN;

min_line_thickness = 1;
max_line_thickness = 20;
link_color_steps = 100;

if length(varargin) > 2
    for i = 3:2:length(varargin)
        if strcmpi(varargin{i},'absolute')
            absolute = varargin{i+1};
        end
        if strcmpi(varargin{i},'figure_handle') || strcmpi(varargin{i},'handle')
            fig = varargin{i+1};
        end
        if strcmpi(varargin{i},'max_bandwidth')
            max_bandwidth = varargin{i+1};
        end
        if strcmpi(varargin{i},'min_line_thickness')
            min_line_thickness = varargin{i+1};
        end
        if strcmpi(varargin{i},'max_line_thickness')
            max_line_thickness = varargin{i+1};
        end

        if strcmpi(varargin{i},'link_color_steps')
            link_color_steps = varargin{i+1};
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

if exist('fig','var')
    current_fig = gcf();
    if current_fig ~= fig
        figure(fig);
    end
end

link_colors = [summer(link_color_steps/2);flipud(autumn(link_color_steps/2))]; %flipud(hot(link_color_steps));
 
n_timesteps = swarm.get_num_timesteps();

if isnan(max_bandwidth)
    tmp_bandwidths = swarm.Communication.bandwidths_and_memories;
    for t = 1:n_timesteps
        for sc = 1:n_spacecraft
            tmp_bandwidths(t, sc, sc) = 0;
        end
    end
    max_bandwidth = max(max(max(tmp_bandwidths)));
end

bandwidth_duals = swarm.Communication.dual_bandwidths_and_memories;
max_bandwidth_duals = max(max(max(bandwidth_duals)));
min_bandwidth_duals = min(min(min(bandwidth_duals)));

h_comm = gobjects(n_spacecraft, n_spacecraft);
h_bw = gobjects(n_spacecraft, n_spacecraft);

for sc1 = 1:n_spacecraft
    for sc2 = 1:n_spacecraft
        link_color_index = ceil(bandwidth_duals(time,sc1,sc2)/(max_bandwidth_duals-min_bandwidth_duals)*(link_color_steps-1)+1);
        if isnan(link_color_index) || link_color_index<1
            warning("Problem is not communication-limited at all: all bandwidth duals are zero");
            link_color_index = 1;
        end
        % Plot the bandwidths
        h_bw(sc1,sc2) = plot3([trajectory_array(time, 1, sc1), trajectory_array(time, 1, sc2)], ...
            [trajectory_array(time, 2, sc1), trajectory_array(time, 2, sc2)], ...
            [trajectory_array(time, 3, sc1), trajectory_array(time, 3, sc2)], ...
            'Color','k', ...
            'linewidth', (swarm.Communication.bandwidths_and_memories(time,sc1,sc2))/max_bandwidth*max_line_thickness+min_line_thickness);
        h_bw(sc1,sc2).Color = [link_colors(link_color_index,:),.3];
        
        % Plot the actual info flow
        if swarm.Communication.flow(time,sc1,sc2)>0
            h_comm(sc1,sc2) = plot3([trajectory_array(time, 1, sc1), trajectory_array(time, 1, sc2)], ...
                [trajectory_array(time, 2, sc1), trajectory_array(time, 2, sc2)], ...
                [trajectory_array(time, 3, sc1), trajectory_array(time, 3, sc2)], ...
                ':', ...
                'Color', link_colors(link_color_index,:), ...
                'linewidth', (swarm.Communication.flow(time,sc1,sc2))/max_bandwidth*max_line_thickness+min_line_thickness);
            hold all;
        end
        
    end
end
    
fig_handles = [h_bw, h_comm];