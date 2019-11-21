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



function [fig_handles] = plot_memory_use(varargin)
% PLOT_MEMORY_USE A function to plot the information flow between
% spacecraft
% Syntax: [axes_limits] = plot_information_flow(swarm, 
% plot_time, color_array*, absolute*, figure_handle*, max_memory*,
% max_memory_marker_size*)
% * Optional keyword inputs

swarm = varargin{1};
time = varargin{2};

absolute = false;
n_spacecraft = swarm.get_num_spacecraft();
%color_array = rand(3,n_spacecraft);
max_memory = NaN;

max_memory_marker_size = 50;
min_memory_marker_size = 15;

if length(varargin) > 2
    for i = 3:2:length(varargin)
        if strcmpi(varargin{i},'color_array') || strcmpi(varargin{i},'colorArray') ||  strcmpi(varargin{i},'color')
            color_array = varargin{i+1};
        end
        if strcmpi(varargin{i},'absolute')
            absolute = varargin{i+1};
        end
        if strcmpi(varargin{i},'figure_handle') || strcmpi(varargin{i},'handle')
            fig = varargin{i+1};
        end
        if strcmpi(varargin{i},'max_memory')
            max_memory = varargin{i+1};
        end
        if strcmpi(varargin{i},'max_memory_marker_size')
            max_memory_marker_size = varargin{i+1};
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

% Grab the figure
if exist('fig','var')
    current_fig = gcf();
    if current_fig ~= fig
        figure(fig);
    end
end

n_timesteps = swarm.get_num_timesteps();

if isnan(max_memory)
    tmp_memories = zeros(n_timesteps, n_spacecraft);
    for t = 1:n_timesteps
        for sc = 1:n_spacecraft
            tmp_memories(t,sc) = swarm.Communication.flow(t, sc, sc);
        end
    end
    max_memory = max(max(tmp_memories));
end

flows_to_carrier = swarm.Communication.flow(:,:,end);
flows_from_carrier = squeeze(swarm.Communication.flow(:,end,:));
delivered_science = zeros(n_timesteps,1);
delivered_science(2:end) = sum(flows_to_carrier(1:end-1,:),2)-sum(flows_from_carrier(2:end,:),2)+swarm.Communication.effective_source_flow(end,1:end-1)';

max_delivered = sum(sum(swarm.Communication.effective_source_flow));

for sc1 = 1:n_spacecraft

    % Plot the s/c location and memory use
    curr_memory = swarm.Communication.flow(time,sc1,sc1);
    if sc1 == n_spacecraft
        % If the sc is the carrier, also count the science we delivered
        curr_memory = curr_memory+sum(delivered_science(1:time));
        ssymbol = 's';
        ssize = curr_memory/max_delivered*(max_memory_marker_size-min_memory_marker_size) +min_memory_marker_size;
    else
        ssymbol = '.';
        ssize = curr_memory/max_memory*(max_memory_marker_size-min_memory_marker_size) +min_memory_marker_size;
    end
    % plot3(spacecraft.orbits{sc1}(1,time),spacecraft.orbits{sc1}(2,time),spacecraft.orbits{sc1}(3,time), ...
    sc_color = color_array(:,mod(sc1-1,size(color_array,2))+1)';
    %[sc1, NaN, sc_color]
    h_sc(sc1) = plot3(trajectory_array(time, 1, sc1), trajectory_array(time, 2, sc1), trajectory_array(time, 3, sc1), ...
        ssymbol,'MarkerSize',ssize, 'MarkerFaceColor',sc_color,'MarkerEdgeColor',sc_color);

end

    
fig_handles = h_sc;