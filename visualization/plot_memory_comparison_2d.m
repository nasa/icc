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

function [ ] = plot_memory_comparison_2d(varargin)
%PLOT_MEMORY_COMPARISON Plots a bar graph of the memory use of each
%spacecraft.
% Syntax: [ ] = plot_memory_comparison(time_step, Swarm, color_array*, font_size*, semilogflag*)
% * optional keyword arguments

time_step = varargin{1};
Swarm = varargin{2};

% Defaults
semilogflag = false;

if length(varargin) > 2
    for i = 3:1:length(varargin)
        if strcmpi(varargin{i},'color_array') || strcmpi(varargin{i},'colorArray') ||  strcmpi(varargin{i},'color')
            color_array = varargin{i+1};
        end
        if strcmpi(varargin{i},'semilogflag') || strcmpi(varargin{i},'semi_log_flag')  || strcmpi(varargin{i},'semilog')
            semilogflag = varargin{i+1};
        end
        if strcmpi(varargin{i},'font_size') || strcmpi(varargin{i},'fontsize') || strcmpi(varargin{i},'standard_font_size')
            standard_font_size = varargin{i+1};
        end
        if strcmpi(varargin{i},'title_font_size')
            title_font_size = varargin{i+1};
        end
        if strcmpi(varargin{i},'font_name')
            font_name = varargin{i+1};
        end
    end
end

n_spacecraft = Swarm.get_num_spacecraft();

% Find memory use
sc_memory_use = zeros(n_spacecraft,1);
for sc = 1:n_spacecraft-1
    sc_memory_use(sc) = Swarm.Communication.flow(time_step, sc, sc);
end

flows_to_carrier = Swarm.Communication.flow(:,:,end);
flows_from_carrier = squeeze(Swarm.Communication.flow(:,end,:));
n_timesteps = Swarm.get_num_timesteps();
delivered_science = zeros(n_timesteps,1);
delivered_science(2:end) = sum(flows_to_carrier(1:end-1,:),2)-sum(flows_from_carrier(2:end,:),2)+Swarm.Communication.effective_source_flow(end,1:end-1)';

sc_memory_use(n_spacecraft) = Swarm.Communication.flow(time_step, n_spacecraft, n_spacecraft) + sum(delivered_science(1:time_step));

% Plot the bar graph
cla()
hold on
if semilogflag
    set(gca, 'YScale', 'log');
end

for i_sc = 1:1:n_spacecraft
    % Show memory limit
    plot([i_sc i_sc+0.1], (1e-6)*[Swarm.Parameters.available_memory(i_sc) Swarm.Parameters.available_memory(i_sc)],'-k','LineWidth',2)

    % Plot a rectangle with the SC's memory use
    patch([i_sc i_sc+0.1 i_sc+0.1 i_sc], (1e-6)*[eps eps sc_memory_use(i_sc,1)+eps sc_memory_use(i_sc,1)+eps],color_array(:,mod(Swarm.Parameters.types{i_sc},size(color_array,2))+1)')
    % If memory usage is high enough, show memory limit
    %if sc_memory_use(i_sc) > 0.1 * Swarm.Parameters.available_memory(i_sc)
    %end
end

if semilogflag
    axis_min_y = 1e-5*((1e-6)*max(Swarm.Parameters.available_memory));
else
    axis_min_y = 0;
end

axis([0 n_spacecraft+1 axis_min_y (1e-6)*max(Swarm.Parameters.available_memory)])

xlabel('SC Number','fontsize',standard_font_size, 'fontname',font_name)
if semilogflag
    ylabel('Mega Bits (log)','fontsize',standard_font_size, 'fontname',font_name)
else
    ylabel('Mega Bits','fontsize',standard_font_size, 'fontname',font_name)
end
title('Memory Use','fontsize',title_font_size, 'fontname',font_name)
set(gca, 'fontsize',standard_font_size, 'fontname',font_name)

end