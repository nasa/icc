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

function [ ] = plot_memory_comparison(time_step, Swarm, color_array, font_size)
%PLOT_MEMORY_COMPARISON Plots a bar graph of the memory use of each
%spacecraft.
% Syntax: [ ] = plot_memory_comparison(time_step, Swarm, color_array*, font_size*)
% * optional input
if nargin<4
    font_size = 25;
end

if nargin < 3
    color_array = ['c' 'r' 'b' 'g' 'm'];
end

n_spacecraft = Swarm.get_num_spacecraft();

% Find memory use
sc_memory_use = zeros(n_spacecraft,1);
for sc = 1:n_spacecraft
    sc_memory_use(sc) = Swarm.Communication.flow(time_step, sc, sc);
end

% Plot the bar graph
cla()
hold on

for i_sc = 1:1:n_spacecraft
    % Plot a rectangle with the SC's memory use
    fill([i_sc i_sc+0.1 i_sc+0.1 i_sc], (1e-6)*[0 0 sc_memory_use(i_sc,1) sc_memory_use(i_sc,1)],color_array(mod(i_sc,length(color_array))+1))
    % If memory usage is high enough, show memory limit
    if sc_memory_use(i_sc) > 0.1 * Swarm.Parameters.available_memory(i_sc)
        plot([i_sc i_sc+0.1], (1e-6)*[Swarm.Parameters.available_memory(i_sc) Swarm.Parameters.available_memory(i_sc)],'-k','LineWidth',2)
    end
end

xlim([0 n_spacecraft+1])
ylim((1e-6)*[0 max(Swarm.Parameters.available_memory)])

xlabel('SC Number','fontsize',font_size)
ylabel('Mega Bits','fontsize',font_size)
title('Memory Use','fontsize',font_size)
set(gca, 'fontsize',font_size)

end

% function [ ] = plot_memory_comparison( sc_memory_use , communicating_sc_index, standard_comm_data_rate, delta_t, sc_max_memory , color_array, standard_font_size )
% %PLOT_MEMORY_COMPARISON Plots a bar graph of the memory use of each
% %spacecraft.
% %   Syntax: [] = plot_memory_comparison( sc_memory_use , communicating_sc_index, standard_comm_data_rate, delta_t, *sc_max_memory , *color_array, *standard_font_size )
% %    *optional input
% 
% %% Interpret Inputs
% 
% n_spacecraft = size(sc_memory_use,1);
% 
% if nargin < 7
%     standard_font_size = 25;
% end
% if nargin < 6
%     color_array = ['c' 'r' 'b' 'g' 'm'];
% end
% if nargin < 5
%     flag_knowMaxMemory = false;
% else
%     flag_knowMaxMemory = true;
% end
% 
% 
% 
% 
% %% Plot
% cla()
% hold on
% 
% for i_sc = 1:1:n_spacecraft
%     % Plot a rectangle with the SC's memory use
%     fill([i_sc i_sc+0.1 i_sc+0.1 i_sc], (1e-6)*[0 0 sc_memory_use(i_sc,1) sc_memory_use(i_sc,1)],color_array(mod(i_sc,length(color_array))+1))
% end
% 
% if communicating_sc_index > 0
%     fill([0 communicating_sc_index+0.1 communicating_sc_index+0.1 0], (1e-6)*[sc_memory_use(communicating_sc_index,1)...
%         sc_memory_use(communicating_sc_index,1) sc_memory_use(communicating_sc_index,1)+standard_comm_data_rate*delta_t ...
%         sc_memory_use(communicating_sc_index,1)+standard_comm_data_rate*delta_t], color_array(mod(communicating_sc_index,length(color_array))+1));
% end
% 
% xlim([0 n_spacecraft+1])
% 
% if flag_knowMaxMemory == true
%     if max(sc_memory_use) > 0.1*sc_max_memory
%         plot([0 n_spacecraft+1], (1e-6)*[sc_max_memory sc_max_memory],'-k','LineWidth',2)
%         ylim((1e-6)*[0 sc_max_memory])
%     end
% end
% 
% xlabel('SC Number','fontsize',standard_font_size,'FontName','Times New Roman')
% ylabel('Mega Bits','fontsize',standard_font_size,'FontName','Times New Roman')
% title('Memory Use','fontsize',standard_font_size,'FontName','Times New Roman')
% set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')
% 
% end
% 
