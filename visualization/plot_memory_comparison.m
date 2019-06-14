function [ ] = plot_memory_comparison( sc_memory_use , communicating_sc_index, standard_comm_data_rate, delta_t, sc_max_memory , color_array, standard_font_size )
%PLOT_MEMORY_COMPARISON Plots a bar graph of the memory use of each
%spacecraft.
%   Syntax: [] = plot_memory_comparison( sc_memory_use , communicating_sc_index, standard_comm_data_rate, delta_t, *sc_max_memory , *color_array, *standard_font_size )
%    *optional input

%% Interpret Inputs

n_spacecraft = size(sc_memory_use,1);

if nargin < 7
    standard_font_size = 25;
end
if nargin < 6
    color_array = ['c' 'r' 'b' 'g' 'm'];
end
if nargin < 5
    flag_knowMaxMemory = false;
else
    flag_knowMaxMemory = true;
end


%% Plot
cla()
hold on

for i_sc = 1:1:n_spacecraft
    fill([i_sc i_sc+0.1 i_sc+0.1 i_sc], (1e-6)*[0 0 sc_memory_use(i_sc,1) sc_memory_use(i_sc,1)],color_array(mod(i_sc,length(color_array))+1))
end

if communicating_sc_index > 0
    fill([0 communicating_sc_index+0.1 communicating_sc_index+0.1 0], (1e-6)*[sc_memory_use(communicating_sc_index,1)...
        sc_memory_use(communicating_sc_index,1) sc_memory_use(communicating_sc_index,1)+standard_comm_data_rate*delta_t ...
        sc_memory_use(communicating_sc_index,1)+standard_comm_data_rate*delta_t], color_array(mod(communicating_sc_index,length(color_array))+1));
end

xlim([0 n_spacecraft+1])

if flag_knowMaxMemory == true
    if max(sc_memory_use) > 0.1*sc_max_memory
        plot([0 n_spacecraft+1], (1e-6)*[sc_max_memory sc_max_memory],'-k','LineWidth',2)
        ylim((1e-6)*[0 sc_max_memory])
    end
end

xlabel('SC Number','fontsize',standard_font_size,'FontName','Times New Roman')
ylabel('Mega Bits','fontsize',standard_font_size,'FontName','Times New Roman')
title('Memory Use','fontsize',standard_font_size,'FontName','Times New Roman')
set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')

end

