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
%                                                                             %cd
% This  software  is a copy  and  may not be current.  The latest  version is %
% maintained by and may be obtained from the Mobility  and  Robotics  Sytstem %
% Section (347) at the Jet  Propulsion  Laboratory.   Suggestions and patches %
% are welcome and should be sent to the software's maintainer.                %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = plot_pareto_curve(initial_orbit, final_orbit, ordered_intermediate_orbits, ordered_transfer_times, ordered_delta_vs, ordered_manoeuvers, GM, body, plot_body)

figure()
hold all
title(sprintf("\\fontsize{12}Pareto plot, %s", body))
xlabel("Transfer time [s]")
ylabel("Transfer dV [km/s]")
for option_ix = 1:length(ordered_intermediate_orbits)
    plot(ordered_transfer_times(option_ix), ordered_delta_vs(option_ix),'o')
    text(ordered_transfer_times(option_ix), ordered_delta_vs(option_ix),sprintf("Strategy %d", option_ix))
end
% Do the dumb thing to find the Pareto curve
% First, sort by time
sorting_struct = [[1:length(ordered_intermediate_orbits)]',ordered_transfer_times,ordered_delta_vs];
sorting_struct = sortrows(sorting_struct, 2);

Pareto_points = [];
for option_ix = 1:size(sorting_struct,1)
    is_dominated = false;
    for option_ix_compare = 1:size(sorting_struct,1)
        if sorting_struct(option_ix,2)>sorting_struct(option_ix_compare,2) && sorting_struct(option_ix,3)>sorting_struct(option_ix_compare,3)
            is_dominated = true;
            break;
        end
    end
    if ~is_dominated
        Pareto_points = [Pareto_points; sorting_struct(option_ix, :)];
    end
end
plot(Pareto_points(:,2),Pareto_points(:,3))
%     for entry = 1:size(Pareto_points, 1)
%         text(Pareto_points(entry,2),Pareto_points(entry,3), sprintf("Strategy %d", Pareto_points(entry,1)),"FontWeight", "bold");
%     end
num_pareto_optimal_strategies = size(Pareto_points,1);
pareto_ix = 0;
figure()
hold all
title("Pareto-optimal strategies")
for option_ix = 1:length(ordered_intermediate_orbits)
    % Below is a truly horrific way of saying "is this a Pareto solution".
    % Future me, forgive me.
    if ~isempty(find(Pareto_points(:,1) == option_ix,1))
        pareto_ix = pareto_ix+1;
        axes = subplot(1, num_pareto_optimal_strategies, pareto_ix);
        intermediate_orbits = ordered_intermediate_orbits{option_ix};
        intermediate_orbits = {intermediate_orbits{2:end}}; % The first orbit is just the initial orbit
        manoeuvers = ordered_manoeuvers{option_ix};
        plot_transfer(initial_orbit, final_orbit, intermediate_orbits, manoeuvers, GM, axes, plot_body);
        title({sprintf("\\fontsize{12}Strategy %d", option_ix), sprintf("\\fontsize{9}dV=%.4f km/s", ordered_delta_vs(option_ix)), sprintf("\\fontsize{9}dT=%.1f s",ordered_transfer_times(option_ix))});
        view(3)
        axis equal
%     else
%         fprintf("Strategy %d is Pareto-dominated\n", option_ix);
    end
end

return