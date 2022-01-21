%  This file is part of Tycho [1].
% 
% Tycho is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% Tycho is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with Tycho.  If not, see <https://www.gnu.org/licenses/>.
%     
% [1] https://sourceforge.net/projects/tycho-astro/

function [] = plot_transfer(initial_orbit, final_orbit, intermediate_orbits, manoeuvers, GM, axis_handle, plot_body, plot_velocity)
% PLOT_TRANSFER: plots a transfer from initial_orbit to final_orbit through
% intermediate_orbits via manoeuvers
% Syntax:  plot_transfer(initial_orbit, final_orbit, intermediate_orbits, manoeuvers, GM, plot_body)

noop_plot_body = @() 0;

if nargin<6
    figure();
    hold all;
    axis_handle = axes;
end

if nargin< 7
    plot_body = noop_plot_body;
end

if nargin<8
    plot_velocity=false;
end

% Plot initial orbit
plot_orbit(initial_orbit, initial_orbit.theta, manoeuvers{1}.true_anomaly_before, GM, axis_handle, plot_velocity, noop_plot_body);


% Plot intermediate orbits
for orbit_ix = 1:length(intermediate_orbits)-1 % The final intermediate orbit is just the final orbit
    orbit = intermediate_orbits{orbit_ix};
    initial_theta = manoeuvers{orbit_ix}.true_anomaly_after;
    final_theta = manoeuvers{orbit_ix+1}.true_anomaly_before;
    plot_orbit(orbit, initial_theta, final_theta, GM, axis_handle, plot_velocity, noop_plot_body);
end

% Plot final orbit
plot_orbit(final_orbit, manoeuvers{end}.true_anomaly_after, final_orbit.theta, GM, axis_handle, plot_velocity, plot_body);

% Plot maneuvers
for manoeuver_ix = 1:length(manoeuvers)
    text(manoeuvers{manoeuver_ix}.location(1), manoeuvers{manoeuver_ix}.location(2), manoeuvers{manoeuver_ix}.location(3), manoeuvers{manoeuver_ix}.type);
end