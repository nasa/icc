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

function [ordered_delta_vs, ordered_transfer_times, ordered_intermediate_orbits, ordered_dv_manoeuvers] = optimal_keplerian_orbit_transfer(initial_orbit, final_orbit, GM)
% OPTIMAL_KEPLERIAN_ORBIT_TRANSFER computes the optimal transfer between two orbits.
% Syntax: [ordered_delta_vs, ordered_transfer_times, ordered_intermediate_orbits, ordered_manoeuvers] =
%  optimal_keplerian_orbit_transfer(initial_orbit, final_orbit, GM)
% Inputs:
% - initial_orbit, a struct containing six orbital elements (see
%   https://en.wikipedia.org/wiki/Orbital_elements):
%   - a: the semimajor axis
%   - e: the eccentricity
%   - i: the inclination
%   - Omega: the RAAN
%   - omega: the argument of periapsis
%   - theta: the true anomaly
% - final_orbit, a struct identical to initial_orbit
% - GM, the gravitational constant of the body under consideration
% Outputs: vecrtors containing each
% - delta_v, a float representing the required delta_v in km/s
% - total_time, a float representing the required time for the transfer in s
% - intermediate_orbits, a list (cell) of orbits with format identical to
%    initial_orbit. The true anomaly theta represents the t.a. immediately as
%    the orbit is reached
% - manoeuvers, a list (cell) of structs representing maneuvers. Each manoeuver
%    is a struct containing
%    - delta_v: a vector representing the required delta_v
%    - time: the time of the delta_v
%    - true_anomaly_before: the true anomaly on the pre-maneuver orbit.
%    - true_anomaly_after: the true anomaly on the post-maneuver orbit. Should
%      correspond to the true anomaly in intermediate_orbits.

% Create all possible permutations of maneuvers

ordered_manoeuvers_permutations = {};

for aim_for_apoapsis = 0:1
    % Hohmann first - we are free to aim for different omegas
	for same_argument_of_periapsis = 0:1
		hohmann_strategy = @(initial_orbit, final_orbit, GM) hohmann_transfer(initial_orbit, final_orbit, GM, aim_for_apoapsis, same_argument_of_periapsis);
		manoeuvers_functions = {hohmann_strategy, @change_of_plane_and_argument_of_periapsis};
		ordered_manoeuvers_permutations = [ordered_manoeuvers_permutations; manoeuvers_functions];
    end
    % Change of plane first - we are not free to aim for different omegas
    same_argument_of_periapsis = 1;
    hohmann_strategy = @(initial_orbit, final_orbit, GM) hohmann_transfer(initial_orbit, final_orbit, GM, aim_for_apoapsis, same_argument_of_periapsis);
	manoeuvers_functions = {@change_of_plane_and_argument_of_periapsis, hohmann_strategy};
	ordered_manoeuvers_permutations = [ordered_manoeuvers_permutations; manoeuvers_functions];
end

% Compute dV for a given permutation
ordered_delta_vs = zeros(size(ordered_manoeuvers_permutations,1),1);
ordered_transfer_times = zeros(size(ordered_manoeuvers_permutations,1),1);
ordered_intermediate_orbits = cell(size(ordered_manoeuvers_permutations,1),1);

for ordered_manoeuver_index = 1:size(ordered_manoeuvers_permutations,1)
	ordered_manoeuvers = {ordered_manoeuvers_permutations{ordered_manoeuver_index,:}};
	intermediate_orbits = {initial_orbit};
	manoeuvers = {};
	overall_delta_v = 0;
	overall_time = 0;
	for manoeuver_index = 1:length(ordered_manoeuvers)
		manoeuver_to_perform = ordered_manoeuvers{manoeuver_index};
		% We start from the last intermediate orbit considered
		start_orbit = intermediate_orbits{end};
		% We apply the new maneuver
		[manoeuver_delta_v, manoeuver_time, manoeuvering_orbits, new_manoeuvers] = manoeuver_to_perform(start_orbit, final_orbit, GM);
		% We append the new orbits to the list
		intermediate_orbits = [intermediate_orbits; manoeuvering_orbits];
		% We append the maneuvers to the list
		manoeuvers = [manoeuvers; new_manoeuvers];
		% We update dV and time
		overall_delta_v = overall_delta_v+manoeuver_delta_v;
		overall_time = overall_time + manoeuver_time;
	end
	% Save the result
    try
        ordered_delta_vs(ordered_manoeuver_index) = overall_delta_v;
        ordered_transfer_times(ordered_manoeuver_index) = overall_time;
        ordered_intermediate_orbits{ordered_manoeuver_index} = intermediate_orbits;
        ordered_dv_manoeuvers{ordered_manoeuver_index} = manoeuvers;
    catch ME
        % This looks barbaric, but it is a handy way to hang the debugger
        % on failure only
        rethrow ME
    end
end
