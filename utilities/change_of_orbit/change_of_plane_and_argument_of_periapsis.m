function [delta_v, total_time, intermediate_orbits, manoeuvers] = change_of_plane_and_argument_of_periapsis(initial_orbit, final_orbit, GM)
% CHANGE_OF_PLANE_AND_ARGUMENT_OF_PERIAPSIS computes the change-of-plane 
% and change of argument of periapsis transfer between two orbits
% Syntax: [delta_v, total_time, intermediate_orbits, manoeuvers] =
%  change_of_plane_and_argument_of_periapsis(initial_orbit, final_orbit, GM)
% Computes a change-of-plane maneuver transfer between initial_orbit and an
% orbit with inclination and RAAN corresponding to final_orbit (but with the
% same in-plane orbital elements a, e as initial_orbit), followed by a
% change of periapsis omega to ensure the periapsis matches the final
% orbit.

[delta_v_cp, total_time_cp, intermediate_orbits_cp, manoeuvers_cp] = change_of_plane_vector(initial_orbit, final_orbit, GM);


% Test change of argument of periapsis
[delta_v_ap, total_time_ap, intermediate_orbits_ap, manoeuvers_ap] = change_argument_of_periapsis(intermediate_orbits_cp{end}, final_orbit, GM);

delta_v = delta_v_cp + delta_v_ap;
total_time = total_time_cp + total_time_ap;
intermediate_orbits = [intermediate_orbits_cp; intermediate_orbits_ap];
manoeuvers = [manoeuvers_cp; manoeuvers_ap];