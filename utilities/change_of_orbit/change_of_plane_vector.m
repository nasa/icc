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

function [delta_v, total_time, intermediate_orbits, manoeuvers] = change_of_plane(initial_orbit, final_orbit, GM)
% CHANGE_OF_PLANE computes the change-of-plane transfer between two orbits
% Syntax: [delta_v, total_time, intermediate_orbits, manoeuvers] =
%  change_of_plane(initial_orbit, final_orbit, GM)
% Computes a change-of-plane maneuver transfer between initial_orbit and an
% orbit with inclination and RAAN corresponding to final_orbit (but with the
% same in-plane orbital elements a, e as initial_orbit). The argument of
% periapsis omega is changed in the maneuver.

% From Chg_Omega_i in Tycho

% How about we forget about all this ugliness and live in a beautiful world
% of linear algebra?
% Compute h for the old orbit in the trivial way.
% Compute h for the new orbit by applying rotations of Omega around z and i
% around x.
% Now the intersection of the two planes is Intersect = cross(old_h, new_h).
% The angle theta where the manoeuver occurs in the initial orbit is
% atan2(cross(Intersect, initial_eccentricity_vector),dot(Intersect,
% initial_eccentricity_vector)).
% We want that angle to be the same theta in the final orbit. So we get the
% new eccentricity vector by rotating intersect by -theta around new_h.
% We find the ascending node of the new orbit by rotating the x axis by
% Omega around z.
% We find the argument of periapsis as atan2(cross(new_eccentricity,
% new_ascending_node), dot(new_eccentricity, new_ascending_node));

[r_pre, v_pre] = op2rv(initial_orbit.a, initial_orbit.e, initial_orbit.i, initial_orbit.Omega, initial_orbit.omega, initial_orbit.theta, GM);
[r_post, v_post] = op2rv(final_orbit.a, final_orbit.e, final_orbit.i, final_orbit.Omega, final_orbit.omega, final_orbit.theta, GM);

h_pre = cross(r_pre, v_pre);
% Note that h_post does NOT refer to the orbit we end up to - we may be off
% shape-wise and omega-wise. However, h_post does correctly identify the
% plane of the orbit, which is what we need.
h_post = cross(r_post, v_post);

planes_intersection = cross(h_pre, h_post);
planes_intersection_unit = planes_intersection./norm(planes_intersection);

% Initial eccentricity vector, useful for theta
e_pre=((norm(v_pre).^2-GM/norm(r_pre)).*r_pre-(dot(r_pre,v_pre).*v_pre))./GM;
e_pre_unit = e_pre/norm(e_pre);
if norm(e_pre) == 0
    warning("Zero eccentricity")
    % Place the eccentricity in the x-y plane.
    e_pre_tmp= cross([0;0;1], h_pre);
    if norm(e_pre_tmp) == 0
        % Then we are in the x-y plane. Circular AND planar.
        e_pre_unit = [1;0;0];
    else
        e_pre_unit = e_pre_tmp/norm(e_pre_tmp);
    end
    %e_pre_unit = r_pre/norm(r_pre);
end
theta_pre = acos(dot(e_pre_unit, planes_intersection_unit));
% Is the intersection within 180 degrees of the eccentricity? If it is,
% then cross(e_pre, planes_intersection) has the same orientation as h_pre.
if dot(cross(e_pre_unit, planes_intersection_unit),h_pre)<0
    theta_pre = 2*pi-theta_pre;
end


theta_post = theta_pre;

% This is so that theta_post and theta_pre are the same at the intersection
e_post_unit = rotmat_axis_angle(h_post, -theta_pre)*planes_intersection_unit;

% And this is, by definition, where we would like the ascending node to be
% We normally use rotmat for changes of frame - to simply rotate a vector
% we need a minus.
ascending_node_post = rotmat(-final_orbit.Omega,3)*[1;0;0];

omega_post = acos(dot(ascending_node_post, e_post_unit));

% If eccentricity vector is below ascending node, then omega_post will be
% negative. If eccentricity is below zero, then by definition you have to
% go farther than 180 degrees from the ASCENDING node to the eccentricity.
% Therefore omega >180.

if e_post_unit(3)<0
    omega_post = 2*pi - omega_post;
end

intermediate_orbit.a = initial_orbit.a;
intermediate_orbit.e = initial_orbit.e;
intermediate_orbit.i = final_orbit.i;
intermediate_orbit.Omega = final_orbit.Omega;
intermediate_orbit.omega = omega_post;
intermediate_orbit.theta = theta_post;
intermediate_orbits = {intermediate_orbit};

[r_after_manoeuver_1,v_before_manoeuver] = op2rv(initial_orbit.a, initial_orbit.e, initial_orbit.i, initial_orbit.Omega, initial_orbit.omega, theta_pre, GM);


[r_after_manoeuver_2, v_after_manoeuver] = op2rv(intermediate_orbit.a, intermediate_orbit.e, intermediate_orbit.i, intermediate_orbit.Omega, intermediate_orbit.omega, intermediate_orbit.theta, GM);

delta_v = norm(v_after_manoeuver - v_before_manoeuver);

plot_flag = false;
if plot_flag
    initial_orbit_plt = initial_orbit;    
    initial_orbit_plt.theta = theta_pre;
    plot_labeled_orbit(initial_orbit_plt, GM)
    plot_labeled_orbit(intermediate_orbit, GM, gcf);
    e_post_plt = e_post_unit*norm(r_pre);
    quiver3(0,0,0,e_post_plt(1),e_post_plt(2),e_post_plt(3),'r')
    planes_intersection_plt = planes_intersection_unit*norm(r_pre);
    quiver3(0,0,0,planes_intersection_plt(1),planes_intersection_plt(2),planes_intersection_plt(3),'c', 'LineWidth', 3)
end

try
assert(norm(r_after_manoeuver_1-r_after_manoeuver_2)<1e-3, "ERROR: pre-manoeuver and post-manoeuver locations do not match");
catch ME
    disp("Hang debugger here");
    rethrow ME;
end

total_time = delta_t_on_orbit(initial_orbit, theta_pre, GM);

manoeuver.true_anomaly_before=theta_pre;
manoeuver.true_anomaly_after = theta_post;
manoeuver.delta_v = delta_v;
manoeuver.time = total_time;
manoeuver.type = "Change of plane";
manoeuver.location = r_after_manoeuver_2;

manoeuvers = {manoeuver};

return
