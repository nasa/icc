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

warning("This function is DEPRECATED and known to be buggy for certain geometric configurations. The replacement is change_of_plane_vector");


% Angle between the orbits
theta_man=acos(cos(initial_orbit.i)*cos(final_orbit.i)+sin(initial_orbit.i)*sin(final_orbit.i)*cos(final_orbit.Omega-initial_orbit.Omega));

if initial_orbit.i>final_orbit.i
	theta_man=2*pi-theta_man;
end

cos_u_i=((sin(final_orbit.i)*cos(final_orbit.Omega-initial_orbit.Omega)...
    -cos(theta_man)*sin(initial_orbit.i)) ...
    /(sin(theta_man)*cos(initial_orbit.i)));
% sin_u_i=asin(sin(final_orbit.Omega-initial_orbit.Omega)/sin(theta_man)*sin(pi-final_orbit.i));

u_i = acos(cos_u_i);


cos_u_f=((cos(initial_orbit.i)*sin(final_orbit.i)-sin(initial_orbit.i)...
    *cos(final_orbit.i)*cos(final_orbit.Omega-initial_orbit.Omega))...
    /(sin(theta_man)));
% sin_u_f=asin(sin(final_orbit.Omega-initial_orbit.Omega)/sin(theta_man)*sin(initial_orbit.i));

u_f = acos(cos_u_f);

% https://en.wikipedia.org/w/index.php?title=Solution_of_triangles&action=edit&section=14
% gamma_angle = acos(sin(pi-final_orbit.i)*sin(initial_orbit.i)*cos(final_orbit.Omega-initial_orbit.Omega) - cos(pi-final_orbit.i)*cos(initial_orbit.i));
% 
% u_i = acos((cos(pi-final_orbit.i)+cos(initial_orbit.i)*cos(gamma_angle))/(sin(initial_orbit.i)*sin(gamma_angle)));
% u_f = acos((cos(initial_orbit.i)+cos(pi-final_orbit.i)*cos(gamma_angle))/(sin(pi-final_orbit.i)*sin(gamma_angle)));


%u=omega+theta;
theta_i=u_i-initial_orbit.omega;

theta_f=theta_i;
intermediate_orbit_omega=u_f-theta_f;

%Bring back true anomalies within 0, 2_pi

theta_i=mod(theta_i,2*pi);
theta_f=mod(theta_f,2*pi);

%final_orbit.i=final_orbit.i;
%final_orbit.Omega=final_orbit.Omega;

%Use energy to get velocity
%E=0.5*v^2+mu/r;

% Get v_theta and delta_v required . Calcolo v_theta e deltaV richiesto.
vvtheta=sqrt(GM/(initial_orbit.a*(1-initial_orbit.e^2)))*(1+initial_orbit.e*cos(theta_i));
delta_v=2*sin(theta_man/2)*vvtheta;

intermediate_orbit.a = initial_orbit.a;
intermediate_orbit.e = initial_orbit.e;
intermediate_orbit.i = final_orbit.i;
intermediate_orbit.Omega = final_orbit.Omega;
intermediate_orbit.omega = intermediate_orbit_omega; % This way fields appear in the correct order
intermediate_orbit.theta = theta_f;
intermediate_orbits = {intermediate_orbit};

total_time = delta_t_on_orbit(initial_orbit, theta_i, GM);

manoeuver.true_anomaly_before=theta_i;
manoeuver.true_anomaly_after = theta_f;
manoeuver.delta_v = delta_v;
manoeuver.time = total_time;
manoeuver.type = "Change of plane";
manoeuver.location = op2rv(intermediate_orbit.a, intermediate_orbit.e, intermediate_orbit.i, intermediate_orbit.Omega, intermediate_orbit.omega, intermediate_orbit.theta, GM);

manoeuvers = {manoeuver};

return