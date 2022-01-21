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

function [delta_v, total_time, intermediate_orbits, manoeuvers] = change_argument_of_periapsis(initial_orbit, final_orbit, GM)
% CHANGE_ARGUMENT_OF_PERIAPSIS computes the maneuver to change the periapsis of an orbit
% Syntax: [delta_v, total_time, intermediate_orbits, manoeuvers] =
%  change_argument_of_periapsis(initial_orbit, final_orbit, GM)
% Computes a maneuver to change the periapsis of initial_orbit to match the 
% argument of periapsis of the final orbit. The semi-major axis, eccentricity,
% and plane of the orbit remains the same as initial_orbit.

% Copied from deltaomega_mod.m in Tycho

intermediate_orbit.a = initial_orbit.a;
intermediate_orbit.e = initial_orbit.e;
intermediate_orbit.i = initial_orbit.i;
intermediate_orbit.Omega = initial_orbit.Omega;


delta_omega=final_orbit.omega - initial_orbit.omega;
manoeuver.true_anomaly_before=delta_omega/2;
intermediate_orbit.theta= 2 * pi - delta_omega/2;
manoeuver.true_anomaly_after = intermediate_orbit.theta;

intermediate_orbit.omega = final_orbit.omega;

%p=ra*(1-e)
%p=rp*(1+e)
%(rp+ra)/2=a
b=[0;0;initial_orbit.a];
A=[1, -(1-initial_orbit.e), 0; ...
   1,0,-(1+initial_orbit.e); ...
   0,1/2,1/2];
%x=[p,ra,rp]
x=A\b;
p=x(1);

%p = h^2/mu
%h=rp*vteta

delta_v = abs(2*sqrt(GM/p)*initial_orbit.e*sin((intermediate_orbit.omega-initial_orbit.omega)/2));
total_time = delta_t_on_orbit(initial_orbit, manoeuver.true_anomaly_before, GM);

manoeuver.delta_v = delta_v;
manoeuver.time = total_time;
manoeuver.type = "Change argument of periapsis";
manoeuver.location = op2rv(intermediate_orbit.a, intermediate_orbit.e, intermediate_orbit.i, intermediate_orbit.Omega, intermediate_orbit.omega, intermediate_orbit.theta, GM);

manoeuvers = {manoeuver};
intermediate_orbits = {intermediate_orbit};

return
