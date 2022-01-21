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

function [dT] = delta_t_on_orbit(orbit, final_theta, GM)
% DELTA_T_ON_ORBIT compute the delta t between two true anomalies on an orbit
% Syntax: [delta_t] = delta_t_on_orbit(orbit, final_theta, GM)
% Inputs:
% - orbit, an orbit struct with six Keplerian parameters.
% - final_theta, the final true anomaly on the orbit
% - GM, the gravitational constant
% Output:
% - delta_t, the time required to travel from orbit.theta to final_theta.

% Copied from deltaT_orbit.m in Tycho

E_i=2*atan(sqrt((1-orbit.e)/(1+orbit.e))*tan(orbit.theta/2));
if orbit.theta>pi
	E_i=E_i+2*pi;
end
T_i=sqrt(orbit.a^3/GM).*(E_i-orbit.e*sin(E_i));



E_f=2*atan(sqrt((1-orbit.e)/(1+orbit.e))*tan(final_theta/2));
if final_theta>pi
	E_f=E_f+2*pi;
end
T_f=sqrt(orbit.a^3/GM).*(E_f-orbit.e*sin(E_f));

dT=T_f-T_i;

%If theta_f<theta_i, this could happen.
% It means that one has to go through the pericenter to get to the final orbit.
%If initial data are VERY fucked up, one may have to do this multiple times.
while dT<0 
	dT=dT+2*pi*sqrt(orbit.a^3/GM);
end

return