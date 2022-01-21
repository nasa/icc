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

function [delta_v, total_time, intermediate_orbits, manoeuvers] = hohmann_transfer(initial_orbit, final_orbit, GM, aim_for_apoapsis, same_argument_of_periapsis)
% HOHMANN_TRANSFER computes the Hohmann transfer between two orbits
% Syntax: [delta_v, total_time, intermediate_orbits, manoeuvers] =
%  hohmann_transfer(initial_orbit, final_orbit, GM, apoapsis_or_periapsis, same_argument_of_periapsis)
% Computes a Hohmann transfer between initial_orbit and an orbit with semi-major
%  axis and eccentricity corresponding to final_orbit (but in the same plane as
%  initial_orbit).
% If apoapsis_or_periapsis is true, the transfer aims for the final orbit's
% apoapsis. If it is false, the transfer aims for the final orbit's periapsis.
% if same_argument_of_periapsis is true, the output orbit has the same argument
%  of periapsis as the initial one. If it is false, the output orbit has an
%  argument of periapsis shifted by pi.

% From Bitang_transfer_i.m in Tycho

% Unchanged variables
bitangent_orbit.i=initial_orbit.i;
bitangent_orbit.Omega=initial_orbit.Omega;

post_manoeuver_orbit.a = final_orbit.a;
post_manoeuver_orbit.e = final_orbit.e;
post_manoeuver_orbit.i = initial_orbit.i;
post_manoeuver_orbit.Omega = initial_orbit.Omega;

%/Unchanged variables

%Meat

if same_argument_of_periapsis
	post_manoeuver_orbit.omega = initial_orbit.omega;
else
	post_manoeuver_orbit.omega = mod(initial_orbit.omega+pi, 2*pi);
end

if aim_for_apoapsis==1 %Shooting for the final orbit's apocenter
	r_f=final_orbit.a*(1+final_orbit.e);
	post_manoeuver_orbit.theta=pi;
	if same_argument_of_periapsis  %Aligned orbits: will start from the initial orbit's pericenter
 		r_i=initial_orbit.a*(1-initial_orbit.e);
 		theta_i=0;
	else %apo to apo
		r_i=initial_orbit.a*(1+initial_orbit.e);
		theta_i=pi;
	end
 
	if r_f>r_i  %Bitangent has pericenter in r_i, apocenter in r_f
		%bitangent_orbit.a*(1+bitangent_orbit.e)=r_f
		%bitangent_orbit.a*(1-bitangent_orbit.e)=r_i
		bitangent_orbit.a=(r_f+r_i)/2;
		bitangent_orbit.e=(r_f-r_i)/(r_f+r_i);
		bitangent_orbit.omega=post_manoeuver_orbit.omega; %Hohmann's apocenter is where the final orbit's apocenter is: they are aligned
		theta_bt_i=0;
		theta_bt_f=pi;
	else %Bitangent has pericenter in r_f, apocenter in r_i
		%bitangent_orbit.a*(1+bitangent_orbit.e)=r_i for apo
		%bitangent_orbit.a*(1-bitangent_orbit.e)=r_f for peri
		bitangent_orbit.a=(r_f+r_i)/2;
		bitangent_orbit.e=(r_i-r_f)/(r_i+r_f);
		bitangent_orbit.omega=mod(post_manoeuver_orbit.omega+pi, 2*pi);   %Bitangent has pericenter in r_f, the final orbit's apocenter: it is opposite to it
		theta_bt_i=pi;
		theta_bt_f=2*pi;
	end
else %Shooting for the final orbit's pericenter
	r_f=final_orbit.a*(1-final_orbit.e);
	post_manoeuver_orbit.theta=0;
	if same_argument_of_periapsis %Aligned orbits: will start from the initial orbit's apocenter
		r_i=initial_orbit.a*(1+initial_orbit.e);
		theta_i=pi;
	else %Opposite orbits: pericenter to pericenter
		r_i=initial_orbit.a*(1-initial_orbit.e);
		theta_i=0;
	end
	if r_f>r_i %Bt orbit has apocenter in r_f, pericenter in r_i
		%bitangent_orbit.a*(1+bitangent_orbit.e)=r_f
		%bitangent_orbit.a*(1-bitangent_orbit.e)=r_i
		bitangent_orbit.a=(r_f+r_i)/2;
		bitangent_orbit.e=(r_f-r_i)/(r_f+r_i);
		bitangent_orbit.omega=mod(post_manoeuver_orbit.omega+pi, 2*pi); %BT orbit has apocenter in r_f, where the final orbit's pericenter is
		theta_bt_i=0;
		theta_bt_f=pi;
	else %r_f<r_i, bt transfer has apocenter in r_i and pericenter in r_f
		%bitangent_orbit.a*(1+bitangent_orbit.e)=r_i
		%bitangent_orbit.a*(1-bitangent_orbit.e)=r_f
		bitangent_orbit.a=(r_f+r_i)/2;
		bitangent_orbit.e=(r_i-r_f)/(r_i+r_f);
		bitangent_orbit.omega=post_manoeuver_orbit.omega; %bt orbit has pericenter where the final orbit's pericenter is
		theta_bt_i=pi;
		theta_bt_f=2*pi;
	end
end

%Velocities
%Initial orbit
[~,v_i]=op2rv(initial_orbit.a,initial_orbit.e,initial_orbit.i,initial_orbit.Omega,initial_orbit.omega,theta_i,GM);

%Bitangent, beginning
[~,v_bt_i]=op2rv(bitangent_orbit.a,bitangent_orbit.e,bitangent_orbit.i,bitangent_orbit.Omega,bitangent_orbit.omega,theta_bt_i,GM);
%Bitangent, end
[~,v_bt_f]=op2rv(bitangent_orbit.a,bitangent_orbit.e,bitangent_orbit.i,bitangent_orbit.Omega,bitangent_orbit.omega,theta_bt_f,GM);

%Final orbit
[~,v_f]=op2rv(final_orbit.a,final_orbit.e,initial_orbit.i,initial_orbit.Omega,post_manoeuver_orbit.omega,post_manoeuver_orbit.theta,GM);

%DeltaVs
deltaV1=norm(v_bt_i-v_i);
deltaV2=norm(v_f-v_bt_f);
delta_v = deltaV1+deltaV2;

bitangent_orbit.theta = theta_bt_i;

delta_t_1 = delta_t_on_orbit(initial_orbit, theta_i, GM);
delta_t_2 = delta_t_on_orbit(bitangent_orbit, theta_bt_f, GM);

total_time = delta_t_1+delta_t_2;


intermediate_orbits = {bitangent_orbit; post_manoeuver_orbit};

first_manoeuver.delta_v = deltaV1;
first_manoeuver.time = delta_t_1;
first_manoeuver.true_anomaly_before = theta_i;
first_manoeuver.true_anomaly_after = bitangent_orbit.theta;
first_manoeuver.type = "Hohmann 1/2";
first_manoeuver.location = op2rv(bitangent_orbit.a, bitangent_orbit.e, bitangent_orbit.i, bitangent_orbit.Omega, bitangent_orbit.omega, bitangent_orbit.theta, GM);

second_manoeuver.delta_v = deltaV2;
second_manoeuver.time = delta_t_2;
second_manoeuver.true_anomaly_before = theta_bt_f;
second_manoeuver.true_anomaly_after = post_manoeuver_orbit.theta;
second_manoeuver.type = "Hohmann 2/2";
second_manoeuver.location = op2rv(bitangent_orbit.a, bitangent_orbit.e, bitangent_orbit.i, bitangent_orbit.Omega, bitangent_orbit.omega, theta_bt_f, GM);

manoeuvers = {first_manoeuver; second_manoeuver};

return