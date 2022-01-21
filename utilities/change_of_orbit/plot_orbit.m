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

function [] = plot_orbit(orbit, initial_theta, final_theta, GM, axis_handle,plot_velocities,plot_body)

% PLOT_ORBIT: plots an orbit around a body
% Syntax: [] = plot_orbit(orbit, initial_theta, final_theta, GM, figure_handle,plot_velocities,plot_texture)
% Inputs:
% - orbit is a struct containing six parameters, a, e, i, Omega, omega, and
%    theta, encoding the orbits's Keplerian parameters;
% - initial_theta is the real anomaly at the start of plotting;
% - final_theta is the real anomaly at the end of plotting;
% - GM (optional, default 3.986*1e5) is the gravitational constant of the
%    body under consideration (passed to op2rv). Defaults to Earth;
% - figure_handle (optional): the handle of the figure where to plot; If
%    not specified, a new figure is created.
% - plot_velocities (optional, default False): a Boolean specifying whether
%    to plot velocity vectors too.
% - plot_body (optional): a function to call to plot the center body.
%    Should take no inputs.

if nargin<7
    plot_body = @() 0;
end
if nargin<6
    plot_velocities = false;
end
if nargin<5
    figure_handle = figure();
    axis_handle = axes();
end
if nargin<4
    GM = 3.986*1e5;
end
if nargin<3
    final_theta = 2*pi;
end
if nargin<2
    initial_theta = 0;
end

if nargin<1
	disp('Not enough parameters were specified, quitting...')
	return
end

% Unpack orbit
a = orbit.a;
e = orbit.e;
i_anomaly = orbit.i;
Omega = orbit.Omega;
omega = orbit.omega;
theta = orbit.theta;

% Initial radius
% r=a*(1-e^2)/(1+e*cos(initial_theta));
% rvec=[r*cos(initial_theta); r*sin(theta); 0];

while final_theta<initial_theta
    final_theta=final_theta+2*pi;
end

% Theta for plotting trajectory
theta_plot_trajectory=linspace(initial_theta,final_theta);
% Theta for plotting velocities
theta_plot_velocities=linspace(initial_theta,final_theta,30);

[rplot_vec, ~] = op2rv(a,e,i_anomaly,Omega,omega,theta_plot_trajectory,GM);

[rplot_vel_vec, vplot_vec] = op2rv(a,e,i_anomaly,Omega,omega,theta_plot_velocities,GM);


% % Radii
% rplot=a*(1-e^2)./(1+e.*cos(theta_plot_trajectory));
% rplot_vec=[rplot.*cos(theta_plot_trajectory); rplot.*sin(theta_plot_trajectory); zeros(size(theta_plot_trajectory))];
% 
% % Velocities
% vr=sqrt(GM/(a*(1-e^2)))*e*sin(theta);
% vt=sqrt(GM/(a*(1-e^2)))*(1+e*cos(theta));
% 
% vrplot=sqrt(GM/(a*(1-e^2))).*e.*sin(theta_plot_velocities);
% vtplot=sqrt(GM/(a*(1-e^2))).*(1+e.*cos(theta_plot_velocities));
% 
% vvec=[vr*cos(theta)-vt*sin(theta); vr*sin(theta)+vt*cos(theta);0];
% vplot_vec=[vrplot.*cos(theta_plot_velocities)-vtplot.*sin(theta_plot_velocities); vrplot.*sin(theta_plot_velocities)+vtplot.*cos(theta_plot_velocities);zeros(size(theta_plot_velocities))];
% 
% T=rotmat(omega,3)*rotmat(i_anomaly,1)*rotmat(Omega,3);
% Tm=T';
% 
% rvec=Tm*rvec;
% vvec=Tm*vvec;
% rplot_vec=Tm*rplot_vec;
% vplot_vec=Tm*vplot_vec;

axes(axis_handle)
hold on;

plot3(rplot_vec(1,:),rplot_vec(2,:),rplot_vec(3,:))
plot_body();

plot3(rplot_vec(1,1),rplot_vec(2,1),rplot_vec(3,1),'+');
axis equal;

xlabel('[km]');
ylabel('[km]');
zlabel('[km]');

if plot_velocities
    plot3(rplot_vec(1,:),rplot_vec(2,:),rplot_vec(3,:))
    quiver3(rplot_vel_vec(1,:),rplot_vel_vec(2,:),rplot_vel_vec(3,:),vplot_vec(1,:),vplot_vec(2,:),vplot_vec(3,:));
end
hold off;