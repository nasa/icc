%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%       A function to plot orbits with labeled angles and vectors         %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
%                                                                             %
% This  software  is a copy  and  may not be current.  The latest  version is %
% maintained by and may be obtained from the Mobility  and  Robotics  Sytstem %
% Section (347) at the Jet  Propulsion  Laboratory.   Suggestions and patches %
% are welcome and should be sent to the software's maintainer.                %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = plot_labeled_orbit(orbit, GM, figure_handle)
% PLOT_LABELED_ORBIT(orbit, figure_handle)

if nargin<3
    figure()
else
    figure(figure_handle);
end
hold all
% Plot the orbit

%plot_orbit(orbit, 0, 2*pi, GM)
theta_plot_trajectory=linspace(0,2*pi);
[r_plot, ~] = op2rv(orbit.a, orbit.e, orbit.i, orbit.Omega, orbit.omega, theta_plot_trajectory, GM);

orbit_handle = patch(r_plot(1,:),r_plot(2,:),r_plot(3,:), [1, 0, 0]);


% Plot H
[r, v] = op2rv(orbit.a, orbit.e, orbit.i, orbit.Omega, orbit.omega, orbit.theta, GM);
h = cross(r,v);

sat_handle = plot3(r(1),r(2),r(3),'ok', 'MarkerSize', 5);
vel_scale = 0.25*norm(r);
v_plot = v/norm(v)*vel_scale;
sat_vel_handle = quiver3(r(1),r(2),r(3), v_plot(1), v_plot(2), v_plot(3), 'k');

default_quiver_scale = norm(r);

h_plot = h/norm(h)*default_quiver_scale;
h_handle = quiver3(0, 0, 0, h_plot(1), h_plot(2), h_plot(3),'c','LineWidth',2);

% Plot the eccentricity vector
e_vec = ((norm(v)^2-GM/norm(r)).*r-(dot(r,v).*v))./GM;

e_plot_norm = norm(op2rv(orbit.a, orbit.e, orbit.i, orbit.Omega, orbit.omega, 0, GM));

e_vec = e_vec/norm(e_vec)*e_plot_norm;

e_handle = quiver3(0,0,0,e_vec(1),e_vec(2),e_vec(3),'m','LineWidth',2);
%e_handle = plot3([0, e_vec(1)], [0, e_vec(2)], [0, e_vec(3)]);

% Plot the ascending node
ascending_node = rotmat(-orbit.Omega,3)*[1;0;0];

ascending_node_plot = ascending_node * default_quiver_scale;

ascending_node_handle = quiver3(0,0,0,ascending_node_plot(1),ascending_node_plot(2),ascending_node_plot(3),'y','LineWidth',2);

% Plot a lightly-shaded ecliptic plane
max_x = max(r_plot(1,:));
min_x = min(r_plot(1,:));
max_y = max(r_plot(2,:));
min_y = min(r_plot(2,:));

scale_ecliptic = 1.02;

ecliptic_handle = patch([scale_ecliptic*min_x, scale_ecliptic*max_x, scale_ecliptic*max_x, scale_ecliptic*min_x], [scale_ecliptic*min_y, scale_ecliptic*min_y, scale_ecliptic*max_y, scale_ecliptic*max_y], [0,0,0,0], [.1 .1 .1]);


alpha(0.1)
% Plot unit vectors
unit_vector_scale = .5*default_quiver_scale;
x_handle = quiver3(0,0,0,unit_vector_scale,0,0,'r');
y_handle = quiver3(0,0,0,0,unit_vector_scale,0,'g');
z_handle = quiver3(0,0,0,0,0,unit_vector_scale,'b');

legend([sat_handle,h_handle, e_handle, ascending_node_handle, x_handle, y_handle, z_handle ], ...
    'Satellite', 'H', 'Eccentricity', 'Ascending node','x', 'y', 'z')
axis equal
view(3)

% legend([orbit_handle, h_handle, e_handle, ascending_node_handle, ecliptic_handle], ...
%     'Orbit', 'H', 'Eccentricity', 'Ascending node', 'Plane of the ecliptic')
