% Test optimal orbit transfer

clear all; close all; clc; run ../../startup.m

addpath(genpath("../transformations"));


addpath(genpath(strcat(ROOT_PATH,'/small_body_dynamics/EROS 433')))
addpath(strcat(ROOT_PATH,'/small_body_dynamics'))
addpath(strcat(ROOT_PATH,'/visualization'))
addpath(genpath(strcat(ROOT_PATH,'/utilities'))) % Add all utilities
userModelsPath = strcat(SBDT_PATH,'/ExampleUserModels');
constantsModel = 1;
addpath(strcat(SBDT_PATH,'/Startup'));

bodies = ["Earth", "Eros"];

%% 
for body = bodies
    if strcmp(body, "Earth")
        r_i=[6735.6576 115.6422 -421.1886]; %km
        r_f=[15642.3563 2456.7486 -3042.4071];

        v_i=[-0.2624 7.6872 -0.3489]; %km/s
        v_f=[0.4239 5.0972 -0.8893]; %km/s

        % We are on Earth
        GM = 3.986*1e5;

        % Convert initial and final position to orbital parameters
        [a_i, e_i, i_i, Omega_i, omega_i, theta_i]=rv2op(r_i,v_i,GM);
        [a_f, e_f, i_f, Omega_f, omega_f, theta_f]=rv2op(r_f,v_f,GM);

        initial_orbit.a = a_i;
        initial_orbit.e = e_i;
        initial_orbit.i = i_i;
        initial_orbit.Omega = Omega_i;
        initial_orbit.omega = omega_i;
        initial_orbit.theta = theta_i;

        final_orbit.a = a_f;
        final_orbit.e = e_f;
        final_orbit.i = i_f;
        final_orbit.Omega = Omega_f;
        final_orbit.omega = omega_f;
        final_orbit.theta = theta_f;

        plot_body = @plot_earth;
    elseif strcmp(body, "Eros")
        constants = addSBDT(SBDT_PATH, userModelsPath, constantsModel);
        eros_sbdt = loadEros( constants, 1, 1, 4, 3 );
        ErosModel = SphericalHarmonicsGravityIntegrator_SBDT(eros_sbdt, constants);

        rng default
        carrier_orbit_rv = initialize_carrier_orbit(ErosModel);
        final_orbit_rv = initialize_random_orbits(1, ErosModel); % [m, m/s]
        
        r_i=carrier_orbit_rv(1:3)*1e-3; %km
        r_f=final_orbit_rv(1:3)*1e-3;

        v_i=carrier_orbit_rv(4:6)*1e-3; %km/s
        v_f=final_orbit_rv(4:6)*1e-3; %km/s

        % We are on Eros
        GM = ErosModel.BodyModel.gravity.gm;

        % Convert initial and final position to orbital parameters
        [a_i, e_i, i_i, Omega_i, omega_i, theta_i]=rv2op(r_i,v_i,GM);
        [a_f, e_f, i_f, Omega_f, omega_f, theta_f]=rv2op(r_f,v_f,GM);

        initial_orbit.a = a_i;
        initial_orbit.e = e_i+0.001;
        initial_orbit.i = i_i;
        initial_orbit.Omega = Omega_i;
        initial_orbit.omega = omega_i;
        initial_orbit.theta = theta_i;
        
%         if abs(initial_orbit.e) < 10*eps
%             initial_orbit.e = 1e-4;
%         end
%         if abs(initial_orbit.Omega)  < 10*eps
%             initial_orbit.Omega = 1e-4;
%         end
%         if abs(initial_orbit.omega)  < 10*eps
%             initial_orbit.omega = 1e-4;
%         end
%         if abs(initial_orbit.i)  < 10*eps
%             initial_orbit.i = 1e-4;
%         end
%         if abs(final_orbit.e) < 10*eps
%             final_orbit.e = 1e-4;
%         end
%         if abs(final_orbit.Omega)  < 10*eps
%             final_orbit.Omega = 1e-4;
%         end
%         if abs(final_orbit.omega)  < 10*eps
%             final_orbit.omega = 1e-4;
%         end
%         if abs(final_orbit.i)  < 10*eps
%             final_orbit.i = 1e-4;
%         end
        
        final_orbit.a = a_f;
        final_orbit.e = e_f+0.001;
        final_orbit.i = i_f;
        final_orbit.Omega = Omega_f;
        final_orbit.omega = omega_f;
        final_orbit.theta = theta_f;

        plot_body = @() render_asteroid_3d(ErosModel, true, 0);

    end


    %% Test deltaT on orbit
    initial_orbit_t = initial_orbit;
    initial_orbit_t.theta = 0;
    dt = delta_t_on_orbit(initial_orbit_t, initial_orbit_t.theta, GM);
    assert(dt==0, "ERROR: time is weird");
    dt = delta_t_on_orbit(initial_orbit_t, initial_orbit_t.theta+pi, GM);
    assert(abs(dt-pi*sqrt(initial_orbit.a^3/GM))==0, "ERROR: time is weird");

    %% Test Hohmann transfer
    for aim_for_apoapsis = 0:1
        for same_argument_of_periapsis = 0:1
            [delta_v, total_time, intermediate_orbits, manoeuvers] = hohmann_transfer(initial_orbit, final_orbit, GM, aim_for_apoapsis, same_argument_of_periapsis);
            assert(intermediate_orbits{end}.a == final_orbit.a);
            assert(intermediate_orbits{end}.e == final_orbit.e);
            assert(intermediate_orbits{end}.i == initial_orbit.i);
            assert(intermediate_orbits{end}.Omega == initial_orbit.Omega);
            if same_argument_of_periapsis
                assert(intermediate_orbits{end}.omega == initial_orbit.omega);
            else
                assert(abs(abs(intermediate_orbits{end}.omega - initial_orbit.omega) - pi)< 1e-10);
            end
            first_burn_location_initial_orbit = op2rv(initial_orbit.a,initial_orbit.e,initial_orbit.i,initial_orbit.Omega,initial_orbit.omega,manoeuvers{1}.true_anomaly_before, GM);
            first_burn_location_intermediate_orbit = op2rv(intermediate_orbits{1}.a,intermediate_orbits{1}.e,intermediate_orbits{1}.i,intermediate_orbits{1}.Omega,intermediate_orbits{1}.omega,manoeuvers{1}.true_anomaly_after, GM);
            assert(norm(first_burn_location_initial_orbit-first_burn_location_intermediate_orbit)<1e-3, "ERROR: first Hohmann burn location does not match")

            second_burn_location_intermediate_orbit = op2rv(intermediate_orbits{1}.a,intermediate_orbits{1}.e,intermediate_orbits{1}.i,intermediate_orbits{1}.Omega,intermediate_orbits{1}.omega,manoeuvers{2}.true_anomaly_before, GM);
            second_burn_location_final_orbit = op2rv(intermediate_orbits{2}.a,intermediate_orbits{2}.e,intermediate_orbits{2}.i,intermediate_orbits{2}.Omega,intermediate_orbits{2}.omega,manoeuvers{2}.true_anomaly_after, GM);            
            assert(norm(second_burn_location_intermediate_orbit - second_burn_location_final_orbit)<1e-3, "ERROR: second Hohmann burn location does not match")

        end
    end



%     %% Test change of plane
%     [delta_v, total_time, intermediate_orbits, manoeuvers] = change_of_plane(initial_orbit, final_orbit, GM);
% 
%     assert(intermediate_orbits{end}.a == initial_orbit.a);
%     assert(intermediate_orbits{end}.e == initial_orbit.e);
% 
%     assert(intermediate_orbits{end}.Omega == final_orbit.Omega);
%     assert(intermediate_orbits{end}.i == final_orbit.i);
%     burn_location_pre = op2rv(initial_orbit.a,initial_orbit.e,initial_orbit.i,initial_orbit.Omega,initial_orbit.omega,manoeuvers{1}.true_anomaly_before, GM);
%     burn_location_intermediate = op2rv(intermediate_orbits{1}.a,intermediate_orbits{1}.e,intermediate_orbits{1}.i,intermediate_orbits{1}.Omega,intermediate_orbits{1}.omega,manoeuvers{1}.true_anomaly_after, GM);
%     try
%         assert(norm(burn_location_pre-burn_location_intermediate)<1e-3, "ERROR: Change of plane burn location is inconsistent")
%         %ERROR if final Omega is greater than pi. THis is a bug in Tycho,
%         %in all likelihood.
%     catch ME
%         figure()
%         axes = subplot(1, 1, 1);
%         intermediate_orbits = [intermediate_orbits, 'dummy'];
%         dummy_manoeuver.true_anomaly_before = pi;
%         dummy_manoeuver.true_anomaly_after = 2*pi;
%         dummy_manoeuver.location = [0,0,0];
%         dummy_manoeuver.type = '';
%         manoeuvers = [manoeuvers; dummy_manoeuver];
%        plot_transfer(initial_orbit, final_orbit, intermediate_orbits, manoeuvers, GM, axes, plot_body, false);
%        view(3)
%        axis equal
%         disp("Hang debugger here")
%         rethrow(ME);
%     end
%     
    %% Test change of plane vector
    [delta_v, total_time, intermediate_orbits, manoeuvers] = change_of_plane_vector(initial_orbit, final_orbit, GM);

    assert(intermediate_orbits{end}.a == initial_orbit.a);
    assert(intermediate_orbits{end}.e == initial_orbit.e);

    assert(intermediate_orbits{end}.Omega == final_orbit.Omega);
    assert(intermediate_orbits{end}.i == final_orbit.i);
    burn_location_pre = op2rv(initial_orbit.a,initial_orbit.e,initial_orbit.i,initial_orbit.Omega,initial_orbit.omega,manoeuvers{1}.true_anomaly_before, GM);
    burn_location_intermediate = op2rv(intermediate_orbits{1}.a,intermediate_orbits{1}.e,intermediate_orbits{1}.i,intermediate_orbits{1}.Omega,intermediate_orbits{1}.omega,manoeuvers{1}.true_anomaly_after, GM);
    try
        assert(norm(burn_location_pre-burn_location_intermediate)<1e-3, "ERROR: Change of plane burn location is inconsistent")
        %ERROR if final Omega is greater than pi. THis is a bug in Tycho,
        %in all likelihood.
    catch ME
        figure()
        axes = subplot(1, 1, 1);
        intermediate_orbits = [intermediate_orbits, 'dummy'];
        dummy_manoeuver.true_anomaly_before = pi;
        dummy_manoeuver.true_anomaly_after = 2*pi;
        dummy_manoeuver.location = [0,0,0];
        dummy_manoeuver.type = '';
        manoeuvers = [manoeuvers; dummy_manoeuver];
       plot_transfer(initial_orbit, final_orbit, intermediate_orbits, manoeuvers, GM, axes, plot_body, false);
       view(3)
       axis equal
        disp("Hang debugger here")
        rethrow(ME);
    end
    
    %% Test change of argument of periapsis
    [delta_v, total_time, intermediate_orbits, manoeuvers] = change_argument_of_periapsis(initial_orbit, final_orbit, GM);

    assert(intermediate_orbits{end}.a == initial_orbit.a);
    assert(intermediate_orbits{end}.e == initial_orbit.e);
    assert(intermediate_orbits{end}.i == initial_orbit.i);
    assert(intermediate_orbits{end}.Omega == initial_orbit.Omega);

    assert(intermediate_orbits{end}.omega == final_orbit.omega);

    %% Test change of plane and argument of periapsis
    [delta_v, total_time, intermediate_orbits, manoeuvers] = change_of_plane_and_argument_of_periapsis(initial_orbit, final_orbit, GM);

    assert(intermediate_orbits{end}.a == initial_orbit.a);
    assert(intermediate_orbits{end}.e == initial_orbit.e);
    assert(intermediate_orbits{end}.Omega == final_orbit.Omega);
    assert(intermediate_orbits{end}.i == final_orbit.i);
    assert(intermediate_orbits{end}.omega == final_orbit.omega);

    %% Test the enumerator
    tstart = tic;
    [ordered_delta_vs, ordered_transfer_times, ordered_intermediate_orbits, ordered_manoeuvers] = optimal_keplerian_orbit_transfer(initial_orbit, final_orbit, GM);
    tend = toc(tstart);
    fprintf("Time to try all considered strategies for %s: %f.\n",body,tend)
    
    % Plot in a subfig grid
    plt_rows = floor(sqrt(length(ordered_intermediate_orbits)));
    plt_columns = ceil(length(ordered_intermediate_orbits)/plt_rows);

    for option_ix = 1:length(ordered_intermediate_orbits)
       figure()
       axes = subplot(1, 1, 1);
       intermediate_orbits = ordered_intermediate_orbits{option_ix};
       intermediate_orbits = {intermediate_orbits{2:end}}; % The first orbit is just the initial orbit
       manoeuvers = ordered_manoeuvers{option_ix};
       plot_transfer(initial_orbit, final_orbit, intermediate_orbits, manoeuvers, GM, axes, plot_body);
       title({sprintf("\\fontsize{12}Strategy %d", option_ix), sprintf("\\fontsize{9}dV=%.4f km/s", ordered_delta_vs(option_ix)), sprintf("\\fontsize{9}dT=%.1f s",ordered_transfer_times(option_ix))});
        view(3)
        axis equal
    end
    
    plot_pareto_curve(initial_orbit, final_orbit, ordered_intermediate_orbits, ordered_transfer_times, ordered_delta_vs, ordered_manoeuvers, GM, body, plot_body);

end

function [] = plot_earth(figure_handle)
if nargin>0
    figure(figure_handle);
end
R = 6378;
plot_texture = 'earth.jpg';

res = 200;
[xs,ys,zs]=sphere(res);
He=imread(plot_texture);
He_sd=flip(imresize(double(He)/255,[res res]),1);
xs=xs*(R+10); %This should avoid bleeding of the previous brightly-coloured spheres when the last, textured one is applied.
ys=ys*(R+10);
zs=zs*(R+10);
surf(xs,ys,zs,He_sd,'edgecolor','none')

end

