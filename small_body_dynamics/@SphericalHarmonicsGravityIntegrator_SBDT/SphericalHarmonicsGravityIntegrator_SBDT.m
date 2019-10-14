%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

classdef SphericalHarmonicsGravityIntegrator_SBDT
    % A class to integrate trajectories of spacecraft around SSSBs. Uses a
    % spherical harmonics model to capture the gravity field around the
    % small body.
    
    properties
        BodyModel
        constants
    end
    
    methods
        function obj = SphericalHarmonicsGravityIntegrator_SBDT(BodyModel, constants)
            % Constructor. Takes two inputs:
            %  BodyModel, a model of the asteroid in SBDT format.
            %  constants, SBDT constants
            if nargin<2
                constants = initialize_SBDT(); 
            end
            if nargin<1
                BodyModel = loadEros( constants, 1, 1, 4, 3 );
            end
            obj.BodyModel = BodyModel;
            obj.constants = constants;
        end
        
        function [time, absolute_traj,relative_traj, mode, state_transition_matrix] = integrate(obj,time_horizon,start_state,mode)
            % Integrates the trajectory of a spacecraft orbiting the small
            % body.
            % Syntax: [tout, absolute_traj,relative_traj, mode, state_transition_matrix] = integrate(time_horizon,start_state, mode)
            % The spacecraft's starting state is start_state (x, y, z, vx,
            % vy, vz). The integration horizon is time_horizon.
            % Mode is 'absolute' (default) or 'relative'. In absolute mode,
            % integrates in the inertial reference frame. In relative mode,
            % integrates in the body-fixed reference frame.
            % The output is:
            % - tout, a grid of integration points in time.
            % - absolute_traj, the spacecraft location and velocity in the
            %    inertial body_centered grame
            % - relative_traj, the spacecraft location and velocity in the
            %    body_centered, body-fixed frame.
            % - mode, the mode of integration (identical to the input if
            %    set)
            % - state_transition_matrix, the state transition matrix in the
            %    reference frame specified by "mode"
            
            if nargin<4
                mode = 'absolute';
            end
            
            omega = [0;0;obj.BodyModel.bodyFrame.pm.w];
            if strcmp(mode,'absolute')
                [time, absolute_traj, state_transition_matrix] = integrate_absolute(obj,time_horizon,start_state);
                relative_traj = zeros(size(absolute_traj));
                for t_ix=1:length(time)
                    % Is this the actual initial angle? Unsure
                    rot_angle_z = obj.BodyModel.bodyFrame.pm.w0+ obj.BodyModel.bodyFrame.pm.w * time(t_ix);
                    Rot_i2b = obj.rotmat(rot_angle_z,3);
                    relative_traj(t_ix,1:3) = (Rot_i2b*absolute_traj(t_ix,1:3)')';
                    relative_traj(t_ix,4:6) = (Rot_i2b*(absolute_traj(t_ix,4:6) - cross(omega,absolute_traj(t_ix,1:3)))')';
                end
                
            elseif strcmp(mode,'relative')
                [time, relative_traj, state_transition_matrix] = integrate_relative(obj,time_horizon,start_state);
                absolute_traj = zeros(size(relative_traj));
                for t_ix=1:length(time)
                    rot_angle_z = obj.BodyModel.bodyFrame.pm.w0+ obj.BodyModel.bodyFrame.pm.w * time(t_ix);
                    Rot_b2i = obj.rotmat(-rot_angle_z,3);
                    absolute_traj(t_ix,1:3) = (Rot_b2i*relative_traj(t_ix,1:3)')';
                    absolute_traj(t_ix,4:6) = (Rot_b2i*(relative_traj(t_ix,4:6) + cross(omega,relative_traj(t_ix,1:3)))')';
                end
                
            else
                error('Integration mode not recognized')
            end
        end
        
        function [time, absolute_traj,state_transition_matrix] = integrate_absolute(obj,time_horizon,start_state)
            % Integrates the trajectory of a spacecraft orbiting the small
            % body.
            % Syntax: [tout, absolute_traj,state_transition_matrix] = integrate_absolute(time_horizon,start_state)
            % The spacecraft's starting state is start_state (x, y, z, vx,
            % vy, vz). The integration horizon is time_horizon.
            % The output is:
            % - tout, a grid of integration points in time.
            % - absolute_traj, the spacecraft location and velocity in the
            %    inertial body_centered grame
            % - state_transition_matrix, the state transition matrix (see
            %    Scheeres)
            
            sun = loadSun(obj.constants, 1, 1, 1, 1);
            simControls = [];
            partials.names = {'x'};
            [ time, absolute_traj, partials, ~, ~ ] = ...
                Inertial2BP_wPartials( time_horizon, start_state/1e3, [], obj.constants, ...
                           {sun;obj.BodyModel}, [], partials, simControls );
            absolute_traj = absolute_traj'*1e3;
            state_transition_matrix = partials.dxf_dp{1};

        end
        
        function [time,relative_traj,state_transition_matrix] = integrate_relative(obj,time_horizon,start_state_absolute)
            % Integrates the trajectory of a spacecraft orbiting the small
            % body.
            % Syntax: [tout, relative_traj, state_transition_matrix] = integrate_relative(time_horizon,start_state)
            % The spacecraft's starting state is start_state (x, y, z, vx,
            % vy, vz). The integration horizon is time_horizon.
            % The output is:
            % - tout, a grid of integration points in time.
            % - relative_traj, the spacecraft location and velocity in the
            %    body_centered, body-fixed frame.
            % - state_transition_matrix, the state transition matrix (see
            %    Scheeres)
            
            start_state_absolute = start_state_absolute/1e3;
            rot_angle_z = obj.BodyModel.bodyFrame.pm.w0+ obj.BodyModel.bodyFrame.pm.w * time_horizon(1);
            Rot_i2b = obj.rotmat(rot_angle_z,3);
            omega = [0;0;obj.BodyModel.bodyFrame.pm.w];
            
            start_state_relative = zeros(size(start_state_absolute));
            start_state_relative(1:3) = (Rot_i2b*(start_state_absolute(1:3)));
            start_state_relative(4:6) = (Rot_i2b*(start_state_absolute(4:6) - cross(omega,start_state_absolute(1:3))));
            
            sun = loadSun(obj.constants, 1, 1, 1, 1);
            simControls = [];
            partials.names = {'x'};
            [ time, relative_traj, partials, ~, ~ ] = ...
                Rot2BP_wPartials( time_horizon, start_state_relative, [], obj.constants, ...
                                   {sun;obj.BodyModel}, [], partials, simControls );
            relative_traj = relative_traj'*1e3;
            state_transition_matrix = partials.dxf_dp{1};
        end
        
        function [] = plot_absolute_traj(obj,time,traj,plotBody,handle,plot_velocity)
            % Plot the absolute trajectory of the spacecraft.
            % Syntax: plot_absolute_traj(time,traj,plotObject,handle)
            % Inputs: time, absolute trajectory (from Integrate).
            % If plotBody is set to True, the location of the SSSB is
            % also plotted for each time instant.
            % If handle is passed, plots in figure handle.
            if nargin<6
                plot_velocity=0;
            end
            if nargin<5
                handle = figure();
            else
                figure(handle);
            end
            if nargin<4
                plotBody = 0;
            end
            traj = traj/1e3;
            plot3(traj(:,1),traj(:,2),traj(:,3));
            hold all
            if (plot_velocity==1)
                quiver3(traj(:,1),traj(:,2),traj(:,3),traj(:,4),traj(:,5),traj(:,6));
            end
            title('Absolute reference frame')
            axis equal
            
            if plotBody
                for t_ix=1:length(time)
                    rot_angle_z = obj.BodyModel.bodyFrame.pm.w0+ obj.BodyModel.bodyFrame.pm.w * time(t_ix);
                    Rot_i2b = obj.rotmat(-rot_angle_z,3);  % I always get rotations wrong  - F
                    settings.rotM = Rot_i2b;
                    settings.fig = handle;
                    plotPolyhedron(obj.BodyModel.shape,settings)
                    hold all
                end
            else
                xlabel('X (km)')
                ylabel('Y (km)')
                zlabel('Z (km)')
            end
            
        end
        
        function [] = plot_relative_traj(obj,time,traj,plotBody,handle,plot_velocity)
            % Plot the relative trajectory of the spacecraft w.r.t. the
            % SSSB.
            % Syntax: plot_relative_traj(time,traj,plotObject,handle)
            % If handle is passed, plots in figure handle.
            if nargin<6
                plot_velocity = 0;
            end
            if nargin<5
                handle = figure();
            else
                figure(handle);
            end
            if nargin<4
                plotBody = 0;
            end
            traj = traj/1e3;
            plot3(traj(:,1),traj(:,2),traj(:,3))
            hold all
            if (plot_velocity==1)
                quiver3(traj(:,1),traj(:,2),traj(:,3),traj(:,4),traj(:,5),traj(:,6));
            end
            hold all
            title('Relative reference frame')
            
            if plotBody
                settings.fig = handle;
                plotPolyhedron(obj.BodyModel.shape,settings)
            end
            axis equal
        end
        function [R]=rotmat(obj,angle,axis)
            %Syntax: [R]=rotmat(angle,axis)
            %Computes rotation matrix for a single rotation around one of the three axes x, y, z in a 3d space.
            % In order to stack rotations, call the function several times. Remember, however, that after the first rotation you'll turn around the NEW axes, which will be different from the old ones.
            % The axis parameter can be x (or i, or 1), y (or j, or 2), z (or k, or 3). Angle is given in radians.
            if axis~='x' && axis~='y' && axis~='z'  && axis~='i'  && axis~='j' && axis~='k' &&axis~=1 && axis~=2 && axis~=3 || nargin<2
                disp('Invalid axis specified, returning identity matrix')
                R=eye(3);
                return
            end
            
            if axis=='x' || axis=='i' || axis==1
                R=[ 1 0 0; 0 cos(angle) sin(angle); 0 -sin(angle) cos(angle)];
            elseif axis=='y' || axis=='j' || axis==2
                R=[cos(angle) 0 -sin(angle); 0 1 0; sin(angle) 0 cos(angle)];
            elseif axis=='z' || axis=='k' || axis==3
                R=[cos(angle) sin(angle) 0; -sin(angle) cos(angle) 0; 0 0 1];
            end
        end
    end
end