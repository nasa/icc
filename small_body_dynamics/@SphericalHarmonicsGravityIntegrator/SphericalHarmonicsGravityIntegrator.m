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

classdef SphericalHarmonicsGravityIntegrator
    % A class to integrate trajectories of spacecraft around SSSBs. Uses a
    % spherical harmonics model to capture the gravity field around the
    % small body.
    
    properties
        SphericalModel
        RotationRate
        RotationEpoch
        ShapeModel
        Integrator
    end
    
    methods
        function obj = SphericalHarmonicsGravityIntegrator(SphericalModel,RotationRate,RotationEpoch,ShapeModel,Integrator)
            % Constructor. Takes five inputs:
            % - SphericalModel is a cell  array containing two elements,
            %    (i) a file containing the spherical coefficients and (ii)
            %    a handle to a file that reads the file and returns the
            %    coefficients in canonical format. The cell array is
            %    consumed directly by gravitysphericalharmonic.m; see the
            %    documentation in the function for details.
            % - RotationRate, the rotation rate of the body, in degrees/s.
            %    The body is assumed to rotate around the z axis.
            % - RotationEpoch, the rotation angle of the body at t=0.
            % - ShapeModel, a shape model of the body. ShapeModel is a
            %    struct containing two fields:
            %   - shapefilename is the name of the shape model. The model
            %    is in PDS3 format.
            %   - scale (optional) is the scale of the model w.r.t. meters.
            % - Integrator is a handle to an integrator function. The
            %    default integrator is ode45.
            if nargin<5
               Integrator = @ode45;
            end
            if nargin<4
                ShapeModel = [];
            end
            if nargin<3
                RotationEpoch = [];
            end
            if nargin<2
                RotationRate = 0;
            end
            if nargin<1
                SphericalModel = [];
            end
            obj.SphericalModel = SphericalModel;
            obj.RotationRate = RotationRate;
            obj.RotationEpoch = RotationEpoch;
            obj.ShapeModel = ShapeModel;
            obj.Integrator = Integrator;
        end
        
        function g_rel = g_relative(obj,time,x_rel)
            % Returns the gravity in the object's rotating body frame.
            % Input: a location vector in the body frame. Output: gravity
            % in the body frame.
            [g_rel_x,g_rel_y, g_rel_z]  = gravitysphericalharmonic(x_rel','custom',16,obj.SphericalModel,'None');
            g_rel = [g_rel_x,g_rel_y, g_rel_z]';
        end
        
        function g_abs = g_absolute (obj,t,x_abs)
            % Returns the gravity in the object's body-centered, inertial
            % frame.
            % Input: a location vector in the inertial frame.
            % Output: gravity in the inertial frame.
            rot_angle = obj.RotationEpoch + obj.RotationRate * t;
            Rot_i2b = obj.rotmat(rot_angle,'z');
            x_rel = Rot_i2b*x_abs;
            g_rel = obj.g_relative(t,x_rel);
            g_abs = Rot_i2b'*g_rel;
        end

        function [time, absolute_traj,relative_traj] = integrate(obj,time_horizon,start_state,mode)
            % Integrates the trajectory of a spacecraft orbiting the small
            % body.
            % Syntax: [tout, absolute_traj,relative_traj] = integrate(time_horizon,start_state, mode)
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
            
            if nargin<4
                mode = 'absolute';
            end
            
            omega = [0;0;obj.RotationRate];
            if strcmp(mode,'absolute')
                [time, absolute_traj] = integrate_absolute(obj,time_horizon,start_state);
                relative_traj = zeros(size(absolute_traj));
                for t_ix=1:length(time)
                    rot_angle_z = obj.RotationEpoch+ obj.RotationRate * time(t_ix);
                    Rot_i2b = obj.rotmat(rot_angle_z,3);
                    relative_traj(t_ix,1:3) = (Rot_i2b*absolute_traj(t_ix,1:3)')';
                    relative_traj(t_ix,4:6) = (Rot_i2b*(absolute_traj(t_ix,4:6) - cross(omega,absolute_traj(t_ix,1:3)))')';
                end
                
            elseif strcmp(mode,'relative')
                [time, relative_traj] = integrate_relative(obj,time_horizon,start_state);
                absolute_traj = zeros(size(relative_traj));
                for t_ix=1:length(time)
                    rot_angle_z = obj.RotationEpoch+ obj.RotationRate * time(t_ix);
                    Rot_b2i = obj.rotmat(-rot_angle_z,3);
                    absolute_traj(t_ix,1:3) = (Rot_b2i*relative_traj(t_ix,1:3)')';
                    absolute_traj(t_ix,4:6) = (Rot_b2i*(relative_traj(t_ix,4:6) + cross(omega,relative_traj(t_ix,1:3)))')';
                end
                
            else
                error('Integration mode not recognized')
            end
        end
        
        function [time, absolute_traj] = integrate_absolute(obj,time_horizon,start_state)
            % Integrates the trajectory of a spacecraft orbiting the small
            % body.
            % Syntax: [tout, absolute_traj,relative_traj] = integrate_absolute(time_horizon,start_state)
            % The spacecraft's starting state is start_state (x, y, z, vx,
            % vy, vz). The integration horizon is time_horizon.
            % The output is:
            % - tout, a grid of integration points in time.
            % - absolute_traj, the spacecraft location and velocity in the
            %    inertial body_centered grame
            % - relative_traj, the spacecraft location and velocity in the
            %    body_centered, body-fixed frame.
            
            [time,absolute_traj] = obj.Integrator(@obj.dstate_abs,time_horizon,start_state);
%             relative_traj = zeros(size(absolute_traj));
%             omega = [0;0;obj.RotationRate];
%             for t_ix=1:length(time)
%                 rot_angle_z = obj.RotationEpoch+ obj.RotationRate * time(t_ix);
%                 Rot_i2b = obj.rotmat(rot_angle_z,3);
%                 relative_traj(t_ix,1:3) = (Rot_i2b*absolute_traj(t_ix,1:3)')';
%                 % WRONG: need to remove rotational velocity!
%                 %relative_traj(t_ix,4:6) = (Rot_i2b*(absolute_traj(t_ix,4:6))')';
%                 % Fixed
%                 relative_traj(t_ix,4:6) = (Rot_i2b*(absolute_traj(t_ix,4:6) - cross(omega,absolute_traj(t_ix,1:3)))')';
%             end
        end
        
        function [time,relative_traj] = integrate_relative(obj,time_horizon,start_state_absolute)
            % Integrates the trajectory of a spacecraft orbiting the small
            % body.
            % Syntax: [tout, absolute_traj,relative_traj] = integrate_relative(time_horizon,start_state)
            % The spacecraft's starting state is start_state (x, y, z, vx,
            % vy, vz). The integration horizon is time_horizon.
            % The output is:
            % - tout, a grid of integration points in time.
            % - absolute_traj, the spacecraft location and velocity in the
            %    inertial body_centered grame
            % - relative_traj, the spacecraft location and velocity in the
            %    body_centered, body-fixed frame.
            rot_angle_z = obj.RotationEpoch+ obj.RotationRate * time_horizon(1);
            Rot_i2b = obj.rotmat(rot_angle_z,3);
            omega = [0;0;obj.RotationRate];
            
            start_state_relative = zeros(size(start_state_absolute));
            start_state_relative(1:3) = (Rot_i2b*(start_state_absolute(1:3)));
            start_state_relative(4:6) = (Rot_i2b*(start_state_absolute(4:6) - cross(omega,start_state_absolute(1:3))));
            
            [time,relative_traj] = obj.Integrator(@obj.dstate_rel,time_horizon,start_state_relative);
%             absolute_traj = zeros(size(relative_traj));
%             for t_ix=1:length(time)
%                 rot_angle_z = obj.RotationEpoch+ obj.RotationRate * time(t_ix);
%                 Rot_b2i = obj.rotmat(-rot_angle_z,3);
%                 absolute_traj(t_ix,1:3) = (Rot_b2i*relative_traj(t_ix,1:3)')';
%                 absolute_traj(t_ix,4:6) = (Rot_b2i*(relative_traj(t_ix,4:6) + cross(omega,relative_traj(t_ix,1:3)))')';
%             end
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
                figure()
            else
                figure(handle)
            end
            if nargin<4
                plotBody = 0;
            end
            traj = traj/1e3;  % Plot in km for consistency with SBDT
            
            plot3(traj(:,1),traj(:,2),traj(:,3));
            hold all
            if (plot_velocity==1)
                quiver3(traj(:,1),traj(:,2),traj(:,3),traj(:,4),traj(:,5),traj(:,6));
            end
            title('Absolute reference frame')
            axis equal
            
            if plotBody
                Model = importdata(obj.ShapeModel.shapefilename);
                vertexIdx = strcmp(Model.rowheaders, 'v');
                facetIdx = strcmp(Model.rowheaders, 'f');
                assert(all(vertexIdx == 1-facetIdx));
                vertices = Model.data(vertexIdx,:);
                if isfield(obj.ShapeModel,'scale')
                    vertices = vertices *obj.ShapeModel.scale;
                end
                facets = Model.data(facetIdx,:);
                facets = facets + 1;                 % Move to one-based indexing
                
                for t_ix=1:length(time)
                    rot_angle_z = obj.RotationEpoch+ obj.RotationRate * time(t_ix);
                    Rot_i2b = obj.rotmat(rot_angle_z,3);
                    vertices_abs = (Rot_i2b'*vertices')';
                    hold all
                    trimesh(facets,vertices_abs(:,1),vertices_abs(:,2),vertices_abs(:,3));
                    
                end
            end
            xlabel('X (km)')
            ylabel('Y (km)')
            zlabel('Z (km)')
            
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
                figure(handle)
            end
            if nargin<4
                plotBody = 0;
            end
            
            traj = traj/1e3;  % Plot in km for consistency with SBDT

            plot3(traj(:,1),traj(:,2),traj(:,3))
            hold all
            if (plot_velocity==1)
                quiver3(traj(:,1),traj(:,2),traj(:,3),traj(:,4),traj(:,5),traj(:,6));
            end
            hold all
            title('Relative reference frame')
            
            if plotBody
                Model = importdata(obj.ShapeModel.shapefilename);
                vertexIdx = strcmp(Model.rowheaders, 'v');
                facetIdx = strcmp(Model.rowheaders, 'f');
                assert(all(vertexIdx == 1-facetIdx));
                vertices = Model.data(vertexIdx,:);
                if isfield(obj.ShapeModel,'scale')
                    vertices = vertices *obj.ShapeModel.scale;
                end
                facets = Model.data(facetIdx,:);
                % Move to one-based indexing
                facets = facets + 1;
                trimesh(facets,vertices(:,1),vertices(:,2),vertices(:,3))
            end
            axis equal
            xlabel('X (km)')
            ylabel('Y (km)')
            zlabel('Z (km)')
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
    methods (Access = private)
        function [ds] = dstate_abs(obj,time,state)
            % Function for the integrator. Returns dx, dy, dz, ddx, ddy,
            % ddz.
            x = state(1);
            y = state(2);
            z = state(3);
            dx = state(4);
            dy = state(5);
            dz = state(6);
            dds = obj.g_absolute(time,[x;y;z]);
            ds = [dx; dy; dz; dds];
        end
        
        function [ds] = dstate_rel(obj,time,state)
            % Function for the integrator. Returns dx, dy, dz, ddx, ddy,
            % ddz.
            % \ddot{X} = - 2 \omega x \dot{X} - \omega x \omega x X + dU/dX
            omega = [0; 0; obj.RotationRate];
            x = state(1);
            y = state(2);
            z = state(3);
            dx = state(4);
            dy = state(5);
            dz = state(6);
            dds = -2* cross(omega, state(4:6)) - cross(omega,cross(omega,state(1:3))) + obj.g_relative(time,[x;y;z]);
            ds = [dx; dy; dz; dds];
        end
        
    end
end