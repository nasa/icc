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

function [bodyModel] = loadEros(constants, orbitNumber, frameNumber, ...
        shapeNumber, gravityNumber)
    %% LOADEROS - Creates models of the asteroid 433 Eros.
    %
    %  Derived from SBDT's loadVesta.
    %
    % Usage:   [bodyModel] = loadEros(constants, orbitNumber, ...
    %               frameNumber, shapeNumber, gravityNumber)
    %
    % Inputs:  
    %  constants -     Structure containing SBDT constant parameters
    %  orbitNumber -   Number indicating which orbit model to use.
    %                  (1)  Circular and Planar Orbit
    %                  (2)  Eccentric and Inclined Orbit (EME2000)
    %  frameNumber -   Number indicating which body fixed frame model to
    %                  use.
    %                  (1)  Body Frame (EMO2000)
    %  shapeNumber -   Number indicating which shape model to use.
    %                  (1)  Spherical Shape
    %                  (2)  Ellipsoid Shape
    %                  (3)  Polyhedral Shape (856 vertices, 1708 faces)
    %                  (4)  Polyhedral Shape (5078 vertices, 10152 faces)
    %  gravityNumber - Number indicating which gravity model to use.
    %                  (1)  Pointmass Gravity
    %                  (2)  Constant Density Gravity
    %                  (3)  Spherical Harmonic Gravity (n15a)
    %                  (4)  Spherical Harmonic Gravity (n393a)
    %
    % Outputs: 
    %  bodyModel -  Structure containing a SBDT body definition.
    %

    %% NOTES -----------------------------------------------------------------
    % 1)    N/A

    %% GENERAL PARAMETERS ----------------------------------------------------
    
    % Small-body name
    sbName = '433 Eros';
    disp('---------------------------------------------------------------');
    disp(cat( 2, 'Loading ', sbName, ' model with ...'));
    
    %% DEFINE ORBIT PARAMETERS -----------------------------------------------
    
    % Select general parameters model
    switch(orbitNumber)
      case 1
        % Orbit model name
        orModelName = 'Circular and Planar Orbit';
        disp(cat(2, '- ', orModelName, ' Model.'));
        
        % Small-body orbit
        conicOrbit = [1.458114857699829 * constants.au; ...   % semi-major axis (km)
            0.0; ...        % eccentricity
            0 * pi / 180; ...       % inclination (rad) 
            0 * pi / 180; ...       % longitude of the ascending node (rad)
            0 * pi / 180; ...       % argument of periapsis (rad)
            0];       % time of periapsis passage (sec)
        bodyModel.orbit.centerBody = 'Sun';
        bodyModel.orbit.frame = 'EME2000';
        
      case 2
        % Orbit model name
        orModelName = 'Eccentric and Inclined Orbit (EME2000)';
        disp(cat(2, '- ', orModelName, ' Model.'));
        
        % Small-body orbit
        % [ JPL Horizons system, 2019-May-2 solution, 2019-May-02 00:00:00.0000 epoch ]
        conicOrbit = [1.458114857699829 * constants.au; ...   sma (km)
            2.227354414120954E-01; ...        % eccentricity
            1.082855904976882E+01 * pi / 180; ...       % inclination (rad)
            3.043061397028930E+02 * pi / 180; ...    % longitude of the ascending node (rad)
            1.788220606936718E+02 * pi / 180; ...       % argument of periapsis (rad)
            cal2sec(char(datetime(2458516.111275572330,'convertfrom','juliandate')))]; 
                                         % time of periapsis passage (sec)
                                         %  cal2sec('23-sep-2014 16:40:59.611209')
        bodyModel.orbit.centerBody = 'Sun';
        bodyModel.orbit.frame = 'EME2000';
        
      otherwise
        % Invalid orientation model number
        error('Invalid orbit model number.');
        
    end;    
               
    %% DEFINE BODY FRAME PARAMETERS ------------------------------------------
    
    % Select general parameters model
    switch(frameNumber) 
      case 1
        % Orientation parameters model name
        bfModelName = 'Body Frame (EME2000)';
        disp(cat(2, '- ', bfModelName, ' Model.'));
        bodyModel.bodyFrame.refFrame = 'EME2000';
        
        % Pole Orientation
        % Reference publication:
        %   Miller, J.K., A.S. Konopliv, P.G. Antreasian, J.J. Bordi,
        %   S. Chesley, C.E. Helfrich, W.M. Owen, T.C.Wang, B.G. Williams,
        %   D.K. Yeomans and D.J. Scheeres, Determination of Shape, Gravity
        %   and Rotational State of Asteroid 433 Eros,
        %    Icarus, 155, 3-17, 2002.
        %    https://www.sciencedirect.com/science/article/pii/S0019103501967533
        bodyModel.bodyFrame.pole.dec0 = 17.2273; % DEC (deg)
        bodyModel.bodyFrame.pole.decdot = 0;  % DEC rate of change 
                                               %(deg/century)
        bodyModel.bodyFrame.pole.ra0 = 11.3692;  % RA (deg)
        bodyModel.bodyFrame.pole.radot = 0;  % RA rate of change (deg/century)
                                               
%         poleVec_eme2000 = [cos(dec) * cos(ra); cos(dec) * sin(ra); sin(dec)];
%         R_eme2emo = [1., 0, 0;
%               0.,  9.174820620691818e-01,  3.977771559319137e-01;
%               0., -3.977771559319137e-01,  9.174820620691818e-01;];
%         poleVec_emo2000 = R_eme2emo * poleVec_eme2000;
%         bodyModel.bodyFrame.pole.dec0 = asin(poleVec_eme2000(3)) * 180 / pi;
%                                         % DEC (deg)
%         bodyModel.bodyFrame.pole.decdot = 0;  % DEC rate of change 
%                                                %(deg/century)
%         bodyModel.bodyFrame.pole.ra0 = atan2(poleVec_emo2000(2), ...
%             poleVec_emo2000(1)) * 180 / pi;  % RA (deg)
%         bodyModel.bodyFrame.pole.radot = 0;  % RA rate of change (deg/century)
        
        % Body rotation
        % Same Icarus reference as above
        bodyModel.bodyFrame.pm.w0 = 326.06;  % prime meridian at J2000 (deg)
        bodyModel.bodyFrame.pm.wdot = 1639.38922;  % prime meridian rate 
                                                     % (deg/day)        
        
      otherwise
        % Invalid body frame model number
        error('Invalid body model number.');
        
    end;               

    %% DEFINE SHAPE PARAMETERS -----------------------------------------------

    % Select shape parameters model
    switch( shapeNumber )
      case 1
        % Orientation parameters model name
        shapeName = 'Spherical Shape';
        % [ Mean radius derived from Miller, et al. 2002, Icarus, 155, 3-17. ]
        disp(cat(2, '- ', shapeName, ' Model.'));
        
        % Shape information
        shapeInfo.shapeType = 1;
        eros_volume = 2503; %km^3
        eros_radius = (3/(4*pi)*eros_volume)^(1/3);
        shapeInfo.radius = eros_radius;    % km
        
        % Load ancillary information
        bodyModel.shape = makeShapeModel(shapeInfo);
    
      case 2
        % Orientation parameters model name
        shapeName = 'Ellipsoid Shape';
        % [ Ellipsoid from https://nssdc.gsfc.nasa.gov/planetary/text/eros.txt. ]
        disp(cat(2, '- ', shapeName, ' Model.'));
        
        % Shape information
        shapeInfo.shapeType = 2;
        shapeInfo.ellipsoid = [33, 13, 13];
                                % 3x1 semi-major axes (km)
        
        % Load ancillary information
        bodyModel.shape = makeShapeModel(shapeInfo);

      case 3
        % Orientation parameters model name
        shapeName = 'Polyhedral Shape (856 vertices)';
        % MSI data from https://sbn.psi.edu/pds/resource/nearbrowse.html
        disp(cat(2, '- ', shapeName, ' Model.'));
        
        % Read in data
        vertices_file = 'MSI_optical_plate_models/V_eros001708_vertices.txt';
        facets_file   = 'MSI_optical_plate_models/F_eros001708_facets.txt';
        if ~exist(facets_file,'file') || ~exist(vertices_file,'file')
            try
                create_MSI_facet_files('eros001708');
            catch
                fprintf('ERROR: vertices and facets file for eros001708 do not exist.\ncd to small_body_dynamics/EROS 433/MSI_optical_shape_Models\nand run create_MSI_facet_files()')
                return
            end
        end
        % Shape information
        shapeInfo.shapeType = 3;
        shapeInfo.DC = 1;
        shapeInfo.faceFile = facets_file;
        shapeInfo.vertexFile = vertices_file;
        
        % Load ancillary information
        bodyModel.shape = makeShapeModel(shapeInfo);
        
      case 4
        % Orientation parameters model name
        shapeName = 'Polyhedral Shape (5078 vertices)';
        % MSI data from https://sbn.psi.edu/pds/resource/nearbrowse.html
        disp(cat(2, '- ', shapeName, ' Model.'));
        
        % Read in data
        vertices_file = 'MSI_optical_plate_models/V_eros010152_vertices.txt';
        facets_file   = 'MSI_optical_plate_models/F_eros010152_facets.txt';
        if ~exist(facets_file,'file') || ~exist(vertices_file,'file')
            try
                create_MSI_facet_files('eros010152');
            catch
                fprintf('ERROR: vertices and facets file for eros010152 do not exist.\ncd to small_body_dynamics/EROS 433/MSI_optical_shape_Models\nand run create_MSI_facet_files("eros010152")')
                return
            end
        end
        % Shape information
        shapeInfo.shapeType = 3;
        shapeInfo.DC = 1;
        shapeInfo.faceFile = facets_file;
        shapeInfo.vertexFile = vertices_file;
        
        % Load ancillary information
        bodyModel.shape = makeShapeModel(shapeInfo);
            
      otherwise
        % Invalid orientation model number
        error('Invalid shape model number.');
        
    end;

    %% DEFINE GRAVITY PARAMETERS ---------------------------------------------

    % Select gravity parameters model
    switch(gravityNumber)
      case 1
        % Gravity parameters model name
        gravModelName = 'Pointmass Gravity';
        % [ Miller, et al. 2002, Icarus, 155, 3-17. ]
        disp(cat(2, '- ', gravModelName, ' Model.'));
        
        % Gravity information
        gravInfo.gravType = 1;
        gravInfo.gravParam = 4.4631*1e-4;   % km^3/s^2
        gravInfo.sphereOfInfluence = conicOrbit(1) ...
            * (gravInfo.gravParam / constants.gmSun)^(0.4);
        
        % Load ancillary information
        bodyModel.gravity = makeGravityModel(gravInfo, ...
            constants, bodyModel.shape);
        
      case 2
        % Gravity parameters model name
        gravModelName = 'Constant Density Gravity';
        % [ Miller, et al. 2002, Icarus, 155, 3-17. ]
        disp(cat(2, '- ', gravModelName, ' Model.'));
            
        % Gravity information
        gravInfo.gravType = 2;
        gravInfo.density = 2.67e+12;    % kg / km^3
        gravInfo.sphereOfInfluence = conicOrbit(1) ...
            * (17.8 / constants.gmSun)^(0.4);
        
        % Load ancillary information
        bodyModel.gravity = makeGravityModel(gravInfo, ...
            constants, bodyModel.shape);  
                
      case 3
        % Gravity parameters model name
        gravModelName = 'Spherical Harmonic Gravity (n15a)';
        % [ Derived from E.M. Standish, JPL-IOM-312.F-01-006, 2001 and
        %   Thomas, et al. 1997, Icarus 128, 88-94 assuming constant density. ]
        disp(cat(2, '- ', gravModelName, ' Model.'));
        
        [GM, Re, degree, C, S] = readErosGravityModel('Gravity_models/n15acoeff.tab');
        
        % Gravity information
        gravInfo.gravType = 3;
        gravInfo.normalized = 1;    % Normalized coefficients
        gravInfo.gm = GM*1e-9;		% Gravitational Parameter (km^3/s^2)
        gravInfo.sphHarmRefRadius = Re*1e-3; % Ref Radius (km)
        gravInfo.sphHarmOrder = degree;     % Order and Degree of expansion
        gravInfo.sphHarmType = 'sbdt';  % must be 'sbdt'
        gravInfo.sphereOfInfluence = conicOrbit(1) ...
            * (gravInfo.gm / constants.gmSun)^(0.4);
        
        % C Coefficients
        i=1;
        C(1,1) = 1;
        gravInfo.Cnm = C;
        
        % S coefficients 
        gravInfo.Snm = S;

        % Create CS matrix (for ODP spherical harmonics format)
        gravInfo.J = nan(gravInfo.sphHarmOrder, 1);
        gravInfo.CS = nan(gravInfo.sphHarmOrder + 1, gravInfo.sphHarmOrder);
        for n = 1:gravInfo.sphHarmOrder
            for m = 1:n
                gravInfo.J(n) = -C(i + n, 1);
                gravInfo.CS(m, n) = C(i + n, i + m);
                gravInfo.CS(n + 1, m) = S(i + n, i + m);
            end;
        end;
               
        % Load ancillary information
        bodyModel.gravity = makeGravityModel(gravInfo, ...
            constants, bodyModel.shape);
        
      case 4
        % Gravity parameters model name
        gravModelName = 'Spherical Harmonic Gravity (n393a)';
        % [ Derived from E.M. Standish, JPL-IOM-312.F-01-006, 2001 and
        %   Thomas, et al. 1997, Icarus 128, 88-94 assuming constant density. ]
        disp(cat(2, '- ', gravModelName, ' Model.'));
        
        [GM, Re, degree, C, S] = readErosGravityModel('Gravity_models/n393coeff.tab');
        
        % Gravity information
        gravInfo.gravType = 3;
        gravInfo.normalized = 1;    % Normalized coefficients
        gravInfo.gm = GM*1e-9;		% Gravitational Parameter (km^3/s^2)
        gravInfo.sphHarmRefRadius = Re*1e-3; % Ref Radius (km)
        gravInfo.sphHarmOrder = degree;     % Order and Degree of expansion
        gravInfo.sphHarmType = 'sbdt';  % must be 'sbdt'
        gravInfo.sphereOfInfluence = conicOrbit(1) ...
            * (gravInfo.gm / constants.gmSun)^(0.4);
        
        % C Coefficients
        i=1;
        C(1,1) = 1;
        gravInfo.Cnm = C;
        
        % S coefficients 
        gravInfo.Snm = S;

        % Create CS matrix (for ODP spherical harmonics format)
        gravInfo.J = nan(gravInfo.sphHarmOrder, 1);
        gravInfo.CS = nan(gravInfo.sphHarmOrder + 1, gravInfo.sphHarmOrder);
        for n = 1:gravInfo.sphHarmOrder
            for m = 1:n
                gravInfo.J(n) = -C(i + n, 1);
                gravInfo.CS(m, n) = C(i + n, i + m);
                gravInfo.CS(n + 1, m) = S(i + n, i + m);
            end;
        end;
               
        % Load ancillary information
        bodyModel.gravity = makeGravityModel(gravInfo, ...
            constants, bodyModel.shape);
        
      otherwise
        % Invalid gravity model number
        error('Invalid gravity model number.');
        
    end;     

    %% CONVERT TO EME2000 if needed
    if strcmp(bodyModel.orbit.frame, 'EMO2000')
        conicOrbit = orbitEMOtoEME(constants, conicOrbit) ;
        bodyModel.orbit.frame = 'EME2000' ;
    end
    if strcmp(bodyModel.bodyFrame.refFrame, 'EMO2000')
       bodyModel.bodyFrame = bodyFrameEMOtoEME(constants, bodyModel.bodyFrame) ;
       bodyModel.bodyFrame.refFrame = 'EME2000';
    end
    
    %% DERIVED ASTEROID PROPERTIES -------------------------------------------
    
    % Rotation rate
    bodyModel.bodyFrame.pm.w = bodyModel.bodyFrame.pm.wdot ...
        * pi / 180 / 86400;  % rad/sec
    
    % Resonance radius 
    resonance = (bodyModel.gravity.gm / bodyModel.bodyFrame.pm.w^2)^(1 / 3);
    
    % Small-body orbit mean motion (rad/sec)
    N_mean = ((constants.gmSun + bodyModel.gravity.gm) / conicOrbit(1)^3)^.5;
    
    % Small-body orbit period (sec)
    orbit_period = 2 * pi / N_mean;
    
    % Small-body orbit parameter (km)
    p = conicOrbit(1) * (1 - conicOrbit(2)^2);
   
    %% RECORD OUTPUTS --------------------------------------------------------
    
    bodyModel.sbName = sbName;
    bodyModel.bodyFrame.pm.period = 2 * pi / bodyModel.bodyFrame.pm.w / 3600;
    bodyModel.orbit.conicOrbit = conicOrbit;
    bodyModel.char.resonance = resonance;
    bodyModel.orbit.N_mean = N_mean;
    bodyModel.orbit.period = orbit_period;
    bodyModel.orbit.p = p;

    disp('Done.');
    disp(' ');  
    
    %% END OF LOADEROS ------------------------------------------------------

end
