function [ErosParameters] = get_Eros_body_parameters(filename)
%F_GET_EROS_BODY_PARAMETERS Takes in a file containing the gravitational
%parameters and returns a struct containing the relevane body parameters
%needed for use in ICC sim. This function may be copied and modified for
%loading other small body parameters. 
%   Syntax: [ErosParameters] = get_Eros_body_parameters(*filename)
%    *optional
%
%   Inputs: 
%    - filename: A string containing the location of the *.tab file
%      specifying the gravitational parameters of Eros 
%       * Default Value: 'EROS 433/Gravity_models/n15acoeff.tab'
% 
%   Outputs: 
%    - ErosParameters : A struct containing the following fields: 
%       Gravity.GM    : [m^3/s^2] Gravitational parameter of Eros 
%       Gravity.degree: The maximum degree of the spherical harmonic 
%                        gravity model.
%       Gravity.C     : A (DEGREE+1)-by-(DEGREE+1) matrix containing the 
%                        normalized Cnm coefficients of the spherical 
%                        harmonic gravity model.
%       Gravity.S     : A (DEGREE+1)-by-(DEGREE+1) matrix containing the 
%                        normalized Snm coefficients of the spherical 
%                        harmonic gravity model.
%       radius        : [m] Equatorial radius of Eros in meters
%       rotation_rate : [rad/s] Rotation rate of Eros 

if nargin < 1 
    filename = 'EROS 433/Gravity_models/n15acoeff.tab';
end

% Get properties from Eros gravity model 
[ErosParameters.Gravity.GM, ErosParameters.radius, ErosParameters.Gravity.degree, ...
    ErosParameters.Gravity.C, ErosParameters.Gravity.S] = readErosGravityModel( filename ); 

% Other 
ErosParameters.rotation_rate = 1639.389232 ... %degrees/day
    *pi/180 / 86400; % result is in rad/s

end

