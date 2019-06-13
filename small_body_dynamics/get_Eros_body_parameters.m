function [ErosProperty] = get_Eros_body_parameters(filename)
%F_GET_EROS_BODY_PARAMETERS Summary of this function goes here
%   Detailed explanation goes here

% Get properties from Eros gravity model 
[ErosProperty.Gravity.GM, ErosProperty.radius, ErosProperty.Gravity.degree, ...
    ErosProperty.Gravity.C, ErosProperty.Gravity.S] = readErosGravityModel( filename ); 

% Other 
ErosProperty.rotation_rate = 1639.389232 ... %degrees-day
    *pi/180 / 86400;

end

