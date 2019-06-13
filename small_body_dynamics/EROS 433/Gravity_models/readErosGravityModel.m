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

function [GM, Re, degree, C, S] = readErosGravityModel( filename )
%
%   Output includes:
%   GM     :a scalar value specifying the planetary gravitational parameter
%          in meters cubed per second squared.  
%   RE     :a scalar value specifying the planetary equatorial radius in
%          meters. 
%   DEGREE :a scalar value specifying the maximum degree of the spherical
%          harmonic gravity model.
%   C      :a (DEGREE+1)-by-(DEGREE+1) matrix containing the normalized Cnm
%          coefficients of the spherical harmonic gravity model.
%   S      :a (DEGREE+1)-by-(DEGREE+1) matrix containing the normalized Snm
%          coefficients of the spherical harmonic gravity model.

%   Adapted from astReadEGMFile.m

% Constants from the accompanying .lbl file
assert(length(filename)>=12,'ERROR: gravity model not recognized')

if strcmp(filename(length(filename)-12:end),'n15acoeff.tab')
    GM = 4.46275472004E-04*1e9; % meters^3/s^2
    Re = 16000.0;       % meters
elseif strcmp(filename(length(filename)-12:end),'n393coeff.tab')
    GM = 4.46176546159E+05;
    Re = 1600;
else
    disp('ERROR: gravity model not recognized')
    assert(-1,'ERROR: gravity model not recognized')
end
    
    
% open data file 
fid = fopen( filename, 'r' );
% read data matrix
if strcmp(filename(length(filename)-12:end),'n15acoeff.tab')
    matrix = fscanf(fid, '%d %d %e %e %e %e', [6 inf] );
elseif strcmp(filename(length(filename)-12:end),'n393coeff.tab')
    matrix = fscanf(fid, '%d %d %e %e', [4 inf] );
end
% close data file
fclose(fid);

% set degree index (zero vs one based indices)
n = matrix(1,:) + 1;
% set order index  (zero vs one based indices)
m = matrix(2,:) + 1;

C = zeros(n(end),n(end));
S = zeros(n(end),n(end));
% put normalized coefficients into matrices
for k = length(n):-1:1
    C(n(k),m(k)) = matrix(3,k);
    S(n(k),m(k)) = matrix(4,k);
end

degree = n(end) - 1;
