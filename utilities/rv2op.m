%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%        Convert radius and velocity to orbital parameters.               %
%        Has a known singularity for circular orbits and orbits in the    %
%        equatorial plane.                                                %
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

function [a,e,i_anomaly,Omega,omega,theta]=rv2op(rvec,vvec,mu)

% Syntax: (a,e,i,Omega,omega,theta)=rv2op(r,v,mu);
% Computes orbital parameters (a,e,i,Omega,omega,theta) given radius r
%  and velocity v of a satellite in orbit.
% If unspecified, satellite is assumed to be in Earth orbit.
% The semimajor axis is expresses in m, angles are in radians.
% Positions are in m and velocities are is in m/s.
% mu is assumed to be in m^3/s^2,

if nargin<3
	%disp('Assuming we are on Earth')
    mu=3.986*1e14;
end
if size(rvec)~=[3 1] & size(rvec)~=[1 3]
	disp('Error: rvec must be a vector')
	return
end
if size(vvec)~=[3 1] & size(vvec)~=[1 3]
	disp('Error: vvec must be a vector')
	return
end

if size(rvec,1)==1
	rvec=rvec';
end
if size(vvec,1)==1
	vvec=vvec';
end

%0_ definisco i versori degli assi, mi aiuteranno a fare i conti.
iver=[1;0;0];
%jver=[0;1;0];
kver=[0;0;1];

%1_Calcolo i moduli di rvec e vvec
r=norm(rvec);
v=norm(vvec);

%2_Energia totale;
%E=0.5*v^2-mu/r;
%Trovo a dall'energia totale
a=mu/(2*mu/r-v^2);

%3_Definizione di h e modulo
hvec=cross(rvec,vvec);
h=norm(hvec);

%4_Trovo l'eccentricità
evec=((v^2-mu/r).*rvec-(dot(rvec,vvec).*vvec))./mu;
e=norm(evec);

%5_Trovo l'angolo i
i_anomaly=acos(hvec(3)/h);
% What if h is zero, i.e., rvec and vvec are aligned?

%6_trovo il versore dell'asse dei nodi
nver=cross(kver,hvec)./norm(cross(kver,hvec));
% What if the orbit is horizontal, i.e., h is aligned with z?
if isnan(nver)
    disp('WARNING: orbit is in x-y plane')
    nver = iver;
end
%7_trovo Omega
Omega=acos(dot(nver,iver));

%Sistemo il segno controllando il seno di omega
if nver(2)<0
	Omega=2*pi-Omega;
end

%8_trovo omega
if norm(evec) == 0
    % Orbit is circular
    omega=0;
else 
    omega=acos(dot(nver,evec)./norm(evec));
    % What if eccentricity is zero?
end
if evec(3)<0
	omega=2*pi-omega;
end


%9_trovo l'anomalia vera
theta=acos(rvec'*evec./(r*e));
% What if eccentricity is zero?


if dot(rvec,vvec)<0
	theta=2*pi-theta;
end

if isnan(a) || isnan(e) || isnan(i_anomaly) || isnan(Omega) || isnan(omega) || isnan(theta)
    disp("NAN")
end