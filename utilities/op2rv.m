%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%        Convert orbital parameters to radius and velocity.               %
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

function [rvec,vvec]=op2rv(a,e,i_anomaly,Omega,omega,theta,mu)

% Syntax: [rvec,vvec]=op2rv(a,e,i_anomaly,Omega,omega,theta,mu)
% Given orbital parameters a,e,i,Omega,omega,theta of a satellite
% in a geo(or helio-, or whateverbodywoulike-)centric orbit,
% computes radius r and velocity v in an equatorial frame.
% Input a is assumed to be in m, mu is assumed to be in m^3/s^2,
% angles are in radians.
% 

if nargin<7
    disp('Assuming we are on Earth')
    mu=3.986*1e14;
end

%if e~=1
par=a*(1-e^2);
%else
%h=
%par=h^2/mu
%end



r=par/(1+e*cos(theta));
rvec=[r*cos(theta); r*sin(theta); 0];

vr=sqrt(mu/(a*(1-e^2)))*e*sin(theta);
vt=sqrt(mu/(a*(1-e^2)))*(1+e*cos(theta));

vvec=[vr*cos(theta)-vt*sin(theta); vr*sin(theta)+vt*cos(theta);0];

T=rotmat(omega,3)*rotmat(i_anomaly,1)*rotmat(Omega,3);

rvec=T'*rvec;
vvec=T'*vvec;